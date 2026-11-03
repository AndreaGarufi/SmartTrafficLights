/**
 * @file main.cpp
 * @brief ESP32 two-node communication using ESP-NOW library and IoTBoard
 * library.
 *
 * What this node does:
 *  1. Periodically broadcasts its MAC so other nodes can discover it.
 *  2. Keeps a small peer list (add on first contact, remove on timeout).
 *  3. Runs a 3-step time-sync handshake (T0 -> T1 -> T2) with each peer to
 *     compute `clockOffset`, so the traffic-light "semaphore" is in phase
 *     with the leader.
 *
 * Threading rule: the ESP-NOW receive callback runs in the Wi-Fi task. It may
 * only handle sync messages and enqueue MACs; the peer list for discovery is
 * modified only from loop().
 */

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <iot_board.h>

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

// ============================================================================
// Configuration
// ============================================================================

constexpr unsigned long BROADCAST_INTERVAL_MS = 6000;
constexpr unsigned long DISPLAY_REFRESH_MS    = 6000;
constexpr unsigned long PEER_TIMEOUT_MS       = 12000;
constexpr unsigned long PEER_SYNC_RQ_EXPR_MS = 3000;


constexpr uint8_t ESPNOW_CHANNEL = 1;  // Same channel as the ESP8266 node

constexpr int MAX_MESH_NODES = 4;
constexpr int MACLEN         = 6;
constexpr bool isLeader = false; // true = leader; false = follower

// Semaphore (traffic light) timing
constexpr uint32_t GREEN_TIME_MS  = 2000;
constexpr uint32_t YELLOW_TIME_MS = 1000;
constexpr uint32_t RED_TIME_MS    = 3000;
constexpr uint32_t SEM_LOOP_TIME_MS =
    GREEN_TIME_MS + YELLOW_TIME_MS + RED_TIME_MS;
constexpr int SEM_STATES = 3;
constexpr int PHASE_OFFSET = 0;
// constexpr int PHASE_OFFSET = SEM_LOOP_TIME_MS/2;



// ============================================================================
// Types
// ============================================================================

typedef enum {
  YELLOW = 0,
  GREEN  = 1,
  BLUE   = 2,
  OFF    = 3
} rgb_color;

typedef enum {
  SEM_GREEN  = 0,
  SEM_YELLOW = 1,
  SEM_RED    = 2
} SemState;

/** Step of the sync handshake. Also used as index into Peer::timestamps. */
typedef enum {
  T0 = 0,
  T1 = 1,
  T2 = 2,
  T3 = 3,
  FIN = 4
} SyncStep;

/** Sync message sent over ESP-NOW. Must match the layout on the other node. */
typedef struct __attribute__((packed)) {
  SyncStep step;
  uint32_t curMs;  // Sender's millis() when the message was built
} SyncMsg;

typedef struct {
  uint8_t peerMac[MACLEN];
  unsigned long lastPeerTime;  // millis() of the last message seen

  bool synced;             // True if synchronized with that node
  uint32_t timestamps[4];  // Handshake timestamps [T0, T1, T2, T3]
} Peer;

/** Item passed from the ESP-NOW callback to loop(). */
struct PeerEvent {
  uint8_t mac[MACLEN];
};

// ============================================================================
// Global state
// ============================================================================

Peer peerList[MAX_MESH_NODES];
int numPeer = 0;

uint8_t deviceMac[MACLEN] = {0};
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

uint32_t clockOffset = 0;  // leader time = millis() + clockOffset

static int semState = SEM_GREEN;
static bool inSynchronization = false;

// Queue used to hand received MACs from the Wi-Fi task to loop()
static const size_t kRxPeerQueueLen = 16;
static QueueHandle_t rxPeerQueue = nullptr;

// ============================================================================
// Peer list management
// ============================================================================

/** @brief Clear every slot of the peer list. */
void initPeerList(Peer* peerL) {
  for (int i = 0; i < MAX_MESH_NODES; i++) {
    memset(peerL[i].peerMac, 0, MACLEN);
    peerL[i].lastPeerTime = 0;
    peerL[i].synced = false;
    for (int j = 0; j < 4; j++)
      peerL[i].timestamps[j] = 0;
  }
}

/** @brief Return the index of the given MAC in the peer list, -1 if absent. */
int getMacIdx(const uint8_t* mac_addr) {
  for (int i = 0; i < numPeer; i++) {
    if (memcmp(mac_addr, peerList[i].peerMac, MACLEN) == 0)
      return i;
  }
  return -1;
}

/** @brief Add the given MAC to the peer list and register it with ESP-NOW. */
void addMac(const uint8_t* mac_addr) {
  if (numPeer >= MAX_MESH_NODES) return;

  Peer& peer = peerList[numPeer];
  memcpy(peer.peerMac, mac_addr, MACLEN);
  peer.lastPeerTime = millis();
  peer.synced = false;
  numPeer++;

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, peer.peerMac, MACLEN);
  peerInfo.channel = ESPNOW_CHANNEL;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != 0) {
    Serial.printf("Failed to add [" MACSTR "] peer\n", MAC2STR(peer.peerMac));
  }
}

/** @brief Remove the peer at idx and compact the list. */
void rmMac(int idx) {
  if (idx < 0 || idx >= numPeer) return;

  esp_now_del_peer(peerList[idx].peerMac);

  // Shift following entries down by one
  for (int j = idx; j < numPeer - 1; j++) {
    memcpy(peerList[j].peerMac, peerList[j + 1].peerMac, MACLEN);
    peerList[j].lastPeerTime = peerList[j + 1].lastPeerTime;
  }

  // Clear the now-unused last slot
  numPeer--;
  memset(peerList[numPeer].peerMac, 0, MACLEN);
  peerList[numPeer].lastPeerTime = 0;
}

/** @brief Remove peers that have not been heard from recently. */
void checkTimeouts() {
  for (int i = 0; i < numPeer; i++) {
    if (millis() - peerList[i].lastPeerTime > PEER_TIMEOUT_MS) {
      Serial.printf("Peer timed out:" MACSTR, MAC2STR(peerList[i].peerMac));
      rmMac(i);
      i = i - 1;  // Re-check this index: the list was compacted
    }
  }
}

/** @brief Add a new peer or refresh the timestamp of a known one. */
static void processRxPeer(const uint8_t* mac) {
  int idx = getMacIdx(mac);
  if (idx == -1) {
    addMac(mac);
  } else {
    peerList[idx].lastPeerTime = millis();
  }

  Serial.printf("Received broadcast from: " MACSTR "\n", MAC2STR(mac));
}

// ============================================================================
// Display and LEDs
// ============================================================================

/** @brief Updates the OLED display with the current connection status. */
void updateDisplayStatus() {
  display->clearDisplay();
  display->setCursor(0, 0);

  if (numPeer >= 1) {
    display->print("Connected\n");
  } else {
    display->print("Not Connected\n");
  }

  display->setCursor(0, 9);
  display->printf("MAC:" MACSTR, MAC2STR(deviceMac));

  for (int i = 0; i < numPeer; i++) {
    display->setCursor(0, 18 + 9 * i);
    display->printf("N%d:" MACSTR, i, MAC2STR(peerList[i].peerMac));
  }

  display->display();
}

/** @brief Set the RGB status LED (active-low channels on this board). */
void setRgbLed(rgb_color c) {
  switch (c) {
    case YELLOW:
      digitalWrite(LED_RGB_R, LOW);
      digitalWrite(LED_RGB_G, LOW);
      digitalWrite(LED_RGB_B, HIGH);
      break;
    case GREEN:
      digitalWrite(LED_RGB_R, HIGH);
      digitalWrite(LED_RGB_G, LOW);
      digitalWrite(LED_RGB_B, HIGH);
      break;
    case BLUE:
      digitalWrite(LED_RGB_R, HIGH);
      digitalWrite(LED_RGB_G, HIGH);
      digitalWrite(LED_RGB_B, LOW);
      break;
    case OFF:
      digitalWrite(LED_RGB_R, HIGH);
      digitalWrite(LED_RGB_G, HIGH);
      digitalWrite(LED_RGB_B, HIGH);
      break;
  }
}

// ============================================================================
// Semaphore (traffic light)
// ============================================================================

static void applySemState(int s) {
  digitalWrite(LED_GREEN,  s == SEM_GREEN);
  digitalWrite(LED_YELLOW, s == SEM_YELLOW);
  digitalWrite(LED_RED,    s == SEM_RED);
}

/** @brief Pick the light from the (leader-aligned) time and update the LEDs. */
void SemaphoreLoop() {
  uint32_t leaderTime = millis() + clockOffset;
  uint32_t t = (millis() + clockOffset + PHASE_OFFSET) % SEM_LOOP_TIME_MS;

  int s;
  if (t < GREEN_TIME_MS)                       s = SEM_GREEN;
  else if (t < GREEN_TIME_MS + YELLOW_TIME_MS) s = SEM_YELLOW;
  else                                         s = SEM_RED;

  if (s != semState) {
    semState = s;
    applySemState(s);
  }
}

// ============================================================================
// Time sync handshake
// ============================================================================

/** @brief Send a sync message of the given step to one peer. */
static void sendSyncMsg(const uint8_t* mac, SyncStep step, uint32_t curMs) {
  SyncMsg msg;
  msg.step = step;
  msg.curMs = curMs;
  esp_now_send(mac, (uint8_t*)&msg, sizeof(msg));
}

static void updateClockOffset(const Peer& peer) {
  uint32_t t0 = peer.timestamps[T0];
  uint32_t t1 = peer.timestamps[T1];
  uint32_t t2 = peer.timestamps[T2];
  uint32_t t3 = peer.timestamps[T3];

  int32_t d1 = (int32_t)(t0 - t1);
  int32_t d2 = (int32_t)(t2 - t3);
  int32_t tripTime = ((int32_t)(t2 - t0) + (int32_t)(t3 - t1)) / 4;

  clockOffset = (d1 + d2) / 2 + tripTime;
}

/**
 * @brief Handle one step of the sync handshake.
 * @param rxTimeMs millis() at the moment the frame was received
 */
static void handleSyncMessage(const uint8_t* mac, const SyncMsg& m,
                              uint32_t rxTimeMs) {
  int idx = getMacIdx(mac);
  if (idx == -1) return;  // Unknown node: ignore
  Peer& peer = peerList[idx];

  switch (m.step) {
    case T0:
      sendSyncMsg(mac, T1, rxTimeMs);
      peer.timestamps[T0] = m.curMs;
      peer.timestamps[T1] = rxTimeMs;
      break;

    case T1:
      sendSyncMsg(mac, T2, rxTimeMs);
      peer.timestamps[T1] = m.curMs;
      peer.timestamps[T2] = rxTimeMs;
      break;

    case T2:
      sendSyncMsg(mac, FIN, 0);
      peer.timestamps[T2] = m.curMs;
      peer.timestamps[T3] = rxTimeMs;
      updateClockOffset(peer);
      Serial.printf("Synchronized with" MACSTR "\n", MAC2STR(mac));
      break;

    case FIN:
      peer.synced = true;
      Serial.printf("Synchronized with" MACSTR "\n", MAC2STR(mac));
      break;

    default:
      break;
  }

  Serial.printf("Received data from: " MACSTR " | Data:%lu | Status: %d\n",
                MAC2STR(mac), m.curMs, m.step);
}

// ============================================================================
// ESP-NOW callbacks
// ============================================================================

/** @brief Callback when data is sent via ESP-NOW. */
void OnDataSent(const uint8_t* mac_addr, esp_now_send_status_t status) {
  // Optional: handle send status
}

/**
 * @brief Callback when data is received via ESP-NOW (Wi-Fi task).
 *
 * Two kinds of frames are accepted:
 *  - SyncMsg (sizeof(SyncMsg) bytes): handled immediately.
 *  - Bare MAC (MACLEN bytes): queued for loop() to add/refresh the peer.
 */
void OnDataRecv(const uint8_t* mac, const uint8_t* incomingData, int len) {
  uint32_t rxTimeMs = millis();

  // Ignore frames from ourselves
  if (memcmp(mac, deviceMac, MACLEN) == 0) return;

  if (len == sizeof(SyncMsg)) {
    SyncMsg m;
    memcpy(&m, incomingData, sizeof(m));
    handleSyncMessage(mac, m, rxTimeMs);
    return;
  }

  if (len == MACLEN) {
    // Hand the MAC off to loop(). Drop the frame if the queue is full.
    PeerEvent evt;
    memcpy(evt.mac, mac, MACLEN);
    xQueueSend(rxPeerQueue, &evt, 0);
  }
}

// ============================================================================
// Periodic tasks (called from loop)
// ============================================================================

/** @brief Process every MAC queued by the receive callback. */
static void drainRxPeerQueue() {
  PeerEvent evt;
  while (xQueueReceive(rxPeerQueue, &evt, 0) == pdTRUE) {
    processRxPeer(evt.mac);
  }
}

/** @brief Start a sync handshake (T0) with every peer not yet synced. */
static void sendSyncRequests() {
  SyncMsg t0Msg;
  t0Msg.step = T0;
  t0Msg.curMs = millis();

  for (int i = 0; i < numPeer; i++) {
    Peer& p = peerList[i];
    if (!p.synced && !inSynchronization && (millis() - p.timestamps[0] > PEER_SYNC_RQ_EXPR_MS)) {
      p.timestamps[0] = millis();
      inSynchronization = true;
      esp_now_send(p.peerMac, (uint8_t*)&t0Msg, sizeof(t0Msg));
    } 
    else {
      inSynchronization = false;
    }
  }
}

/** @brief Expire old peers, announce ourselves, and start syncs. */
static void broadcastPresence() {
  checkTimeouts();
  esp_now_send(broadcastAddress, deviceMac, MACLEN);
  if (isLeader)
    sendSyncRequests();
}

// ============================================================================
// Setup
// ============================================================================

static bool createRxQueue() {
  rxPeerQueue = xQueueCreate(kRxPeerQueueLen, sizeof(PeerEvent));
  if (rxPeerQueue == nullptr) {
    Serial.println("Failed to create RX peer queue");
    return false;
  }
  return true;
}

static bool initEspNow() {
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return false;
  }

  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  // Register the broadcast address as a peer
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, broadcastAddress, MACLEN);
  peerInfo.channel = ESPNOW_CHANNEL;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add broadcast peer");
    return false;
  }
  return true;
}

void setup() {
  // Board peripherals
  IoTBoard::init_serial();
  IoTBoard::init_leds();
  IoTBoard::init_display();
  setRgbLed(BLUE);

  initPeerList(peerList);

  // Initial display clear
  display->clearDisplay();
  display->setCursor(0, 0);
  display->display();

  // Wi-Fi: read local MAC, fix channel for compatibility with the ESP8266
  WiFi.mode(WIFI_STA);
  WiFi.macAddress(deviceMac);
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);

  Serial.print("INDIRIZZO MAC: ");
  Serial.printf(MACSTR, MAC2STR(deviceMac));
  Serial.println();

  if (!createRxQueue()) return;
  if (!initEspNow()) return;

  updateDisplayStatus();
}

// ============================================================================
// Main loop
// ============================================================================

void loop() {
  drainRxPeerQueue();

  // Status LED: green when connected to at least one peer, blue otherwise
  if ((numPeer >= 1) && isLeader)
    setRgbLed(YELLOW);
  else if ((numPeer >= 1) && !isLeader)
    setRgbLed(GREEN);
  else
    setRgbLed(BLUE);

  if (IoTBoard::runEvery(DISPLAY_REFRESH_MS))
    updateDisplayStatus();

  if (IoTBoard::runEvery(BROADCAST_INTERVAL_MS, 1))
    broadcastPresence();

  SemaphoreLoop();
}