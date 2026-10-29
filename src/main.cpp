//NOTA! 
//con la funzione broadcast() ogni scheda manda il messaggio anche a se stessa quindi se lo
//avvii con una singola scheda vedrai comunque il suo indirizzo mac stampato sul display
//aggiornamento: l'ho segnalato con la scritta: "MY MAC"

//Il led RGB per qualche motivo funziona al contrario: LOW ->accende il led, HIGH -> spegne il led

//funzioni utilizzate: 
//espmesh->getMac() ->ritorna il mac del singolo dispositivo
//espmesh->isRoot() ->ritorna true se il dispositivo è root
//espmesh->isConnected() ->ritorna true se il dispositivo è connesso

//PROMEMORIA
//continua con il salvataggio nell'array di tutti i mac che arrivano + quello della scheda


//INFO!
//LED RGB GIALLO -> Root
//LED RGB VERDE -> Connesso
//LED RGB SPENTO -> NON connesso

#include <iot_board.h>
//#include <WiFi.h>
#include <sys/socket.h>

#define MAX_STATION 2

const char *SSID = " Galaxy S21 5G";  //SSID proprio AP
const char *PASS = "campobasso12345"; //password proprio AP

uint8_t mac[6] = {0};
uint8_t MAC_GARA[MAX_STATION][6] = {0};     //inizialmente sono 2 poi dovrà essere modificato a 4
uint8_t numeroElementi = 0;


bool macConsegnato = false;


bool salvataggioMAC(const uint8_t *data, size_t len){
  
  bool presente = false;
  for(int i = 0; i < numeroElementi; i++){
    //scorro tutto il ciclo for per capire se il mac arrivato è nuovo oppure no

    if (memcmp(MAC_GARA[i], data, 6) == 0) {
      // Il mac address corrisponde a quello memorizzato in MAC_GARA[i]
      Serial.println("MAC GIÀ RICEVUTO.\n");
      presente = true;
      break;
    }
  }

  if(presente == false){

    if(numeroElementi >= MAX_STATION){
      Serial.println("ARRAY PIENO.\n");
      return false; //array pieno non mi sevono altri mac
    }

    memcpy(MAC_GARA[numeroElementi], data, 6);
    numeroElementi++;
    Serial.println("NUOVO MAC RICEVUTO E SALVATO.\n");
    return true;  //salvataggio effttuato possono stampare
  }

  return false; //mac gia ricevuto

  
}

void onMesh(const EspMeshEvent &e) {    //evento di tipo espmesh

  switch (e.type)   //controlla il tipo di evento, se di tipo connected, disconnected, became root o message
  {
  case EspMeshEventType::Connected:
    digitalWrite(LED_RGB_R, HIGH);
    digitalWrite(LED_RGB_G, LOW);       //led verde acceso
    digitalWrite(LED_RGB_B, HIGH);

    break;
  case EspMeshEventType::Disconnected:
    digitalWrite(LED_RGB_R, HIGH);
    digitalWrite(LED_RGB_G, HIGH);    //led spento
    digitalWrite(LED_RGB_B, HIGH);

    break;
  case EspMeshEventType::BecameRoot:
    digitalWrite(LED_RGB_R, LOW);
    digitalWrite(LED_RGB_G, LOW);    //questi 3 per ottenere il giallo
    digitalWrite(LED_RGB_B, HIGH);



    break;
  case EspMeshEventType::Message:

    if(e.len == 6){
      uint8_t *macAddress = (uint8_t*)e.data;

      if(salvataggioMAC(e.data,e.len)){   //se effettivamente ho salvato un nuovo indirizzo mac lo stampo a schermo

        display->printf("MAC Ricevuto: ");
        display->display();
        
        for(int i = 0; i < e.len; i++){
          display->printf("%u:",macAddress[i]);
        }
        display->printf("\n");
        display->display();

      }

    }
    break;
  
  default:
    break;
  }


}


void setup(){

  IoTBoard::init_serial();
  IoTBoard::init_leds();
  IoTBoard::init_display();

  EspMeshConfig configMesh;             
  configMesh.router_ssid = SSID; //SSID dell'AP
  configMesh.router_pass = PASS; //Password per connettersi ad AP
  configMesh.on_event = onMesh;         

  IoTBoard::init_espmesh(configMesh);   
  display->clearDisplay();
  display->setCursor(0,0);
  display->display();

  espmesh->getMac(mac);
  Serial.printf("INDIRIZZO MAC: ");
  for(int i = 0; i < 6; i++){
    Serial.printf("%u:",mac[i]);
  }
  memcpy(MAC_GARA[0], mac, 6);    //assegno al primo slot dell'elenco dei mac address il mac del dispositivo stesso
  numeroElementi++;

  display->printf("MY MAC: ");
  for(int i = 0; i < 6; i++){
    display->printf("%u:", mac[i]);
  }
  display->println("\n");
  display->display();
}

void loop(){
  espmesh->update();          
  if(IoTBoard::runEvery(6000)){
    espmesh->broadcast((char*)mac, 6);     
  }

}


