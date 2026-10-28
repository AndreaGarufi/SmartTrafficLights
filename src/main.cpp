//NOTA! 
//con la funzione broadcast() ogni scheda manda il messaggio anche a se stessa quindi se lo
//avvii con una singola scheda vedrai comunque il suo indirizzo mac stampato sul display
//aggiornamento: l'ho segnalato con la scritta: "MY MAC"


//PROMEMORIA
//continua con il salvataggio nell'array di tutti i mac che arrivano + quello della scheda

#include <iot_board.h>
//#include <WiFi.h>
#include <sys/socket.h>

const char *SSID = " Galaxy S21 5G";  //SSID proprio AP
const char *PASS = "campobasso12345"; //password proprio AP

uint8_t mac[6] = {0};
bool macConsegnato = false;

void onMesh(const EspMeshEvent &e) {    //evento di tipo espmesh

  switch (e.type)   //controlla il tipo di evento, se di tipo connected, disconnected, became root o message
  {
  case EspMeshEventType::Connected:
    display->clearDisplay();
    display->setCursor(0,0);
    display->print("Connesso: NON ROOT\n");
    display->display();
    break;
  case EspMeshEventType::Disconnected:
    display->clearDisplay();
    display->setCursor(0,0);
    display->print("NON connesso\n");
    display->display();
    break;
  case EspMeshEventType::BecameRoot:
    display->clearDisplay();
    display->setCursor(0,0);
    display->print("Connesso: ROOT\n");
    display->display();
    break;
  case EspMeshEventType::Message:

    if(espmesh->isConnected()){
      if(espmesh->isRoot()){
        display->clearDisplay();
        display->setCursor(0,0);
        display->print("Connesso: ROOT\n");
        display->display();        
      }else{
        display->clearDisplay();
        display->setCursor(0,0);
        display->print("Connesso: NON ROOT\n");
        display->display();    
      }
    }

    if(e.len == 6){

      uint8_t *macAddress = (uint8_t*)e.data;
      if(memcmp(macAddress, mac, 6) == 0){
        display->printf("MY MAC: ");
      }else{
        display->printf("MAC Ricevuto: ");
      }
      display->display();
      
      for(int i = 0; i < e.len; i++){
        display->printf("%u:",macAddress[i]);
      }
      display->println("\n");
      display->display();
    }
  
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

}

void loop(){
  espmesh->update();          
  if(IoTBoard::runEvery(6000)){
    espmesh->broadcast((char*)mac, 6);     
  }

}


