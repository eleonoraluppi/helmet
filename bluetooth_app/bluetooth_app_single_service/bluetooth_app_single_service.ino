#include <bluefruit.h> // Include la libreria Bluefruit per il modulo nRF52

// Definisci i UUID per iL serviziO e le caratteristiche
#define CASCHETTO_SERVICE_UUID    0x180F  // Servizio di prossimità
#define PROXIMITY_CHAR_UUID       0x2A56  // Caratteristica di prossimità
#define FALL_CHAR_UUID            0x2A58  // Caratteristica di caduta
#define DISTRACTION_CHAR_UUID            0x2A54  // Caratteristica di Distrazione


// Crea il servizio GATT per il rilevamento della macchina e della caduta
BLEService proximityService(CASCHETTO_SERVICE_UUID);
//BLEService fallService(FALL_SERVICE_UUID);

// Crea le caratteristiche per la prossimità (alert macchina) e la caduta
BLECharacteristic proximityCharacteristic(PROXIMITY_CHAR_UUID, BLERead | BLEIndicate, 20); // max 20 byte
BLECharacteristic fallCharacteristic(FALL_CHAR_UUID, BLERead | BLEIndicate, 20); // max 20 byte
BLECharacteristic distractionCharacteristic(DISTRACTION_CHAR_UUID, BLERead | BLEIndicate, 20); // max 20 byte

unsigned long lastProximityAlertTime = 0;

unsigned int connession=0;

void setup() {
  Serial.begin(115200);
  while ( !Serial ) delay(10);   // for nrf52840 with native usb

// Inizializza il Bluetooth
  Serial.println("Initialise the Bluefruit nRF52 module");
  Bluefruit.begin();          
  Bluefruit.setTxPower(4);    // Imposta la potenza di trasmissione
  Bluefruit.setName("Caschetto1"); // Imposta il nome del dispositivo Bluetooth

  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);

  proximityService.begin();
  proximityCharacteristic.setProperties(CHR_PROPS_INDICATE);
  proximityCharacteristic.begin();

  //fallService.begin();
  fallCharacteristic.setProperties(CHR_PROPS_INDICATE);
  fallCharacteristic.begin();

  distractionCharacteristic.setProperties(CHR_PROPS_INDICATE);
  distractionCharacteristic.begin();

  // Inizializza il dispositivo BLE come server
  Bluefruit.Advertising.addService(proximityService);
  Bluefruit.Advertising.addName();  // aggiungi il nome del dispositivo  
  //Bluefruit.Advertising.restartOnDisconnect(true);  // Continua a fare advertising dopo la disconnessione
  //Bluefruit.Advertising.setInterval(32, 244);       // Intervalli di advertising
  //Bluefruit.Advertising.setFastTimeout(30);         // Tempo massimo in modalità veloce
  //Bluefruit.Advertising.start(0);
  Bluefruit.Advertising.start();  // Avvia l'advertising

  

  Serial.println("Inizializzazione completata. Attendo connessione...");
}

void loop() {
  unsigned long currentMillis = millis();
  if(connession){
    
  // Simula un alert macchina ogni 3 secondi
  if (currentMillis - lastProximityAlertTime >= 9000) {
      sendFallAlert("Caduta rilevata");
      delay(2000);

      sendProximityAlert("Macchina in avvicinamento");
      delay(3000);
      sendDistractionAlert("Sei distratto");

    lastProximityAlertTime = currentMillis;
  }
  
  }

}

// Funzione per inviare un alert macchina
void sendProximityAlert(const char* message) {
  Serial.println("Alert macchina: " + String(message));
  //proximityCharacteristic.write(message);  // Imposta il valore della caratteristica
  proximityCharacteristic.indicate(message);           // Invia la notifica
}

// Funzione per inviare un alert caduta
void sendFallAlert(const char* message) {
  Serial.println("Alert caduta: " + String(message));
  //fallCharacteristic.write(message);  // Imposta il valore della caratteristica
  fallCharacteristic.indicate(message);           // Invia la notifica
}
// Funzione per inviare un alert distrazione
void sendDistractionAlert(const char* message) {
  Serial.println("Alert distrazione: " + String(message));
  //fallCharacteristic.write(message);  // Imposta il valore della caratteristica
  distractionCharacteristic.indicate(message);           // Invia la notifica
}

//todo: sto cercando di confrontare per capire se questo centra... o è il centrale???
void connect_callback(uint16_t conn_handle)
{
  // Get the reference to current connection
  BLEConnection* connection = Bluefruit.Connection(conn_handle);

  char central_name[32] = { 0 };
  connection->getPeerName(central_name, sizeof(central_name));

  Serial.print("Connected to ");
  Serial.println(central_name);
  connession=1;
}

void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void) conn_handle;
  (void) reason;

  Serial.print("Disconnected, reason = 0x"); Serial.println(reason, HEX);
  Serial.println("Advertising!");
  connession=0;
}

