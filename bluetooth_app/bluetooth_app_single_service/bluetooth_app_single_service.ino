#include <bluefruit.h> // Include la libreria Bluefruit per il modulo nRF52

// Definisci i UUID per iL serviziO e le caratteristiche
#define CASCHETTO_SERVICE_UUID    0x180F  // Servizio di prossimità
#define PROXIMITY_CHAR_UUID       0x2A56  // Caratteristica di prossimità
#define FALL_CHAR_UUID            0x2A58  // Caratteristica di caduta
#define DISTRACTION_CHAR_UUID            0x2A54  // Caratteristica di Distrazione
#define NORM_DATA_CHAR_UUID      0x2A5B  // Caratteristica per ricezione N_norm, E_norm, D_norm
#define REQUEST_NORM_DATA        0x2A5C



// Crea il servizio GATT per il rilevamento della macchina e della caduta
BLEService proximityService(CASCHETTO_SERVICE_UUID);

// Crea le caratteristiche per la prossimità (alert macchina) e la caduta
BLECharacteristic proximityCharacteristic(PROXIMITY_CHAR_UUID, BLERead | BLEIndicate, 20); // max 20 byte
BLECharacteristic fallCharacteristic(FALL_CHAR_UUID, BLERead | BLEIndicate, 20); // max 20 byte
BLECharacteristic distractionCharacteristic(DISTRACTION_CHAR_UUID, BLERead | BLEIndicate, 20); // max 20 byte
BLECharacteristic requestNormCharacteristic(REQUEST_NORM_DATA, BLERead | BLEIndicate, 20); // max 20 byte
BLECharacteristic normDataCharacteristic(NORM_DATA_CHAR_UUID, BLEWrite | BLEWriteWithoutResponse,20); // Max 20 byte


unsigned long lastProximityAlertTime = 0;
unsigned long currentMillis =0;

unsigned int connession=0;
unsigned int firstTime=1;

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

  requestNormCharacteristic.setProperties(CHR_PROPS_INDICATE);
  requestNormCharacteristic.begin();

  normDataCharacteristic.begin();
  normDataCharacteristic.setWriteCallback(normDataWriteCallback);


  // Inizializza il dispositivo BLE come server
  Bluefruit.Advertising.addService(proximityService);
  Bluefruit.Advertising.addName();  // aggiungi il nome del dispositivo  
  //Bluefruit.Advertising.restartOnDisconnect(true);  // Continua a fare advertising dopo la disconnessione
  //Bluefruit.Advertising.setInterval(32, 244);       // Intervalli di advertising
  //Bluefruit.Advertising.setFastTimeout(30);         // Tempo massimo in modalità veloce
  //Bluefruit.Advertising.start(0);
  Bluefruit.Advertising.start();  // Avvia l'advertising

  

  Serial.println("Inizializzazione completata. Attendo connessione...");
  currentMillis = millis();

}

void loop() {
 // unsigned long currentMillis = millis();
 
  if(connession){
    unsigned long currentMillis2 = millis();

    if (currentMillis2 - currentMillis >= 9000 & firstTime) {
      firstTime=0;
      Serial.println("mando un messaggio");
      requestNormCharacteristic.indicate("1");
    }           

    
  // Simula un alert macchina ogni 3 secondi
  //if (currentMillis - lastProximityAlertTime >= 9000) {
    //  sendFallAlert("Caduta rilevata");
    //  delay(2000);

    //  sendProximityAlert("Macchina in avvicinamento");
    //  delay(3000);
    //  sendDistractionAlert("Sei distratto");

    //lastProximityAlertTime = currentMillis;
  //}
  
  }
  else{
      currentMillis = millis();

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

void normDataWriteCallback(uint16_t conn_handle, BLECharacteristic* chr, uint8_t* data, uint16_t len) {
  Serial.print("Dati ricevuti: ");
  
  String receivedData;
  for (int i = 0; i < len; i++) {
    receivedData += (char)data[i];  // Costruisce una stringa dai byte ricevuti
  }
  Serial.println(receivedData);

  Serial.print("Dati ricevuti RAW: [");
Serial.print(receivedData);
Serial.println("]");

  // Parsing dei valori N_norm, E_norm, D_norm
double nNorm, eNorm, dNorm;
  //sscanf(receivedData.c_str(), "%lf,%lf,%lf", &nNorm, &eNorm, &dNorm); //(non funzionava bene!)
  
char buffer[receivedData.length() + 1]; // Creiamo un array di caratteri
strcpy(buffer, receivedData.c_str());   // Copiamo il contenuto della stringa

char *ptr = buffer; // Ora abbiamo un puntatore a char modificabile

// Parsing manuale con gestione dei separatori
nNorm = strtod(ptr, &ptr);
if (*ptr == ',') ptr++;  // Salta la virgola

eNorm = strtod(ptr, &ptr);
if (*ptr == ',') ptr++;  // Salta la virgola

dNorm = strtod(ptr, NULL);

  Serial.print("N_norm: "); Serial.println(nNorm,17);
  Serial.print("E_norm: "); Serial.println(eNorm,17);
  Serial.print("D_norm: "); Serial.println(dNorm,17);
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
  firstTime=1;
}

void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void) conn_handle;
  (void) reason;

  Serial.print("Disconnected, reason = 0x"); Serial.println(reason, HEX);
  Serial.println("Advertising!");
  connession=0;
}

