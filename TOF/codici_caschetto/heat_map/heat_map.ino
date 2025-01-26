#include <Arduino.h>
#include <Adafruit_TinyUSB.h>  // for Serial
#include <Wire.h>
#include <SparkFun_VL53L5CX_Library.h>  //http://librarymanager/All#SparkFun_VL53L5CX

#define deviceAddress1 0x29
// #define deviceAddress2
// #define deviceAddress3
SparkFun_VL53L5CX TOF1;
VL53L5CX_ResultsData measurementData;  // Result data class structure, 1356 byes of RAM

int imageResolution;
int number = 0;
int image_frequency = 10;  //Hz
int time_ISR = 100;        //ms


//////////////////////////////////////////////GLOBAL VARIABLES/////////////////////////////////////////////////////
int STATE1 = 0;
// STATE2 = 0, STATE3 = 0;
// STATE 0:  controllo
// STATE 11: allerta 1 livello
// STATE 12: allerta 2 livello
// STATE 2:  allarme
int sampling1 = 0; //set the 2 modes of sampling
//sampling2 = 0, sampling3 = 0;  
// sampling 0 : chess
// sampling 1 : all the matrix

int threshold1 = 10;  //threshold allerta primo livello  [cm/s]
int threshold2 = 15;  //threshold allerta secondo livello[cm/s]
int alert1_1 = 0;     //flag per passare allo stato allerta 1
int alert2_1 = 0;     //flag per passare allo stato allerta 2
int alertbuz_1 = 0;   //flag per passare allo stato di allarme
int reset = 0;        //flag per tornare allo stato di controllo iniziale, se nulla succede
int conta_alertbuz =0;

// int alert1_2 = 0;
// int alert2_2 = 0;
// int alert1_3 = 0;
// int alert2_3 = 0;
int reset_state1 = 0;
const int matrixSize = 8;
int distanceMatrixpast1[matrixSize][matrixSize];  //values stored at t=-1
int distanceMatrix[matrixSize][matrixSize];       //values stored at t=0
int delta_v[matrixSize][matrixSize] = { 0 };      //initializing matrix at 0

SoftwareTimer blinkTimer;  //timer used for ISR

void setup() 
{
  Serial.begin(115200);
  delay(1000);
  Serial.println("Caschetto in fase di set up");

  Wire.begin();           //This resets to 100kHz I2C
  Wire.setClock(400000);  //Sensor has max I2C freq of 400kHz

  ////////////////////////////////////////////////////// SET TOF1 ///////////////////////////////////////////////////

  Serial.println("Initializing TOF1. This can take up to 10s. Please wait.");
  // Ho commentato questo controllo perchè non runnava il codice altrimenti, c'è da controllare questa funzione come va usata

  // if (TOF1.setAddress(deviceAddress1) == false) {
  //   Serial.println(F("TOF1 failed to set new address. Please try again. Freezing..."));
  //   while (1)
  //     ;
  // }

  if (TOF1.begin() == false) {
    Serial.println(F("TOF1 not found - check your wiring. Freezing"));
    while (1)
      ;
  }

  TOF1.setResolution(8 * 8);               //Enable all 64 pads
  imageResolution = TOF1.getResolution();  //Query sensor for current resolution - either 4x4 or 8x8
  //imageWidth = sqrt(imageResolution);    //Calculate printing width

  //Using 4x4, min frequency is 1Hz and max is 60Hz
  //Using 8x8, min frequency is 1Hz and max is 15Hz
  bool response = TOF1.setRangingFrequency(image_frequency);
  if (response == true) {
    int frequency = TOF1.getRangingFrequency();
    if (frequency > 0) {
      Serial.print("Ranging frequency set to ");
      Serial.print(frequency);
      Serial.println(" Hz.");
    } else
      Serial.println(F("Error recovering ranging frequency."));
  } else {
    Serial.println(F("Cannot set ranging frequency requested. Freezing..."));
    while (1)
      ;
  }

  Serial.println("Start TOF1 ranging");
  TOF1.startRanging();

  /////////////////////////////////////////////////////// SET TIMER  TOF/////////////////////////////////////////////////

  // Configure the timer with 100 ms interval and define the callback
  blinkTimer.begin(time_ISR, blink_timer_callback);
  // Start the timer
  blinkTimer.start();
  Serial.println("Timer started.");
  delay(2000); //Pause for 2 sec just to understand

  //MANDARE MESSAGGIO CON BLUETOOTH
}

void loop() {
  /////////////////////////////////////////STATE MACHINE for TOF 1//////////////////////////////////////////////////////
  //After the set up, the state is automatically set at 0 = CONTROLLO
  // switch (STATE1) {
  //   case 0:
  //     //CONTROLLO
  //     //Chess control
  //     sampling1 = 0;

  //     if (alert1_1 == 1) {
  //       STATE1 = 11;  //passo ad allerta di primo livello
  //       alert1_1 = 0;
  //     } 
  //     else if (alert2_1 == 1) {
  //       STATE1 = 12;  //passo ad allerta di secondo livello
  //       alert1_1 = 0;
  //     }

  //   break;

  //   case 11:
  //     //ALLERTA 1 LIVELLO
  //     sampling1 = 1;  //complete sampling1

  //     if (alertbuz_1 == 1) {    //è stata mantenuta la velocità mentre eravamo in allerta 1 livello
  //       STATE1 = 2;             //passo ad allarme buzzer
  //       alertbuz_1 = 0;         //riazzero la flag
  //     } 
  //     else if (reset == 1) {    //se invece dopo 3 secondi non è successo nulla reset
  //       STATE1 = 0;
  //     }
  //   break;

  //   case 12:
  //     //ALLERTA 2 LIVELLO
  //     sampling1 = 1;  //complete sampling1

  //     if (conta_alertbuz > 4) { //dopo il check di allerta di secondo livello, se superato in almeno 4 pixel 
  //       STATE1 = 2;  // andare subito nello stato di allarme buzzer
  //       conta_alertbuz = 0;
  //     } 
  //     else {
  //       STATE1 = 0;  // se la soglia non stata superata all'iterazione successiva, torno allo stato di controllo
  //     }

  //   break;

  //   case 2:
  //     //ALLARME
  //     //start buzzer
  //     Serial.println("BUZZER 1 !!!"); // qui manca da decidere per quanto farlo suonare e quindi quando tornare allo stato 0
  //     while(1);
  //   break;
  // }
}

void blink_timer_callback(TimerHandle_t xTimerID) {
  (void)xTimerID;

  if (TOF1.isDataReady() == true) {
    if (TOF1.getRangingData(&measurementData)) {

      sampling1_total();
      // switch(sampling1) 
      // {
      //   case 0:
      //     //get the data as a chess board
      //     sampling1_chess();
      //     break;

      //   case 1:
      //     //get all the data
      //     sampling1_total();
      //     break;
      // }
    }
  }
}

// void sampling1_chess() {
//   int conta_alert1 = 0;
//   int conta_alert2 = 0;

//   //Save past matrix
//   memcpy(distanceMatrixpast1, distanceMatrix, sizeof(distanceMatrix));

//   //Save new matrix
//   for (int y = 0; y < matrixSize; y = y + 2)  //rows
//   {
//     for (int x = 0; x < matrixSize; x = x + 2)  //column
//     {
//       int index = x + (y * matrixSize / 2);  //[0;63]
//       distanceMatrix[y / 2][x / 2] = measurementData.distance_mm[index];
//       if(distanceMatrix[y / 2][x / 2] <10)
//       {
//         conta_alert1++;
//       }
//       if(distanceMatrix[y / 2][x / 2] <5)
//       {
//         conta_alert2++;
//       }
//       // delta_v[y / 2][x / 2] = (distanceMatrixpast1[y / 2][x / 2] - distanceMatrix[y / 2][x / 2]) / time_ISR;  // [cm]/[s]

//       // //CHECK DUE SOGLIE
//       // if (delta_v[y / 2][x / 2] > threshold1) {
//       //   conta_alert1++;
//       // }

//       // if (delta_v[y / 2][x / 2] > threshold2) {
//       //   conta_alert2++;
//       // }

//       // Serial.print(delta_v[y][x]);
//       Serial.print(distanceMatrix[y][x]);
//       Serial.print(","); //to use processing
//       //Serial.print("\t");
//     }
//     Serial.println();
//   }
//   Serial.println();

//   //Acquisizione matrice completata, a seconda di quale soglia passiamo allo stato di allerta corrispondente
//   if (conta_alert1 > 2) {
//     alert1_1 = 1;  // --> passare ad allerta di primo livello
//   }

//   if (conta_alert2 > 2) {
//     alert2_1 = 1;  // --> passare ad allerta di secondo livello
//   }
// }


void sampling1_total() {

  int minmatrix = 200;
  int minspeed = 200;
  //int delta_v[matrixSize][matrixSize] = { 0 };
  //tenuto l'azzeramento nella matrice soltanto quando cambia da sampling chess a sampling total, stampava altrimenti tutti zeri
  // Probabilmente perchè è troppo veloce il codice dell'interrupt.

  //Save past matrix
  memcpy(distanceMatrixpast1, distanceMatrix, sizeof(distanceMatrix));

  //get, save and print the data
  for (int y = 0; y < matrixSize; y++)  //rows
  {
    for (int x = 0; x < matrixSize; x++)  //column
    {
      int index = x + (y * matrixSize);
      distanceMatrix[y][x] = measurementData.distance_mm[index];
      delta_v[y][x] = (distanceMatrixpast1[y][x] - distanceMatrix[y][x]) / time_ISR;  // [cm]/[s]

      // if (STATE1 == 12) 
      // { //se siamo in stato allerta 2 livello
      //   if (delta_v[y][x] > threshold2)  //threshold in cm/s
      //   {
      //     conta_alertbuz++;  //flag che conta quanti pixel superano la soglia 2
      //   }
      // } else if (STATE1 == 11) 
      // { //se invece siamo in allerta 1 livello
      //   if (distanceMatrixpast1[y][x] - distanceMatrix[y][x] < minmatrix) 
      //   { //calcolo dei minimi
      //     minmatrix = distanceMatrix[y][x] / 10;  //[cm]
      //     minspeed = delta_v[y][x];               //[cm/s]
      //   }
      // }

      //Serial.print(delta_v[y][x]);
      //Serial.print(conta_alertbuz);
      Serial.print(distanceMatrix[y][x]);
      Serial.print(","); //to use processing
      //Serial.print("\t");
    }
  }
  Serial.println();

  //Ad acquisizione completata se siamo in allerta 1 livello check velocità: copre la min distanza tra veicolo e ciclista?
  // if (STATE1 == 11) {
  //   if (minspeed > minmatrix) {
  //     alertbuz_1 = 1;
  //   } else {
  //     reset_state1++; 
  //   }

  //   if (reset_state1 == 30) {
  //     reset = 1;
  //   }
  // }
}