/*
  Read an 8x8 array of distances from the VL53L5CX
  By: Nathan Seidle
  SparkFun Electronics
  Date: October 26, 2021
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  This example shows how to read all 64 distance readings at once.

  Feel like supporting our work? Buy a board from SparkFun!
  https://www.sparkfun.com/products/18642

*/

#include <Arduino.h>
#include <Adafruit_TinyUSB.h>  // for Serial

#include <Wire.h>

#include <SparkFun_VL53L5CX_Library.h>  //http://librarymanager/All#SparkFun_VL53L5CX

SparkFun_VL53L5CX myImager;
VL53L5CX_ResultsData measurementData;  // Result data class structure, 1356 byes of RAM

int imageResolution = 0;  //Used to pretty print output
int imageWidth = 0;       //Used to pretty print output
int number=0;

SoftwareTimer blinkTimer;

const int matrixSize = 8;
int distanceMatrix[matrixSize][matrixSize];


void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("SparkFun VL53L5CX Imager Example");

  Wire.begin();           //This resets to 100kHz I2C
  Wire.setClock(400000);  //Sensor has max I2C freq of 400kHz

  Serial.println("Initializing sensor board. This can take up to 10s. Please wait.");
  if (myImager.begin() == false) {
    Serial.println(F("Sensor not found - check your wiring. Freezing"));
    while (1)
      ;
  }

  myImager.setResolution(8 * 8);  //Enable all 64 pads

  imageResolution = myImager.getResolution();  //Query sensor for current resolution - either 4x4 or 8x8
  imageWidth = sqrt(imageResolution);          //Calculate printing width

  //Using 4x4, min frequency is 1Hz and max is 60Hz
  //Using 8x8, min frequency is 1Hz and max is 15Hz
  bool response = myImager.setRangingFrequency(15);
  if (response == true)
  {
    int frequency = myImager.getRangingFrequency();
    if (frequency > 0)
    {
      Serial.print("Ranging frequency set to ");
      Serial.print(frequency);
      Serial.println(" Hz.");
    }
    else
      Serial.println(F("Error recovering ranging frequency."));
  }
  else
  {
    Serial.println(F("Cannot set ranging frequency requested. Freezing..."));
   while(1);
  }

  myImager.startRanging();


  // Configure the timer with 1000 ms interval, with our callback

  blinkTimer.begin(100, blink_timer_callback);

  // Start the timer
  blinkTimer.start();
}

void loop() {
}
/*
void blink_timer_callback(TimerHandle_t xTimerID)
{
  number=number+1;
  // freeRTOS timer ID, ignored if not used
  (void) xTimerID;
  if (myImager.isDataReady() == true)
  {
    if (myImager.getRangingData(&measurementData)) //Read distance data into array
    {
      //The ST library returns the data transposed from zone mapping shown in datasheet
      //Pretty-print data with increasing y, decreasing x to reflect reality
      for (int y = 0 ; y <= imageWidth * (imageWidth - 1) ; y += imageWidth)
      {
        for (int x = imageWidth - 1 ; x >= 0 ; x--)
        {
          Serial.print("\t");
          Serial.print(measurementData.distance_mm[x + y]);
        }
        Serial.println();
      }
      Serial.println(number);
    }
  }
  
}
*/

//void blink_timer_callback(TimerHandle_t xTimerID) {
  //(void)xTimerID;
 // number=number+1;


//    if (myImager.isDataReady() == true)
//{
//
  //if (myImager.getRangingData(&measurementData)) {
    //for (int y = 0; y < matrixSize; y++) {
      //for (int x = 0; x < matrixSize; x++) {
        //int index = x + (y * matrixSize);
        //distanceMatrix[y][x] = measurementData.distance_mm[index];
      //}
    //}
//
  //  Serial.println("Matrice di distanza:");
    //for (int y = 0; y < matrixSize; y++) {
      //for (int x = 0; x < matrixSize; x++) {
        //Serial.print(distanceMatrix[y][x]);
        //Serial.print("\t");
      //}
      //Serial.println();
    //}
    //Serial.println();
    //Serial.println(number);
  //}
//}
//}

void blink_timer_callback(TimerHandle_t xTimerID) {
  (void)xTimerID;
  number = number + 1;

  if (myImager.isDataReady() == true) {
    if (myImager.getRangingData(&measurementData)) {
      for (int y = 0; y < matrixSize; y++) {
        for (int x = 0; x < matrixSize; x++) {
          int index = x + (y * matrixSize);
          distanceMatrix[y][x] = measurementData.distance_mm[index];
        }
      }

      Serial.println("Matrice di distanza:");
      for (int y = 0; y < matrixSize; y++) {
        for (int x = matrixSize - 1; x >= 0; x--) {
          Serial.print(distanceMatrix[matrixSize - 1 - x][y]);
          Serial.print("\t");
        }
        Serial.println();
      }
      Serial.println();
      Serial.println("Numero di misurazioni: ");
      Serial.println(number);
    }
  }
}

//questo codice mi salva i dati in una matrice da sinistra a destra e dall'alto in basso. però mi stampa in questo modo:
//3       2       1       0
//7       6       5       4
//11      10      9       8
//15      14      13      12















