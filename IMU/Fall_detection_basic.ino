/****************************************************************
 FALL DETECTION
 9-axis IMU data
 ***************************************************************/

#include "ICM_20948.h" // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU
#define SERIAL_PORT Serial
#define WIRE_PORT Wire // Your desired Wire port.      Used when "USE_SPI" is not defined
#define AD0_VAL 1 // The value of the last bit of the I2C address.
// On the SparkFun 9DoF IMU breakout the default is 1, and when the ADR jumper is closed the value becomes 0

ICM_20948_I2C myICM; 
float xaxis = 0, yaxis = 0, zaxis = 0;  // current acc 
float deltx = 0, delty = 0, deltz = 0;  // delta acc (new_acc - old_acc)
float magnitude = 0;        // MSE of accellerations (if higher than a threshold, an impact has occurred)
int sensitivity = 2000;   // threshold for impact detection [mg] (20g=20000mg)
byte impatto_avvenuto;    // flag

// different values for stato
# define NORMAL 0
# define IMPACT_ACC 1
# define IMPACT_ANG 2
# define IMPACT_NO_MOVEMENT 3
int stato = NORMAL;   

// FUNCTIONS DECLARATION
void read_accXYZ(ICM_20948_I2C *sensor);  // Dichiarazione della funzione



void setup()
{
  SERIAL_PORT.begin(115200);
  while (!SERIAL_PORT)
  {};

  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000);
  //myICM.enableDebugging(); // Uncomment this line to enable helpful debug messages on Serial

  bool initialized = false;
  while (!initialized)
  {
    myICM.begin(WIRE_PORT, AD0_VAL);
    SERIAL_PORT.print(F("Initialization of the sensor returned: "));
    SERIAL_PORT.println(myICM.statusString());
    if (myICM.status != ICM_20948_Stat_Ok)
    {
      SERIAL_PORT.println("Trying again...");
      delay(500);
    }
    else
    {
      initialized = true;
    }
  }
}



void loop()
{
  if (stato == NORMAL)  // search if acceleration values are higher than a threshold
  {
    if (myICM.dataReady())
    {
      myICM.getAGMT();         // The values are only updated when you call 'getAGMT'
    
      // save old values of acceleration
      int oldx = xaxis;       
      int oldy = yaxis;
      int oldz = zaxis;

      // read new values of acceleration
      read_accXYZ(&myICM);  
      
      // calculate the new delta for any acceleration
      deltx = xaxis - oldx;
      delty = yaxis - oldy;
      deltz = zaxis - oldz;

        // calculate the magnitude of the accelerations to see if an impact has occurred.
      magnitude = sqrt(sq(deltx) + sq(delty) + sq(deltz));
      if (magnitude >= sensitivity)   //impact detected
      {
        stato = IMPACT_ACC;
        Serial.println("Impact detected!!");
        Serial.print("Magnitude:");
        Serial.println(magnitude);
      }
    }
  }

  if (stato == IMPACT_ACC)  // control if the angular acceleration is higher than a threshold
  {
      stato = NORMAL;
  }
  
}

void read_accXYZ(ICM_20948_I2C *sensor)
{
  // function that reads the value of accelleration in the 3 directions [mg]
  xaxis= sensor->accX();  
  yaxis= sensor->accY();
  zaxis= sensor->accZ();
  /*
  // to print the values in the serial monitor
  Serial.print("Xaxis: ");
  Serial.print(xaxis);
  Serial.print("  Yaxis: ");
  Serial.print(yaxis);
  Serial.print("  Zaxis: ");
  Serial.println(zaxis);
  */
}


