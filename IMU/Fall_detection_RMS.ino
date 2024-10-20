/****************************************************************
 FALL DETECTION
 9-axis IMU data
 ***************************************************************/

#include "ICM_20948.h" // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU
#define SERIAL_PORT Serial
#define WIRE_PORT Wire // Your desired Wire port.      Used when "USE_SPI" is not defined
#define AD0_VAL 1 // The value of the last bit of the I2C address.
// On the SparkFun 9DoF IMU breakout the default is 1, and when the ADR jumper is closed the value becomes 0

ICM_20948_I2C myICM;      // my sampling rate is 723 Hz
float accX, accY, accZ;   
float deltaX = 0, deltaY = 0, deltaZ = 0;  // delta acc (new_acc - old_acc)
float MS_x = 0, MS_y = 0, MS_z = 0;
float previous_accX = 0.0;
float previous_accY = 0.0;
float previous_accZ = 0.0;
int sensitivity = 500;   // threshold for impact detection [mg]
byte impatto_avvenuto;    // flag

const int windowSize = 100;  // dimension of the moving window (impact last 50-200 ms => sampling freq=723Hz => 36-144 samples => 100)
int index_window = 0;  // Index for the arrray
float xWindow[windowSize]={0};  // initialization
float yWindow[windowSize]={0};
float zWindow[windowSize]={0};

// different values for 'stato'
# define NORMAL 0
# define IMPACT_ACC 1
# define IMPACT_ANG 2
# define IMPACT_NO_MOVEMENT 3
int stato = NORMAL;   

// FUNCTIONS DECLARATION
void read_accXYZ(ICM_20948_I2C *sensor);  




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
    if (myICM.dataReady())  // if there is a new data
    {
      myICM.getAGMT(); 
      read_accXYZ(&myICM);  // read new acc data

      // Compute delta_acc
      deltaX = accX - previous_accX;
      deltaY = accY - previous_accY;
      deltaZ = accZ - previous_accZ;

      // put the delta_acc in the moving window
      xWindow[index_window] = deltaX;
      yWindow[index_window] = deltaY;
      zWindow[index_window] = deltaZ;

      // increase the index
      index_window = (index_window + 1) % windowSize; // if index = 9 => (9+1)%10=0 => I restart from the beginning

      // MS
      for (int i = 0; i < windowSize; i++) 
      {
          MS_x += sq(xWindow[i]);
          MS_y += sq(yWindow[i]);
          MS_z += sq(zWindow[i]);
      }
      MS_x /= windowSize;   // divide for the number of samples 
       MS_y /= windowSize;
      MS_z /= windowSize;

      // RMS
      float RMS_x = sqrt(MS_x);
      float RMS_y = sqrt(MS_y);
      float RMS_z = sqrt(MS_z);
      float RMS = sqrt((sq(RMS_x) + sq(RMS_y) + sq(RMS_z)) / 3);
    
      // update old_values
      previous_accX = accX;
      previous_accY = accY;
      previous_accZ = accZ;

      if (RMS>sensitivity)  //check the impact
      {
       stato=IMPACT_ACC;
       Serial.print("impact detected:: ");
       Serial.print("RMS: ");
       Serial.println(RMS);
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
  accX= sensor->accX();  
  accY= sensor->accY();
  accZ= sensor->accZ();

}


