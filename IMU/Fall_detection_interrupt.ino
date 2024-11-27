/****************************************************************
 FALL DETECTION
 9-axis IMU data
 ***************************************************************/

#include "ICM_20948.h" // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU
#include "NRF52TimerInterrupt.h"
#define SERIAL_PORT Serial
#define WIRE_PORT Wire // Your desired Wire port.      Used when "USE_SPI" is not defined
#define AD0_VAL 1 // The value of the last bit of the I2C address.
// On the SparkFun 9DoF IMU breakout the default is 1, and when the ADR jumper is closed the value becomes 0

SoftwareTimer IMU_timer;

ICM_20948_I2C myICM;      // my sampling rate is 723 Hz
float ax, ay, az;   
float gx, gy, gz;
float deltaX = 0, deltaY = 0, deltaZ = 0;  // delta acc (new_acc - old_acc)
float MS_x = 0, MS_y = 0, MS_z = 0;        // to perform the RMS_delta_acc
float MS_x_2 = 0, MS_y_2 = 0, MS_z_2 = 0;  // to perform the RMS_acc
float previous_accX = 0.0;
float previous_accY = 0.0;
float previous_accZ = 0.0;
int thr_acc = 4.9;   // threshold for impact detection [m/s^2]
int thr_ang = 250;   // threshold for angular inclination [°/s]
float Acc_tot=0;

const int windowSize = 100;  // dimension of the moving window (impact last 50-200 ms => sampling freq=723Hz => 36-144 samples => 100)
int index_window = 0;  // Index for the arrray
float xWindow[windowSize]={0};  // initialization window for RMS_delta_acc
float yWindow[windowSize]={0};
float zWindow[windowSize]={0};
float xWindow2[windowSize]={0};  // initialization window for RMS_acc
float yWindow2[windowSize]={0};
float zWindow2[windowSize]={0};

// variables for calibration
float accel_bias[3] = {-0.06538175643396205, -0.10585146494869235, -0.3593090659477376};
float accel_scale[3] = {0.10185438493480474 * 9.81, 0.10179187688049583 * 9.81, 0.10068963445555981 * 9.81};
float gyro_bias[3] = {0.595166, 1.332381, -0.340505}; // DPS
float gyro_scale[3] = {1, 1, 1};

// different values for 'stato_IMU'
# define NORMAL 0
# define IMPACT_HIGH_ACC 1
# define IMPACT_HIGH_ANG 2
# define IMPACT_NO_MOVEMENT 3
int stato_IMU = NORMAL;   
int sec=0;  //seconds passed from the impact 
int control_final_state=0;






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
      IMU_timer.begin(1000, IMU_timer_callback);   //start the timer (updated every minute)
      initialized = true;
    }
  }
}

void IMU_timer_callback(TimerHandle_t xTimerID)
{
  (void) xTimerID;
  sec=sec+1;
  Serial.println(ax);
  Serial.println(ay);
  Serial.println(az);
  Serial.println("1 second");
  if (sec==5)
  {
    Serial.println("5 second passed!!");
    control_final_state = 1;
  }
}


void loop()
{
  if (myICM.dataReady())  // if there is a new data
  {
    myICM.getAGMT(); 
    read_accXYZ(&myICM);  // read new acc data
    read_gyrXYZ(&myICM);
  }

  //  depending on the "stato_IMU" of the IMU, we make different checks
  if (stato_IMU == NORMAL)
  {
    //Acc_tot=(sqrt(ax*ax + ay*ay + az*az));
    //Serial.println(Acc_tot);
    find_impact();  // find high acc
  }
  else if (stato_IMU == IMPACT_HIGH_ACC) //If an impact has occurred, check the angle 
  {
    find_angle(); // find if high °
  }
  else if (stato_IMU == IMPACT_HIGH_ANG)
  {
    //stato_IMU = NORMAL;
    control_state(); 
  }
  else if(stato_IMU == IMPACT_NO_MOVEMENT)
  {
    // send the alarm 
    Serial.println("ALARM!!!!!!!");
  }
}


void find_impact()  // function that search for high acceleration. it identifies an impact if RMS_delta > thr
{                   // !! at the moment i'm performing both RMS and RMS_delta to compare them, then I will choose only one (I think RMS_Delta)

      // Compute delta_acc
      deltaX = ax - previous_accX;
      deltaY = ay - previous_accY;
      deltaZ = az - previous_accZ;

      // put the delta_acc in the moving window
      xWindow[index_window] = deltaX;
      yWindow[index_window] = deltaY;
      zWindow[index_window] = deltaZ;
      xWindow2[index_window] = ax;
      yWindow2[index_window] = ay;
      zWindow2[index_window] = az;

      // increase the index
      index_window = (index_window + 1) % windowSize; // if index = 9 => (9+1)%10=0 => I restart from the beginning

      // MS
      for (int i = 0; i < windowSize; i++) 
      {
          MS_x += sq(xWindow[i]);
          MS_y += sq(yWindow[i]);
          MS_z += sq(zWindow[i]);
          MS_x_2+= sq(xWindow2[i]);
          MS_y_2+= sq(yWindow2[i]);
          MS_z_2+= sq(zWindow2[i]);
      }
      MS_x /= windowSize;   // divide for the number of samples 
      MS_y /= windowSize;
      MS_z /= windowSize;
      MS_x_2 /= windowSize; 
      MS_y_2 /= windowSize; 
      MS_z_2 /= windowSize; 

      // RMS
      float RMS_x = sqrt(MS_x);
      float RMS_y = sqrt(MS_y);
      float RMS_z = sqrt(MS_z);
      float RMS_delta = sqrt((sq(RMS_x) + sq(RMS_y) + sq(RMS_z)) / 3);
      float RMS_x_2 = sqrt(MS_x_2);
      float RMS_y_2 = sqrt(MS_y_2);
      float RMS_z_2 = sqrt(MS_z_2);
      float RMS = sqrt((sq(RMS_x_2) + sq(RMS_y_2) + sq(RMS_z_2)) / 3);
    
      // update old_values
      previous_accX = ax;
      previous_accY = ay;
      previous_accZ = az;

      if (RMS_delta>thr_acc)  //check the impact
      {
       stato_IMU=IMPACT_HIGH_ACC;
       IMU_timer.start();
      
       Serial.println("impact detected:: ");
       Serial.print("RMS_delta :");
       Serial.println(RMS_delta);
       Serial.print(" RMS: ");
       Serial.println(RMS);

      }
}

void find_angle()  //function that search for high angles. it identifies an impact if theta of pitch or roll > thr
{

      if (abs(gx) >thr_ang || abs(gy)>thr_ang)// || abs(gyrZ)>thr_ang)     // da modificare con l'angolo se possibile
      {
        stato_IMU = IMPACT_HIGH_ANG;
        Serial.print("High Angolar velocity!!");
        Serial.print(" X :");
        Serial.print(gx);
        Serial.print(" Y :");
        Serial.print(gy);
        //Serial.print(" Z :");
        //Serial.println(gyrZ);
      }
      if (stato_IMU != IMPACT_HIGH_ANG)  //if theta < thr it was a false alert (FP)
      {
        Serial.print("False alarm");
        stato_IMU = NORMAL;   //come back to the normal state
        IMU_timer.stop();     // stoppo il timer, se no continua a contare
        sec=0;                // riporto il contatore dei secondi a 0
      }
    
}


void control_state ()   // aspetto 5 secondi dopo l'impatto e controllo se c'è solo acc(z)=9.8 m/s^2 => RMS_delta ~ 0 (di bias è circa sempre 8-9)                  
{                      // se è cosi per 5-10 secondi => !!!
  if (control_final_state==1)   // 5 seconds have passed from the impact
    {
      Acc_tot=(sqrt(ax*ax + ay*ay + az*az));
      if (Acc_tot<8.5 || Acc_tot >11)      // if acc(x), Acc(y) ≠ 0 and acc (Z) ≠ 9.8m/s^2 (0.5 di mrgine)
      {
        Serial.print("movement detected");
        Serial.println(Acc_tot);
        Serial.println(ax);
        Serial.println(ay);
        Serial.println(az);
        stato_IMU = NORMAL; // the person is not immobilized
        IMU_timer.stop();     // stoppo il timer, se no continua a contare
        sec=0;                // riporto il contatore dei secondi a 0
        control_final_state = 0;       
      }
      else if (sec >= 15)   // if after 15 seconds from the impact there is only acc(z) =g => immobilized
      {
        Serial.print("no movement");
        stato_IMU = IMPACT_NO_MOVEMENT; 
      }
    }
}



void read_accXYZ(ICM_20948_I2C *sensor)
{
  // function that reads the value of accelleration in the 3 directions [mg]
  // accX= sensor->accX();  
  //accY= sensor->accY();
  //accZ= sensor->accZ();
  ax = myICM.accX()/ 1000 * 9.81;
  ay = myICM.accY()/ 1000 * 9.81;
  az = myICM.accZ()/ 1000 * 9.81;
  calibrate_accel_data(&ax, &ay, &az);
}

void read_gyrXYZ(ICM_20948_I2C *sensor)
{
  //function that reads the angular velocity in the 3 directions [DPS]
  //gyrX= sensor->gyrX();
  //gyrY= sensor->gyrY();
  //gyrZ= sensor->gyrZ();
  gx = myICM.gyrX();
  gy = myICM.gyrY();
  gz = myICM.gyrZ();
  calibrate_gyro_data(&gx, &gy, &gz);
}

void calibrate_accel_data(float *ax, float *ay, float *az) 
{
    // Calibra i dati dell'accelerometro
    *ax = (*ax  - accel_bias[0]) * accel_scale[0];  // Dereferenzia e calibra ax
    *ay = (*ay   - accel_bias[1]) * accel_scale[1];  // Dereferenzia e calibra ay
    *az = (*az   - accel_bias[2]) * accel_scale[2];  // Dereferenzia e calibra az
}

void calibrate_gyro_data(float *gx, float *gy, float *gz) 
{
    // Calibra i dati dell'accelerometro
    *gx = (*gx  - gyro_bias[0]) * gyro_scale[0];  // Dereferenzia e calibra ax
    *gy = (*gy   - gyro_bias[1]) * gyro_scale[1];  // Dereferenzia e calibra ay
    *gz = (*gz   - gyro_bias[2]) * gyro_scale[2];  // Dereferenzia e calibra az
}



