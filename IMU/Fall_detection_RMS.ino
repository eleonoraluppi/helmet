/****************************************************************
 FALL DETECTION
 9-axis IMU data
 ***************************************************************/

#include "ICM_20948.h" // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU
#include "nrf.h"  // Needed to configure the timer
#define SERIAL_PORT Serial
#define WIRE_PORT Wire // Your desired Wire port.      Used when "USE_SPI" is not defined
#define AD0_VAL 1 // The value of the last bit of the I2C address.
// On the SparkFun 9DoF IMU breakout the default is 1, and when the ADR jumper is closed the value becomes 0

ICM_20948_I2C myICM;      // my sampling rate is 723 Hz
float accX, accY, accZ;   
float gyrX, gyrY, gyrZ;
float deltaX = 0, deltaY = 0, deltaZ = 0;  // delta acc (new_acc - old_acc)
float MS_x = 0, MS_y = 0, MS_z = 0;        // to perform the RMS_delta_acc
float MS_x_2 = 0, MS_y_2 = 0, MS_z_2 = 0;  // to perform the RMS_acc
float previous_accX = 0.0;
float previous_accY = 0.0;
float previous_accZ = 0.0;
int thr_acc = 500;   // threshold for impact detection [mg]
int thr_ang = 250;   // threshold for angular inclination [°/s]

const int windowSize = 100;  // dimension of the moving window (impact last 50-200 ms => sampling freq=723Hz => 36-144 samples => 100)
int index_window = 0;  // Index for the arrray
float xWindow[windowSize]={0};  // initialization window for RMS_delta_acc
float yWindow[windowSize]={0};
float zWindow[windowSize]={0};
float xWindow2[windowSize]={0};  // initialization window for RMS_acc
float yWindow2[windowSize]={0};
float zWindow2[windowSize]={0};

// different values for 'stato_IMU'
# define NORMAL 0
# define IMPACT_HIGH_ACC 1
# define IMPACT_HIGH_ANG 2
# define IMPACT_NO_MOVEMENT 3
int stato_IMU = NORMAL;   
int sec=0;  //seconds passed from the impact 






void setup()
{
  SERIAL_PORT.begin(115200);
  while (!SERIAL_PORT)
  {};

  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000);
  //myICM.enableDebugging(); // Uncomment this line to enable helpful debug messages on Serial

  Init_Timer(); //timer initialization
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

// ############################################ timer (NON FUNZIONA, VA SISTEMATO!!)
void Init_Timer()
{
  NRF_TIMER1->TASKS_STOP = 1; // stop the timer
  NRF_TIMER1->MODE = TIMER_MODE_MODE_Timer;  // set the timer in this modality: it increase the counter for each clock
  NRF_TIMER1->PRESCALER = 4;  //set a prescaler: 2^4=16 => 16MHz/16=1MHz
  NRF_TIMER1->CC[0] = 1*1000000;  // Set the overflow limit(MAX): 5 seconds 
  NRF_TIMER1->INTENSET = TIMER_INTENSET_COMPARE0_Msk; //Enable interrupt on COMPARE[0]
  NRF_TIMER1->EVENTS_COMPARE[0] = 0;  // clear any previous event in case it is set
  NVIC_EnableIRQ(TIMER0_IRQn);  //enable timer interrupt in NVIC (Nested Vectored Interrupt Controller )
}
/*
void Init_RTC()   //init real-time clock
{
  NRF_CLOCK->TASKS_LFCLKSTART = 1;    // enable the LFCLK (=32.768KHz)
  while (NRF_CLOCK->EVENTS_LFCLKSTARTED == 0); // wait that the LF clock is ready
  NRF_CLOCK->EVENTS_LFCLKSTARTED = 0;  // deleat the event 

  NRF_RTC1->PRESCALER = 32767;  // set prescaler to 32767 => time_clock=freq/prescaler = 1 second
  NRF_RTC1->CC[0] = 1;  // Set the overflow limit(MAX): 1 second
  NRF_RTC1->INTENSET = RTC_INTENSET_COMPARE0_Enabled << RTC_INTENSET_COMPARE0_Pos;  //enable interrupt on compare[0]
  NRF_RTC1->EVENTS_COMPARE[0] = 0;  // clear any previous event in case it is set
  NVIC_EnableIRQ(RTC1_IRQn); //enable timer interrupt in NVIC (Nested Vectored Interrupt Controller)
}
//EVENTS_LFCLKSTARTED: devo farlo per leggere se effettivamente il clock è partito

extern "C" void MyRTC1_IRQHandler(void) 
{
  Serial.println("interrup");
  if (NRF_RTC1->EVENTS_COMPARE[0] != 0)// if the "event" ()= 5 seconds have passed) has happened. I make this check because i can have 4 ≠ interrupt channels (compare[1,2,3]).
   {
    NRF_RTC1->EVENTS_COMPARE[0] = 0;  // put 'compare' to 0 again
    Serial.println("1 second has passed");
    
    // Reimposta il compare per il prossimo secondo
    NRF_RTC1->CC[0] += 1;  // 1 secondo in più
  }
}*/

extern "C" void TIMER1_IRQHandler(void)   //Interrupt on the timer OVF 
{
  if (NRF_TIMER1->EVENTS_COMPARE[0] != 0) // if the "event" ()= 5 seconds have passed) has happened. I make this check because i can have 4 ≠ interrupt channels (compare[1,2,3]).
  {
    NRF_TIMER1->EVENTS_COMPARE[0] = 0; // put 'compare' to 0 again
    sec ++;
    Serial.println("1 second has passed. t = ");
    Serial.print(sec);
    Serial.println("secondi");
  }
}


// ############################################

void loop()
{
  //  depending on the "stato_IMU" of the IMU, we make different checks
  if (stato_IMU == NORMAL)
  {
    find_impact();  // find if high acc
  }
  else if (stato_IMU == IMPACT_HIGH_ACC) //If an impact has occurred, check the angle 
  {
    find_angle(); // find if high °
  }
  else if (stato_IMU == IMPACT_HIGH_ANG)
  {
    stato_IMU = NORMAL;
    // aspetto 5 secondi dopo l'impatto e controllo se c'è solo g(z)=1 => RMS ~ 0 (di bias è circa sempre 8-9)
    // se è cosi per 5-10 secondi => !!!
  }
}


void find_impact()  // function that search for high acceleration. it identifies an impact if RMS_delta > thr
{                   // !! at the moment i'm performing both RMS and RMS_delta to compare them, then I will choose only one (I think RMS_Delta)
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
      xWindow2[index_window] = accX;
      yWindow2[index_window] = accY;
      zWindow2[index_window] = accZ;

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
      previous_accX = accX;
      previous_accY = accY;
      previous_accZ = accZ;

      if (RMS_delta>thr_acc)  //check the impact
      {
       stato_IMU=IMPACT_HIGH_ACC;
      
       Serial.println("impact detected:: ");
       Serial.print("RMS_delta :");
       Serial.println(RMS_delta);
       Serial.print(" RMS: ");
       Serial.println(RMS);
       
       NRF_RTC1->TASKS_START = 1; //start the timer
      }
    }
}

void find_angle()  //function that search for high angles. it identifies an impact if theta of pitch or roll > thr
{
    if (myICM.dataReady()) 
    { 
      myICM.getAGMT(); 
      read_gyrXYZ(&myICM);
      
      if (abs(gyrX) >thr_ang || abs(gyrY)>thr_ang)// || abs(gyrZ)>thr_ang)     // da modificare con l'angolo se possibile
      {
        stato_IMU = IMPACT_HIGH_ANG;
        Serial.print("High Angolar velocity!!");
        Serial.print(" X :");
        Serial.print(gyrX);
        Serial.print(" Y :");
        Serial.print(gyrY);
        //Serial.print(" Z :");
        //Serial.println(gyrZ);
      }
      if (stato_IMU != IMPACT_HIGH_ANG)  //if theta < thr it was a false alert (FP)
      {
        stato_IMU = NORMAL;   //come back to the normal state
      }
    }
}

void read_accXYZ(ICM_20948_I2C *sensor)
{
  // function that reads the value of accelleration in the 3 directions [mg]
  accX= sensor->accX();  
  accY= sensor->accY();
  accZ= sensor->accZ();
}

void read_gyrXYZ(ICM_20948_I2C *sensor)
{
  //function that reads the angular velocity in the 3 directions [DPS]
  gyrX= sensor->gyrX();
  gyrY= sensor->gyrY();
  gyrZ= sensor->gyrZ();
}




