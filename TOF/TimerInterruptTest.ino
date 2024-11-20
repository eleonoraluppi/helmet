#include "NRF52TimerInterrupt.h"

#ifndef LED_BLUE_PIN
  #if defined(LED_BLUE)
    #define LED_BLUE_PIN          LED_BLUE
  #else
    #define LED_BLUE_PIN          7
  #endif
#endif


#define TIMER1_INTERVAL_MS        500     //ogni quanto l'interrupt viene richiamato

 
volatile uint32_t preMillisTimer1 = 0; //perchè non è necessario ottimizzare questa variabile, dato che cambia in continuazione

static bool toggle1 = false;

// Depending on the board, you can select NRF52 Hardware Timer from NRF_TIMER_1-NRF_TIMER_4 (1 to 4)
// If you select the already-used NRF_TIMER_0, it'll be auto modified to use NRF_TIMER_1

// Init NRF52 timer NRF_TIMER4
NRF52Timer ITimer1(NRF_TIMER_4);

void TimerHandler1()
{
  preMillisTimer1 = millis();

  //timer interrupt toggles outputPin
  digitalWrite(LED_BLUE_PIN, toggle1);
  toggle1 = !toggle1;
  Serial.println();
}

void setup()
{

  pinMode(LED_BLUE_PIN, OUTPUT);

  Serial.begin(115200);

  while (!Serial && millis() < 5000);

  delay(100);

  Serial.print(F("\nStarting TimerInterruptTest on "));
  Serial.println(NRF52_TIMER_INTERRUPT_VERSION);
  Serial.print(F("CPU Frequency = "));
  Serial.print(F_CPU / 1000000);
  Serial.println(F(" MHz"));


  // Interval in microsecs (1.000.000 microsecs= 1 sec)
  //abilitare ka callback con la sua durata
  if (ITimer1.attachInterruptInterval(TIMER1_INTERVAL_MS * 1000, TimerHandler1))
  {
    preMillisTimer1 = millis();
    Serial.print(F("Starting ITimer1 OK, millis() = "));
    Serial.println(preMillisTimer1);
  }
  else
    Serial.println(F("Can't set ITimer1. Select another freq. or timer"));
}

void loop()
{

  // static unsigned long lastTimer1   = 0;
  // static bool timer1Stopped         = false;

  // if (millis() - lastTimer1 > TIMER1_DURATION_MS)
  // {
  //   lastTimer1 = millis();

  //   if (timer1Stopped)
  //   {
  //     preMillisTimer1 = millis();
  //     Serial.print(F("Start ITimer1, millis() = "));
  //     Serial.println(preMillisTimer1);
  //     ITimer1.restartTimer();
  //   }
  //   else
  //   {
  //     Serial.print(F("Stop ITimer1, millis() = "));
  //     Serial.println(millis());
  //     ITimer1.stopTimer();
  //   }

  //   timer1Stopped = !timer1Stopped;
  //}
}
