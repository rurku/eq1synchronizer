// upload command
// arduino_debug.exe --upload eq1synchronizer.ino --board myboards:avr:uno1Mhz 2>&1 | grep -v StatusLogger

#include <LowPower.h>

//#define DEBUG

#define ASSR_READY_MASK (_BV(TCN2UB) | _BV(OCR2AUB) | _BV(OCR2BUB) | _BV(TCR2AUB) | _BV(TCR2BUB))
#define PIN_FAST 9
#define PIN_PWM 6


unsigned long volatile totalCount = 0;

void setup() {
  // put your setup code here, to run once:

  #ifdef DEBUG
  Serial.begin(9600);           //  setup serial
  #endif

  //pinMode(5, INPUT_PULLUP); // Pin 5 is T1 in atmega328 - timer 1 external clock input
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(PIN_FAST, INPUT_PULLUP);

  cli();

  // enable asynchronous operation of timer/counter 2
  // this may corrupt values of TCNT2, OCR2x, and TCCR2x
  ASSR |= _BV(AS2);
  
  // Wait until registers are ready for write
  while (ASSR & ASSR_READY_MASK);
  
  // Reset registers to initial values
  TCCR2A = 0;
  TCCR2B = 0;
  TCNT2 = 0;
  OCR2A = 0;
  OCR2B = 0;
  
  // Wait until registers are ready for write again
  while (ASSR & ASSR_READY_MASK);

  // Set prescaler to 1/128 so it overflows every second with 32,768 kHz crystal
  TCCR2B = (1 << CS22) | (0 << CS21) | (1 << CS20);
  // Trigger Output Compare interrupt 1/8 of a second after overflow, to toggle the LED
  OCR2A = 32; // For some reason the value wasn't always latching properly when I set it to 32 from the start. Setting it to 0 first and then to 32 seems to fix it.

  // Wait for new values to latch properly
  while (ASSR & ASSR_READY_MASK);

  // Clear all interrupt flags
  TIFR2 = 0;

  // Enable interrupt on overflow and Output Compare A
  TIMSK2 |= (_BV(TOIE2) | _BV(OCIE2A));


  // Set timer/counter 1 to external source on pin T1 rising edge
  TCCR1A = 0;
  
  TCCR1B = (1 << CS12) | (1 << CS11) | (1 << CS10);
  TCCR1C = 0;
  TCNT1 = 0; // clear the counter
  TIFR0 = 0; // clear any interrupt flags
  totalCount = 0;
  TIMSK1 = (1 << TOIE1); // enable overflow interrupt

  #ifndef DEBUG
  // Disable interrupt of timer0 - it's used for time functions like micros() and delay() but we're not using them (i think)
  TIMSK0 &= ~_BV(TOIE0);
  #endif

  sei();

}

volatile bool dontSleep = false;
void noSleep()
{
  dontSleep = true;
}

bool volatile countReady = false;
unsigned long volatile countThisSecond = 0;
unsigned long volatile seconds = 0;

// Timer2 is connected to 32kHz crystal and overflows every second
ISR(TIMER2_OVF_vect)
{
  countThisSecond = getTotalCount();
  seconds++;
  countReady = true;
}

// Timer2 Compare A will trigger 1/8s after overflow
ISR(TIMER2_COMPA_vect)
{
  digitalWrite(13, digitalRead(13) == HIGH ? LOW : HIGH);
}

// Timer1 counts motor pulses
ISR(TIMER1_OVF_vect)
{
  totalCount += 65536;
}

unsigned long getTotalCount()
{
  uint8_t oldSREG = SREG;
  cli();
  unsigned long total = totalCount;
  unsigned int t = TCNT1;
  total += t;
  // Check if the overflow flag is on. This means that the counter has overflown but interrupt has not run yet. So we must add 65536 to the result.
  if (TIFR1 & _BV(TOV1) && t < 65535)
  {
    total += 65536;
  }
  SREG = oldSREG;
  return total;
}

int count = 0;

// Motor makes 1 rotation in approx. 137100 pulses
// worm gear has 100 teeth, so we want 13710000 pulses per earth's rotation
// Earth makes a full rotation in 86164s
#define PULSES_PER_SECOND 159.115175711434 //159.115175711434

unsigned char duty = 40;
unsigned long diff;

bool isSync = false;
unsigned long syncPulses;
unsigned long syncSeconds;

unsigned long countLastSecond = 0;

void loop() {
  unsigned long count;
  bool fast = digitalRead(PIN_FAST) == LOW;
  if (countReady && !fast) {
    cli();
    count = countThisSecond;
    countReady = false;
    sei();

    long diff = count - countLastSecond;
    if (abs(diff - PULSES_PER_SECOND) / PULSES_PER_SECOND < 0.05) 
    {
      isSync = true;
      syncPulses += diff;
      syncSeconds += 1;
      if (syncPulses > PULSES_PER_SECOND * syncSeconds) 
      {
        if (diff > PULSES_PER_SECOND)
        {
          duty = max(duty - 1, 20);
        }
      } 
      else
      {
        if (diff < PULSES_PER_SECOND)
        {
          duty = min(duty + 1, 255);
        }
      }
    } 
    else
    {
      isSync = false;
      syncPulses = 0;
      syncSeconds = 0;
      if (diff > 0) // Make adjustments only if the motor is actually running
      {
        if (diff > PULSES_PER_SECOND)
        {
          duty = max(duty - 1, 20);
        }
        else
        {
          duty = min(duty + 1, 255);
        }
      }  
    }

    countLastSecond = count;

    analogWrite(6, duty);
    // the LED will toggle in TIMER2_COMPA_vect. If isSync = true then LED will be on for 1/8s and off for the rest of the cycle. If isSync = false then LED will be off for 1/8s and on for the rest of the cycle.
    digitalWrite(13, isSync ? HIGH : LOW);
    
    #ifdef DEBUG
    Serial.print(diff);
    Serial.print(' ');
    Serial.println(duty);
    #endif //DEBUG
  }

  if (fast) 
  {
    analogWrite(PIN_PWM, 255);
    isSync = false;
    syncPulses = 0;
    syncSeconds = 0;
  }

  #ifndef DEBUG
  LowPower.idle(SLEEP_120MS, ADC_OFF, TIMER2_ON, TIMER1_ON, TIMER0_ON, SPI_OFF, USART0_OFF, TWI_OFF);
  #endif
}
