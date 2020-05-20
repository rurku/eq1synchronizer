// upload command
// arduino_debug.exe --upload eq1synchronizer.ino --board myboards:avr:uno1Mhz --port COM4 2>&1 | grep -v StatusLogger

// Circuit design https://easyeda.com/rurku/eq2-motor-clock-extension

#define ASSR_READY_MASK (_BV(TCN2UB) | _BV(OCR2AUB) | _BV(OCR2BUB) | _BV(TCR2AUB) | _BV(TCR2BUB))
#define PIN_FAST A5
#define PIN_PWM 6

typedef enum OpMode {
  Tracking,
  Override
} OpMode;


unsigned long volatile totalCount = 0;

void setup() {
  // put your setup code here, to run once:

  Serial.begin(9600);           //  setup serial

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

  sei();

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

unsigned char duty;

// Motor makes 1 rotation in approx. 137100 pulses
// worm gear has 100 teeth, so we want 13710000 pulses per earth's rotation
// Earth makes a full rotation in 86164s
float trackingRate = 159.115175711434;

unsigned char trackingDuty = 40;
unsigned char overrideDuty = 0;

OpMode opMode = Tracking;
bool fast; // last status of fast button
bool isSync = false;
unsigned long syncPulses;
unsigned long syncSeconds;

unsigned long countLastSecond = 0;

void track() {
  opMode = Tracking;
  isSync = false;
  syncPulses = 0;
  syncSeconds = 0;
}

unsigned long overrideStartMillis;
unsigned long overrideMilliseconds;
void override(unsigned char duty, unsigned long milliseconds)
{
  opMode = Override;
  overrideDuty = duty;
  overrideStartMillis = millis();
  overrideMilliseconds = milliseconds;
}

void writeMetrics(long diff) {
  char status = '!';
  if (opMode == Override) {
    status = 'o'; // override
  } else if (opMode == Tracking) {
    if (isSync) {
      status = 't'; // tracking
    } else {
      status = 's'; // seeking
    }
  }
  Serial.print(status);
  Serial.print(diff);
  Serial.print(' ');
  Serial.println(duty);
}

void optimizeTrackingDuty(long diff) {
  if (abs(diff - trackingRate) / trackingRate < 0.05) {
    isSync = true;
    syncPulses += diff;
    syncSeconds += 1;
    if (syncPulses > trackingRate * syncSeconds) {
      if (diff > trackingRate) {
        trackingDuty = max(trackingDuty - 1, 0);
      }
    } 
    else {
      if (diff < trackingRate) {
        trackingDuty = min(trackingDuty + 1, 255);
      }
    }
  } 
  else {
    isSync = false;
    syncPulses = 0;
    syncSeconds = 0;
    if (diff > 0) { // Make adjustments only if the motor is actually running
      if (diff > trackingRate) {
        trackingDuty = max(trackingDuty - 1, 0);
      }
      else {
        trackingDuty = min(trackingDuty + 1, 255);
      }
    }
  }
}

char commandBuffer[20];
unsigned char commandPos;
unsigned char readPos;

long readInt() {
  // skip leading whitespace
  while (readPos < commandPos && isWhitespace(commandBuffer[readPos])) {
    readPos++;
  }
  bool isNegative = false;
  long result = 0;
  if (readPos < commandPos) {
    if (commandBuffer[readPos] == '-') {
      isNegative = true;
      readPos++;
    }
    
    while (readPos < commandPos && isDigit(commandBuffer[readPos])) {
      result = result * 10 + commandBuffer[readPos] - '0';
      readPos++;
    }
  }
  if (isNegative) {
    return -result;
  } else {
    return result;
  }
}

float readFloat() {
  // skip leading whitespace
  while (readPos < commandPos && isWhitespace(commandBuffer[readPos])) {
    readPos++;
  }
  bool isNegative = false;
  float result = 0;
  if (readPos < commandPos) {
    if (commandBuffer[readPos] == '-') {
      isNegative = true;
      readPos++;
    }
    
    while (readPos < commandPos && isDigit(commandBuffer[readPos])) {
      result = result * 10 + commandBuffer[readPos] - '0';
      readPos++;
    }
    if (readPos < commandPos && commandBuffer[readPos] == '.') {
      readPos++;
      // now read the fractional part
      float weight = 0.1;
      while (readPos < commandPos && isDigit(commandBuffer[readPos])) {
        result = result + (commandBuffer[readPos] - '0') * weight;
        weight = weight / 10;
        readPos++;
      }
    }
  }

  if (isNegative) {
    return -result;
  } else {
    return result;
  }
}

void processCommand() {
  if (Serial.available() > 0) {
    int c = Serial.read();
    if (c == '\n' || c == '\r') {
      // command complete - process it
      if (commandPos > 0) {
        if (commandBuffer[0] == 'o') { // override format o [duty(unsigned char)] [milliseconds (unsigned long)]
          readPos = 1;
          unsigned char ovrDuty = readInt();
          unsigned long ovrMilliseconds = readInt();
          override(ovrDuty, ovrMilliseconds);
          Serial.println("ack");
        }
        else if (commandBuffer[0] == 't' ) { // track format t [trackingRate(float)]}
          readPos = 1;
          trackingRate = readFloat();
          track();
          Serial.println("ack");
        }
      }
      commandPos = 0;
      readPos = 0;
    }
    else {
      if (commandPos < sizeof(commandBuffer)) {
        commandBuffer[commandPos] = (unsigned char)c;
        commandPos ++;
      }
    }
  }
}

void loop() {
  unsigned long count;
  if (countReady) {
    cli();
    count = countThisSecond;
    countReady = false;
    sei();
    long diff = count - countLastSecond;
    countLastSecond = count;

    writeMetrics(diff);

    if (opMode == Tracking)
    {
      optimizeTrackingDuty(diff);
    }

    // the LED will toggle in TIMER2_COMPA_vect. If isSync = true then LED will be on for 1/8s and off for the rest of the cycle. If isSync = false then LED will be off for 1/8s and on for the rest of the cycle.
    digitalWrite(LED_BUILTIN, isSync ? HIGH : LOW);
  }

  processCommand();

  if (digitalRead(PIN_FAST) == LOW && !fast) {
    fast = true;
    override(255,0);
  }
  if (digitalRead(PIN_FAST) == HIGH && fast) {
    fast = false;
    track();
  }

  // check override time
  if (opMode == Override) {
    if (overrideMilliseconds > 0 && millis() - overrideStartMillis > overrideMilliseconds) {
      track();
    }
  }

  switch (opMode) {
    case Tracking:
      duty = trackingDuty;
      break;
    case Override:
      duty = overrideDuty;
      break;
  }

  analogWrite(PIN_PWM, duty);
}
