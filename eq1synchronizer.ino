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

// All global variables

bool volatile pulsesThisSecondReady = false;
unsigned long volatile pulsesThisSecond = 0;
unsigned long volatile timer2OverflowCount = 0; // timer0 counts crystal clock ticks. With 1/128 prescaler it overflows every second.
unsigned long volatile timer1OverflowCount = 0; // timer1 counts pulses. It's 16 bit counter so 1 overflow = 65536 pulses.

#define TICKS_PER_SECOND 256

unsigned char duty;

OpMode opMode = Tracking;
bool fast; // last status of fast button

// tracking

// Motor makes 1 rotation in approx. 137100 pulses
// worm gear has 100 teeth, so we want 13710000 pulses per earth's rotation
// Earth makes a full rotation in 86164s
#define SIDEREAL_PULSES_PER_SECOND 159.115175711434
float trackingRate = SIDEREAL_PULSES_PER_SECOND;

#define ARCSEC_PER_PULSE (((360.0 / 86164) * 60 * 60) / SIDEREAL_PULSES_PER_SECOND)

unsigned char dutyLow = 0; // by only using these duty cycles we're actually not using PWM output but simply switching putput high and low
unsigned char dutyHigh = 255;

// state of ticks and pulses when tracking started
unsigned long trackingInitialPulses;
unsigned long trackingInitialTicks; // tick = 1/256s with 1/128 prescaler

// Number of pulses we should be at. It's increased in each iteration based on elapsed time, and can be changed by 'm' command.
unsigned long trackingTargetPulses;
// Number of pulses since tracking start that we should be at if only elapsed time was taken into account. It's used to calculate the increment of trackingTargetPulses in next iteration.
unsigned long timeTrackingTargetPulses;


// override
unsigned long overrideStartMillis;
unsigned long overrideMilliseconds;
unsigned char overrideDuty = 0;

// command parsing
char commandBuffer[20];
unsigned char commandPos;
unsigned char readPos;

// used for calculating metrics
unsigned long lastMicros = 0;
unsigned long highMicros = 0;
unsigned long lowMicros = 0;
unsigned long iterations = 0;

unsigned long pulsesLastSecond = 0;


// Timer2 is connected to 32kHz crystal and overflows every second
ISR(TIMER2_OVF_vect)
{
  pulsesThisSecond = pulses();
  timer2OverflowCount++;
  pulsesThisSecondReady = true;
}

// Timer2 Compare A will trigger 1/8s after overflow
ISR(TIMER2_COMPA_vect)
{
  digitalWrite(13, digitalRead(13) == HIGH ? LOW : HIGH);
}



// Timer1 counts motor pulses
ISR(TIMER1_OVF_vect)
{
  timer1OverflowCount ++;
}

unsigned long pulses()
{
  uint8_t oldSREG = SREG;
  cli();
  unsigned long m = timer1OverflowCount;
  unsigned int t = TCNT1;
  // Check if the overflow flag is on. This means that the counter has overflown but interrupt has not run yet. So we must add 65536 to the result.
  if ((TIFR1 & _BV(TOV1)) && t < 65535)
  {
    m++;
  }
  SREG = oldSREG;

  return (m << 16) + t;
}

unsigned long ticks()
{
	unsigned long m;
	uint8_t oldSREG = SREG, t;
	
	cli();
	m = timer2OverflowCount;
	t = TCNT2;

	if ((TIFR2 & _BV(TOV2)) && (t < 255))
		m++;

	SREG = oldSREG;
	
	return (m << 8) + t;
}



void track() {
  opMode = Tracking;
  trackingInitialPulses = pulses();
  trackingInitialTicks = ticks();
  trackingTargetPulses = trackingInitialPulses;
  timeTrackingTargetPulses = 0;
}

void override(unsigned char duty, unsigned long milliseconds)
{
  opMode = Override;
  overrideDuty = duty;
  overrideStartMillis = millis();
  overrideMilliseconds = milliseconds;
}

void writeMetrics(long newPulses) {
  char status = '!';
  if (opMode == Override) {
    status = 'o'; // override
  } else if (opMode == Tracking) {
    status = 't'; // tracking
  }
  float calcDuty = (float)highMicros / (lowMicros + highMicros);
  Serial.print(status);
  Serial.print(newPulses);
  Serial.print(' ');
  Serial.print(calcDuty);
  Serial.print(' ');
  Serial.println(iterations);
}


unsigned char optimizeTrackingDuty() {
  unsigned long ticksNow = ticks();
  unsigned long pulsesNow = pulses();
  unsigned long newTimeTrackingTargetPulses = (ticksNow - trackingInitialTicks) * trackingRate / TICKS_PER_SECOND;
  trackingTargetPulses += newTimeTrackingTargetPulses - timeTrackingTargetPulses;
  timeTrackingTargetPulses = newTimeTrackingTargetPulses;

  if (pulsesNow < trackingTargetPulses) {
    return dutyHigh;
  } else {
    return dutyLow;
  }
}

// move the target by the specified angle relative to current pointing position
void moveTarget(float arcsec) {
  trackingTargetPulses = pulses() + arcsec / ARCSEC_PER_PULSE;
}


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
        else if (commandBuffer[0] == 'm' ) { // move target format m [arcsec(float)]}
          readPos = 1;
          moveTarget(readFloat());
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


void setup() {
  // put your setup code here, to run once:

  Serial.begin(9600);           //  setup serial

    Serial.print("t1oc");
  Serial.print(timer1OverflowCount);


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

  // Clear all interrupt flags. This is done by writing 1 to the bit positions.
  TIFR2 = 0xff; 

  // Enable interrupt on overflow and Output Compare A
  TIMSK2 |= (_BV(TOIE2) | _BV(OCIE2A));


  // Set timer/counter 1 to external source on pin T1 rising edge
  TCCR1A = 0;
  
  TCCR1B = (1 << CS12) | (1 << CS11) | (1 << CS10);
  TCCR1C = 0;
  TCNT1 = 0; // clear the counter
  TIFR1 = 0xff; // Clear all interrupt flags. This is done by writing 1 to the bit positions.
  timer1OverflowCount = 0;
  TIMSK1 = (1 << TOIE1); // enable overflow interrupt
  sei();
}


void loop() {
  unsigned long pulsesThisSecondCopy;

  unsigned long newMicros = micros();
  if (duty == dutyHigh) {
    highMicros += newMicros - lastMicros;
  }
  else {
    lowMicros += newMicros - lastMicros;
  }
  lastMicros = newMicros;
  iterations++;

  if (pulsesThisSecondReady) {
    cli();
    pulsesThisSecondCopy = pulsesThisSecond;
    pulsesThisSecondReady = false;
    sei();
    long newPulsesThisSecond = pulsesThisSecondCopy - pulsesLastSecond;
    pulsesLastSecond = pulsesThisSecondCopy;

    // Reset tracking data if motor is not running. This way it will not try to catch up when it starts again.
    if (newPulsesThisSecond == 0 && opMode == Tracking) {
      track();
    }

    writeMetrics(newPulsesThisSecond);

    highMicros = 0;
    lowMicros = 0;
    iterations = 0;

    // the LED will toggle in TIMER2_COMPA_vect. If tracking is good then LED will be on for 1/8s and off for the rest of the cycle. If not then LED will be off for 1/8s and on for the rest of the cycle.
    digitalWrite(LED_BUILTIN, abs((long)(trackingTargetPulses - pulses())) < trackingRate ? HIGH : LOW);
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

  unsigned char trackingDuty = 0;
  if (opMode == Tracking) {
    trackingDuty = optimizeTrackingDuty();
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
