void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);           //  setup serial

  pinMode(2, INPUT_PULLUP);
  pinMode(A5, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(2), countInterrupt, FALLING);

}

int count = 0;
// volatile unsigned long totalCount = 0;
volatile bool measureReady = false;
// Motor makes 1 rotation in approx. 137100 pulses
// worm gear has 100 teeth, so we want 13710000 pulses per earth's rotation
// Earth makes a full rotation in 86164s
#define CYCLE_DESIRED_MICROS 1005561
#define PULSES_PER_CYCLE 160

unsigned char duty = 15;
unsigned long lastMicros, nowMicros;
volatile unsigned long diff;

void countInterrupt() {
  count ++;
  // totalCount ++;
  if (count == PULSES_PER_CYCLE) {
    count = 0;
    nowMicros = micros();
    diff = nowMicros - lastMicros;
    lastMicros = nowMicros;
    measureReady = true;
  }
}

// void write(unsigned char trend, int peak, unsigned long timePeak) {
//   if (Serial.availableForWrite() < 20 && !flushing) {
//     flushing = 1;
//     Serial.println("flushing");
//   }
//   if (flushing && Serial.availableForWrite() == 63) {
//     flushing = 0;
//   }

//   if (!flushing) {
//     Serial.print(trend);
//     Serial.print(' ');
//     Serial.print(peak);
//     Serial.print(' ');
//     Serial.println(timePeak);
//   }
// }

int avgCount = 0;
unsigned long sumDiff = 0;

// unsigned long lastMicros2;
void loop() {

  // if (digitalRead(A5) == LOW) {
  //   Serial.println(totalCount);
  //   totalCount = 0;
  //   Serial.println("reset");
  //   delay(3000);
  // }
  // unsigned long mic = micros();
  // if ((mic % 1000000) < (lastMicros2 % 1000000))
  // {
  //   Serial.println(totalCount);
  // }
  // lastMicros2 = mic;  
  
  while (!measureReady) {};
  measureReady = false;
  if (diff > CYCLE_DESIRED_MICROS && duty < 255) {
    duty ++;
  } else if (diff < CYCLE_DESIRED_MICROS && duty > 0)
  {
    duty --;
  }
  analogWrite(3, duty);
  Serial.print(diff);
  Serial.print(' ');
  Serial.println(duty);

  avgCount++;
  sumDiff += diff;
  if (avgCount == 10) {
    Serial.println(sumDiff / 10);
    avgCount = 0;
    sumDiff = 0;
  }

}
