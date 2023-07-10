#define F_CPU 16500000UL // CPU frequency
#define T0_CTC_MODE (1 << WGM01)
#define T0_PRESCALER  1024
#define T0_POSTSCALER 256
#define T0_PRESCALED_F  float(F_CPU) / float(T0_PRESCALER)
#define T0_POSTSCALED_F float(T0_PRESCALED_F) / float(T0_POSTSCALER)
#define T0_PRESCALER_1024 (1 << CS02) | (0 << CS01) | (1 << CS00)
#define T0_STOP (1 << TSM) 
#define T0_RESET_PRESCALER (1 << PSR0)
#define RM 0 // Ring mode on HIGH
#define FR 1 // Reverse A and B (Tip and Ring pins on Ag1171) output polarities on LOW
#define SHK 2 // Indicates off-hook condition when HIGH
#define DEBUG 0 // Debug pin

typedef struct Cadence {
  byte freq;
  byte numRings;
  byte onTime_periods;
  byte offTime_periods;
  byte pauseTime_periods;
} Cadence;

struct CadenceState {
  byte ticksRemaining;
  byte step;
  bool on;
} cadenceState = {0, 0 , false};

byte postscalerCounter = 0;
byte postscalerTicks = 0;
byte prescalerTicks = 0;
bool generateRingSignal = false;
bool ringSignalRunning = false;

ISR(TIMER0_COMPB_vect) {
  if(postscalerCounter == postscalerTicks - 1) {
    // Last full postscaler tick has occurred, next interval should be
    // truncated to the remaining number of prescaler ticks
    OCR0A = prescalerTicks - 1;
    OCR0B = OCR0A;
    postscalerCounter++;
  }
  else if(postscalerCounter == postscalerTicks || postscalerTicks == 0) {
    // All ticks have occurred, either generate an edge or stop generating
    // ringing signal
    if(generateRingSignal == true) {
      // Generate an edge
      digitalWrite(FR, digitalRead(FR) ^ 1);
    }
    else {
      // Don't generate an edge, make sure FR is set HIGH
      digitalWrite(FR, HIGH);
      // Tell main program we've now stopped generating a ring signal
      ringSignalRunning = false;
    }

    if(postscalerTicks > 0) {
      // Reset postscaler if we're using it
      OCR0A = T0_POSTSCALER - 1;
      OCR0B = OCR0A;
      postscalerCounter = 0;
    }

    // Update cadence timer
    /*cadenceState.ticksRemaining = cadenceState.ticksRemaining == 0
                                    ? 0 : cadenceState.ticksRemaining - 1;*/
  }
  else {
    // Not at an interesting tick yet
    postscalerCounter++;
  }
}

void debugFlash(int val) {
  bool oldVal = digitalRead(DEBUG);
  digitalWrite(DEBUG, LOW);
  delay(1000);
  for(int i = 0; i < val; i++) {
    digitalWrite(DEBUG, HIGH);
    delay(10);
    digitalWrite(DEBUG, LOW);
    delay(10);
  }
  delay(1000);
  digitalWrite(DEBUG, oldVal);
}

void startTimer0() {
  GTCCR &= ~T0_STOP;
}

void stopTimer0() {
  GTCCR |= T0_STOP | T0_RESET_PRESCALER;
}

void initialiseTimer0() {
  // Reset prescaler counter and stop timer
  stopTimer0();

  // Clear registers
  TCCR0A = 0;
  TCCR0B = 0;
  TCNT0 = 0;

  // CTC mode because the ringing signal is too slow for WGM on T0, so we'll
  // divide the postscaled frequency by the target frequency and use (quotient
  // * postscaler + remainder * prescaler) timer ticks to generate the signal
  TCCR0A |= T0_CTC_MODE;

  // Using Compare Match B Output, so set both compare registers initially to
  // postscaler value to generate an interrupt every tick of the postscaler
  OCR0A = T0_POSTSCALER - 1;
  OCR0B = OCR0A;

  // Set prescaler
  TCCR0B |= T0_PRESCALER_1024;

  // Reset postscaler
  postscalerCounter = 0;

  // Enable interrupt on Compare Match B
  TIMSK |= (1 << OCIE0B);
}

void setup() {
  // Setup pin I/O
  pinMode(RM, OUTPUT); 
  pinMode(FR, OUTPUT);
  pinMode(SHK, INPUT);

  // Initialise Ag1171
  digitalWrite(RM, LOW);
  digitalWrite(FR, HIGH);
}

void setupRingSignal(byte freq) {
  initialiseTimer0();
  // We need to generate an edge twice every period of the signal
  float edgeFreq = freq * 2;
  float targetTicks = T0_POSTSCALED_F / edgeFreq;
  // Truncate target ticks to get quotient
  postscalerTicks = byte(targetTicks);
  // Use remainder to get number of prescaler ticks
  prescalerTicks = byte((targetTicks - postscalerTicks) * T0_POSTSCALER);

  // Special case for 0 postscaler ticks, as we start off on prescaler ticks
  if(postscalerTicks == 0) {
    OCR0A = prescalerTicks - 1;
    OCR0B = OCR0A;
  }
}

void startRingSignal() {
  // Tell Timer0 ISR it's okay to generate a signal
  generateRingSignal = true;
  ringSignalRunning = true;
  // Put Ag1171 into ringing mode
  digitalWrite(RM, HIGH);
  // Start ringing signal on Timer0
  startTimer0();
}

void stopRingSignal() {
  bool noop = 0;

  // Tell Timer0 ISR to stop ringing signal at the end of current cycle
  generateRingSignal = false;

  // Wait for Timer0 to switch FR permanently HIGH then turn off ringing mode
  while(true) {
    // workaround for compiler incorrectly removing no-op loop
    if(ringSignalRunning == true){
      break;
    }
  }

  // Ag1171 needs a delay of > 10 ms between FR permanently HIGH and RM LOW
  delay(11);
  digitalWrite(RM, LOW);
}

void startRinging(int freq) {
  setupRingSignal(freq);
  startRingSignal();
}

void stopRinging() {
  stopRingSignal();
  stopTimer0();
}

void advanceCadence() {

}

void loop() {
  startRinging(25);
  for(int i = 0; i < 4; i++) {
    delay(500);
    stopRingSignal();
    delay(500);
    startRingSignal();
  }
  stopRinging();
  delay(500);
  startRinging(5);
  for(int i = 0; i < 4; i++) {
    delay(500);
    stopRingSignal();
    delay(500);
    startRingSignal();
  }
}
