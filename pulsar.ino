/*  Pulsar - Control a landline phone via USB using an Ag1171 SLIC
    Copyright (C) 2023  Aindrea Bell

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU Affero General Public License as
    published by the Free Software Foundation, either version 3 of the
    License, or (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Affero General Public License for more details.

    You should have received a copy of the GNU Affero General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

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
#define CMD_BUFF_SIZE 30 // Command buffer size
#define CMD_MAX_PARAMS 5 // Maximum number of parameters in a command
#define NO_DATA -1

/* --- TEST CODE --- */
#define DEBUG 0 // Debug pin
/* --- TEST CODE --- */

#include <DigiCDC.h>

typedef struct Cadence {
  byte freq;
  byte numRings;
  int onTime;
  int offTime;
  int pauseTime;
} Cadence;

Cadence ukRing = {25, 2, 400, 200, 2000};
Cadence usRing = {20, 1, 2000, 0, 4000};

struct RingSignalState {
  byte postscalerCounter;
  byte postscalerTicks;
  byte prescalerTicks;
  bool generateRingSignal;
  bool ringSignalRunning;
  bool updateNeeded;
} ringSignalState = {0, 0, 0, false, false, false};

struct CadenceState {
  byte numRings;
  byte onEdges;
  byte offEdges;
  byte pauseEdges;
  byte edgesRemaining;
  int totalSteps;
  int currentStep;
} cadenceState = {0, 0, 0, 0, 255, 0, 0};

struct LineState {
  bool offHook;
  bool ringing;
} lineState = {false, false};

struct CommandState {
  char buffer[CMD_BUFF_SIZE + 1]; // One extra byte for null terminator
  byte index;
} commandState;

ISR(TIMER0_COMPB_vect) {
  ringSignalState.updateNeeded = true;
}

/* --- TEST CODE --- */
void debugFlash(int val) {
  bool oldVal = digitalRead(DEBUG);
  digitalWrite(DEBUG, LOW);
  SerialUSB.delay(5);
  for(int i = 0; i < val; i++) {
    digitalWrite(DEBUG, HIGH);
    SerialUSB.delay(5);
    digitalWrite(DEBUG, LOW);
    SerialUSB.delay(5);
  }
  digitalWrite(DEBUG, oldVal);
}
/* --- TEST CODE --- */

void startTimer0() {
  // Idempotent if timer already running
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
  ringSignalState.postscalerCounter = 0;

  // Enable interrupt on Compare Match B
  TIMSK |= (1 << OCIE0B);
}

void setup() {
  SerialUSB.begin();
  resetCommandBuffer();

  // Setup pin I/O
  pinMode(RM, OUTPUT); 
  pinMode(FR, OUTPUT);
  pinMode(SHK, INPUT);

  // Initialise Ag1171
  digitalWrite(RM, LOW);
  digitalWrite(FR, HIGH);

  initialiseTimer0();
}

void setupRingSignal(byte freq) {
  initialiseTimer0();
  // We need to generate an edge twice every period of the signal
  float edgeFreq = freq * 2;
  float targetTicks = T0_POSTSCALED_F / edgeFreq;
  // Truncate target ticks to get quotient
  ringSignalState.postscalerTicks = byte(targetTicks);
  // Use remainder to get number of prescaler ticks
  ringSignalState.prescalerTicks = byte((targetTicks - ringSignalState.postscalerTicks) * T0_POSTSCALER);

  // Special case for 0 postscaler ticks, as we start off on prescaler ticks
  if(ringSignalState.postscalerTicks == 0) {
    OCR0A = ringSignalState.prescalerTicks - 1;
    OCR0B = OCR0A;
  }
}

void startRingSignal() {
  // Tell Timer0 ISR it's okay to generate a signal
  ringSignalState.generateRingSignal = true;
  ringSignalState.ringSignalRunning = true;
  // Put Ag1171 into ringing mode
  digitalWrite(RM, HIGH);
  // Start ringing signal on Timer0
  startTimer0();
}

// Stop generating a ringing signal but keep the timer going
void stopRingSignal() {
  // Tell update function not to generate a signal anymore at the end of current cycle
  ringSignalState.generateRingSignal = false;

  // Wait for Timer0 to switch FR permanently HIGH then turn off ringing mode
  while(ringSignalState.ringSignalRunning == true) {
    // Keep USB alive
    SerialUSB.refresh();
    // Process timer state
    updateRingSignal();
  }

  // Ag1171 needs a delay of > 10 ms between FR permanently HIGH and RM LOW
  SerialUSB.delay(11);
  digitalWrite(RM, LOW);
}

void updateRingSignal() {
  // Do nothing until timer has fired
  if(ringSignalState.updateNeeded == true) {
    // Last full postscaler tick
    if(ringSignalState.postscalerCounter == ringSignalState.postscalerTicks - 1) {
      // Next interval should be truncated to the remaining number of prescaler
      // ticks
      OCR0A = ringSignalState.prescalerTicks - 1;
      OCR0B = OCR0A;
      ringSignalState.postscalerCounter++;
    }
    // Final postscaler tick (truncated) or not using postscaler
    else if(ringSignalState.postscalerCounter == ringSignalState.postscalerTicks || ringSignalState.postscalerTicks == 0) {
      // All ticks have occurred, either generate an edge or stop generating
      // ringing signal
      if(ringSignalState.generateRingSignal == true) {
        // Generate an edge
        digitalWrite(FR, digitalRead(FR) ^ 1);
      }
      else {
        // Don't generate an edge, make sure FR is set HIGH
        digitalWrite(FR, HIGH);
        // Tell main program we've now stopped generating a ring signal
        ringSignalState.ringSignalRunning = false;
      }

      if(ringSignalState.postscalerTicks > 0) {
        // Reset postscaler if we're using it
        OCR0A = T0_POSTSCALER - 1;
        OCR0B = OCR0A;
        ringSignalState.postscalerCounter = 0;
      }

      // Update cadence timer
      cadenceState.edgesRemaining = cadenceState.edgesRemaining == 0
                                      ? 0
                                      : cadenceState.edgesRemaining - 1;
    }
    // Not at an interesting tick yet
    else {
      ringSignalState.postscalerCounter++;
    }

    ringSignalState.updateNeeded = false;
  }
}

void setupCadence(Cadence *cadence) {
  cadenceState.numRings = cadence->numRings;
  int edgeTime = 1000 / (cadence->freq * 2);
  cadenceState.onEdges = cadence->onTime / edgeTime;
  cadenceState.offEdges = cadence->offTime / edgeTime;
  cadenceState.pauseEdges = cadence->pauseTime / edgeTime;
  cadenceState.edgesRemaining = cadenceState.onEdges;
  cadenceState.totalSteps = 2 * cadence->numRings;
  cadenceState.currentStep = 0;
}

void updateCadence() {
  // Do nothing until end of current cadence step
  if(cadenceState.edgesRemaining == 0) {
    // Next step is last step ("Pause")
    if(cadenceState.currentStep == cadenceState.totalSteps - 2) {
      stopRingSignal();
      cadenceState.edgesRemaining = cadenceState.pauseEdges;
    }
    // Next step is "Off" step
    else if(cadenceState.currentStep % 2 == 0) {
      stopRingSignal();
      cadenceState.edgesRemaining = cadenceState.offEdges;
    }
    // Next step is "On" step
    else if(cadenceState.currentStep % 2 == 1) {
      startRingSignal();
      cadenceState.edgesRemaining = cadenceState.onEdges;
    }

    cadenceState.currentStep = (cadenceState.currentStep + 1) % cadenceState.totalSteps;
  }
}

void startRinging(Cadence *cadence) {
  setupRingSignal(cadence->freq);
  setupCadence(cadence);
  lineState.ringing = true;
  startRingSignal();
}

void stopRinging() {
  stopRingSignal();
  stopTimer0();
  lineState.ringing = false;
}

void updateLine() {
  lineState.offHook = digitalRead(SHK);

  if(lineState.ringing == true && lineState.offHook == true) {
    stopRinging();
  }
}

void ringCommand() {
  if(lineState.ringing == false && lineState.offHook == false) {
    startRinging(&ukRing);
  }
}

void stopCommand() {
  if(lineState.ringing == true) {
    stopRinging();
  }
}

void resetCommandBuffer() {
  for(byte i = 0; i < CMD_BUFF_SIZE; i++) {
    commandState.buffer[i] = 0;
  }
  commandState.index = 0;
  SerialUSB.flush();
}

void executeCommand(char command, byte nParams, int params[]) {

}

void parseCommand() {
  // Parse command
  char command = commandState.buffer[0];
  byte paramIndices[CMD_MAX_PARAMS] = {0};
  byte paramLengths[CMD_MAX_PARAMS] = {0};

  // Parse parameters
  byte param = 0;
  paramIndices[0] = 1;
  for(int i = 1; i < commandState.index; i++) {
    // All params parsed, we can stop now
    if(param >= CMD_MAX_PARAMS) {
      break;
    }
    // If previous character was a delimiter, this character is the start of
    // the next parameter
    else if(commandState.buffer[i - 1] == ',') {
      paramIndices[param] = i;
    }
    
    // Delimiter found! 
    if(commandState.buffer[i] == ','){
      paramLengths[param] = i - paramIndices[param];
      param++;
    }
    // Treat the end of the command buffer like a delimiter
    else if(i == commandState.index - 1) {
      paramLengths[param] = (i + 1) - paramIndices[param];
      break;
    }
  }

  // Get number of parameters
  byte nParams = 0;
  for(byte i = 0; i < CMD_MAX_PARAMS; i++) {
    if(paramLengths[i] == 0) {
      break;
    }
    nParams++;
  }

  /* --- TEST CODE --- */
  SerialUSB.print(F("Command: "));
  SerialUSB.println(command);
  SerialUSB.print(F("nParams: "));
  SerialUSB.println(int(nParams));
  SerialUSB.println(F("Params: "));
  for(byte i = 0; i < nParams; i++) {
    SerialUSB.print(F("  "));
    SerialUSB.print(int(i));
    SerialUSB.print(F(": "));
    for(byte j = 0; j < paramLengths[i]; j++) {
      SerialUSB.print(commandState.buffer[paramIndices[i] + j]);
    }
    SerialUSB.println();
  }
  /* --- TEST CODE --- */

  /*// Numericise parameters
  int params[nParams] = {0};
  char paramBuffer[CMD_BUFF_SIZE - 1];
  for(byte i = 0; i < nParams; i++) {

  }

  // Ready to execute command
  executeCommand(command, nParams, params);*/
}

void updateCommand() {
  // Read and echo
  char nextChar = SerialUSB.read();
  SerialUSB.write(nextChar);

  // Nothing to be read, so nothing to be done
  if(nextChar == NO_DATA) {
    return;
  }
  // Parse line as command
  else if(nextChar == '\n') {
    if(commandState.index == 0) {
      return;
    }
    else {
      parseCommand();
      resetCommandBuffer();
    }
  }
  // Command is too long
  else if(commandState.index >= CMD_BUFF_SIZE){
    resetCommandBuffer();
    SerialUSB.println(F("\nERROR: Command too long!"));
  }
  // Character is part of command
  else {
    commandState.buffer[commandState.index] = nextChar;
    commandState.index++;
  }
}

void loop() {
  if(SerialUSB.available()) {
    updateCommand();
  }
  updateLine();
  updateRingSignal();
  updateCadence();
}
