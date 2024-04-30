#include <SoftwareSerial.h>


// Pin definitions
const int redLightPin = 13;    // Red traffic light
const int amberLightPin = 12;  // Amber traffic light
const int greenLightPin = 11;  // Green traffic light
const int stopLedPin = 10;     // Stop LED for pedestrians (Red)
const int walkLedPin = 9;      // Walk LED for pedestrians (Green)
const int buzzerPin = 8;       // Buzzer
const int buttonPin = 7;       // Push button

// Timing constants
const unsigned long greenLightDuration = 10000; // 10 seconds
const unsigned long shortGreenLightDuration = 1000; // 1 second
const unsigned long amberLightDuration = 3000;  // 3 seconds
const unsigned long redLightDuration = 10000;   // 10 seconds
const unsigned long debounceDelay = 50;         // Debounce delay for the button
const unsigned long longPressDuration = 2000;   // Duration to consider as a long press
static unsigned long lastFlickerTime = 0;
const unsigned long flickerInterval = 200;
bool rainModeActive = false; // Tracks the rain mode status

// State machine and timing
enum TrafficState { GREEN, AMBER, RED, SEMI_PERMANENT_GREEN };
TrafficState state = GREEN;
TrafficState previousState = RED;
unsigned long lastChangeTime = 0;
unsigned long buttonPressTime = 0;
bool buttonPressed = false;
bool shortGreenLightNextCycle = false;

// Cycle counter
int cycleCounter = 0;

void buzzTransition() {
  digitalWrite(buzzerPin, HIGH);
  delay(100); // Short buzz for 100 milliseconds
  digitalWrite(buzzerPin, LOW);
}
void changeLights(bool green, bool amber, bool red, bool stop, bool walk) {
  // Ensure to reset any flickering effect before setting new states
  digitalWrite(greenLightPin, green);
  digitalWrite(amberLightPin, amber);
  digitalWrite(redLightPin, red);
  digitalWrite(stopLedPin, stop);
  digitalWrite(walkLedPin, walk);
}

void setup() {
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT); 
  pinMode(redLightPin, OUTPUT);
  pinMode(amberLightPin, OUTPUT);
  pinMode(greenLightPin, OUTPUT);
  pinMode(stopLedPin, OUTPUT);
  pinMode(walkLedPin, OUTPUT);
  pinMode(buzzerPin, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP);
}

void loop() {
  unsigned long currentTime = millis();

  // Serial input handling for rain mode toggle
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input == "rain") {
      rainModeActive = !rainModeActive; // Toggle rain mode
      if (rainModeActive) {
        Serial.println("Rain mode activated. Rain has started.");
      } else {
        Serial.println("Rain mode deactivated. Rain has stopped.");
      }
    }
  }

  // Button press handling for manual reset
  if (digitalRead(buttonPin) == HIGH) {
    if (!buttonPressed) {
      buttonPressed = true;
      buttonPressTime = currentTime;
    }
  } else if (buttonPressed) {
    buttonPressed = false;
    if ((currentTime - buttonPressTime > debounceDelay) && (state == GREEN)) {
      // If currently in GREEN state and the button is pressed, immediately transition to AMBER
      Serial.println("Button pressed. Shortening current green light.");
      delay(shortGreenLightDuration); // time between state transistion 
      previousState = state;
      state = AMBER;
      lastChangeTime = currentTime; // Update lastChangeTime to ensure immediate transition
      changeLights(LOW, HIGH, LOW, HIGH, LOW); // Activate amber light, indicating the transition
    }
  }

  // Rain mode flickering effect
  static unsigned long lastFlickerTime = 0;
  const unsigned long flickerInterval = 200; // Adjust flicker interval as needed
  if (rainModeActive && (currentTime - lastFlickerTime > flickerInterval)) {
      // Flicker only the active light based on the current state
      switch(state) {
          case GREEN:
              // Flicker only the green light
              digitalWrite(greenLightPin, !digitalRead(greenLightPin));
              break;
          case AMBER:
              // Flicker only the amber light
              digitalWrite(amberLightPin, !digitalRead(amberLightPin));
              break;
          case RED:
              // Flicker only the red light
              digitalWrite(redLightPin, !digitalRead(redLightPin));
              break;
      }
      lastFlickerTime = currentTime;
  }

  // State transitions with buzzer signaling
  switch (state) {
    case GREEN:
      if (currentTime - lastChangeTime > greenLightDuration) {
        // Transition to AMBER
        if (rainModeActive) buzzTransition();
        previousState = state;
        state = AMBER;
        lastChangeTime = currentTime;
        changeLights(LOW, HIGH, LOW, HIGH, LOW);
      }
      break;
    case AMBER:
      if (currentTime - lastChangeTime > amberLightDuration) {
        if (rainModeActive) buzzTransition();
        if (previousState == GREEN) {
          state = RED;
          lastChangeTime = currentTime;
          changeLights(LOW, LOW, HIGH, LOW, HIGH); // Traffic red, pedestrian walk
        } else if (previousState == RED) {
          state = GREEN;
          lastChangeTime = currentTime;
          changeLights(HIGH, LOW, LOW, HIGH, LOW); // Traffic green, pedestrian stop
          cycleCounter++; // Increment cycle counter after completing a full cycle
        }
        previousState = state;
        // Check for semi-permanent green state condition
        if (cycleCounter >= 2) {
          state = SEMI_PERMANENT_GREEN;
        }
      }
      break;
    case RED:
      if (currentTime - lastChangeTime > redLightDuration) {
        if (rainModeActive) buzzTransition();
        previousState = state;
        state = AMBER;
        lastChangeTime = currentTime;
        changeLights(LOW, HIGH, LOW, HIGH, LOW); // Traffic amber, pedestrian stop, preparing for green
      }
      break;

    case SEMI_PERMANENT_GREEN:
      // Keep the green light on and pedestrian stop signal, with optional flickering in rain mode
      if (rainModeActive) {
        static unsigned long lastFlickerTime = 0;
        const unsigned long flickerInterval = 200; // Adjust flicker interval as needed
        if (currentTime - lastFlickerTime > flickerInterval) {
          // Flicker the green light to simulate the effect of rain
          digitalWrite(greenLightPin, !digitalRead(greenLightPin));
          lastFlickerTime = currentTime;
        }
      } else {
        // Ensure the green light is solidly on if not already
        digitalWrite(greenLightPin, HIGH);
        digitalWrite(amberLightPin, LOW);
        digitalWrite(redLightPin, LOW);
        digitalWrite(stopLedPin, HIGH); // Ensure pedestrian stop signal is on
        digitalWrite(walkLedPin, LOW);
      }

      if (buttonPressed) {
        // Reset cycle counter and return to normal operation
        cycleCounter = 0;
        state = GREEN;
        lastChangeTime = currentTime;
        Serial.println("Exiting semi-permanent green state, returning to normal operation.");
        // Ensure lights are correctly set for GREEN state upon exit
        changeLights(HIGH, LOW, LOW, HIGH, LOW);
      }
      break;
  }
}



