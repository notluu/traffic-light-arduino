#include <SoftwareSerial.h>

// Pin definitions
const int redLightPin = 13;    // Red traffic light
const int amberLightPin = 12;  // Amber traffic light
const int greenLightPin = 11;  // Green traffic light
const int stopLedPin = 10;     // Stop LED for pedestrians (Red)
const int walkLedPin = 9;      // Walk LED for pedestrians (Green)
const int buzzerPin = 8;       // Buzzer
const int buttonPin = 7;       // Push button
const int softSerialTxPin = 3; // ESP01 Tx
const int softSerialRxPin = 2; // ESP01 Rx

// Create a software serial object for ESP01
SoftwareSerial esp01(softSerialRxPin, softSerialTxPin);

// Timing constants
const unsigned long greenLightDuration = 10000; // Duration for green traffic light (10 seconds)
const unsigned long amberLightDuration = 3000;  // Duration for amber traffic light (3 seconds)
const unsigned long redLightDuration = 2000;    // Duration for red traffic light (2 seconds)
const unsigned long buttonResponseTime = 1500;  // Shortened duration for red pedestrian light (1.5 seconds)

unsigned long buzzerStartTime;
bool buzzerActive = false;

// Debounce delay
const unsigned long debounceDelay = 50;  // Debounce delay in milliseconds
unsigned long lastButtonPress = 0;       // Time of the last button press

bool buttonPressedDuringGreen = false;  // Flag to track if button was pressed during green light for traffic
bool buttonPressed = false;             // Flag to track if button was pressed

bool sensorActive = false;              // Flag to track the sensor state
unsigned long sensorActivatedTime;      // Time when the sensor was activated

int iterationsWithoutButtonPress = 1;   // Counter for iterations without a button press
const int maxIterationsWithoutPress = 2;// Maximum allowed iterations without a button press

void transitionThroughPhases(bool resetCounter = false);

void setup() {
  pinMode(redLightPin, OUTPUT);
  pinMode(amberLightPin, OUTPUT);
  pinMode(greenLightPin, OUTPUT);
  pinMode(stopLedPin, OUTPUT);
  pinMode(walkLedPin, OUTPUT);
  pinMode(buzzerPin, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP);
  esp01.begin(9600);
  Serial.begin(9600); // Start serial communication at 9600 baud rate
}

void loop() {
  // Read and process sensor input
  if (Serial.available() > 0) {
    char incomingByte = Serial.read();
    Serial.print("recieved input");
    Serial.println(incomingByte);
    processSensorInput(incomingByte);
  }

  // Check for button press
  if (digitalRead(buttonPin) == HIGH && millis() - lastButtonPress > debounceDelay) {
    lastButtonPress = millis();
    if (!buttonPressedDuringGreen) {
      buttonPressedDuringGreen = true;
      activateBuzzer();
      iterationsWithoutButtonPress = 0;
      Serial.println("Pedestrian button pressed. Requesting crossing...");
      // Shorten the green light duration and start transition
      shortenGreenLight();
    }
  }

  // Normal operation or keeping the traffic light green
  if (iterationsWithoutButtonPress <= maxIterationsWithoutPress) {
    if (!buttonPressedDuringGreen) {
      // Normal traffic light sequence
      runLightSequence(greenLightDuration, HIGH, LOW, LOW, HIGH, LOW); // Green light
      transitionThroughPhases();
    }
  } else {
    // Keeping the traffic light green
    changeLights(HIGH, LOW, LOW, HIGH, LOW); // Green for traffic, Red for pedestrian
    if (buttonPressedDuringGreen) {
      transitionThroughPhases();
      buttonPressedDuringGreen = false;
    }
  }
}


void shortenGreenLight() {
  // Shorten the green light phase to an immediate transition
  runLightSequence(0, HIGH, LOW, LOW, HIGH, LOW); // Shortened green light
  transitionThroughPhases();
}

void transitionThroughPhases(bool resetCounter = false) {
  Serial.println("Transitioning through traffic light phases...");
  runLightSequence(amberLightDuration, LOW, HIGH, LOW, HIGH, LOW); // Amber light
  Serial.println("Traffic light at amber. Preparing for red phase...");
  runLightSequence(redLightDuration, LOW, LOW, HIGH, LOW, HIGH);   // Red light
  Serial.println("Traffic light at red. Pedestrians may cross.");
  runLightSequence(amberLightDuration, LOW, HIGH, LOW, HIGH, LOW); // Amber light
  Serial.println("Returning to green phase for traffic.");
  changeLights(HIGH, LOW, LOW, HIGH, LOW); // Green light for traffic, Red for pedestrian
  Serial.println("Traffic light phase transition complete.");

  if (resetCounter) {
    iterationsWithoutButtonPress = 0; // Reset the counter only if a button press initiated the transition
  }
}

bool runLightSequence(unsigned long duration, bool green, bool amber, bool red, bool stop, bool walk) {
  unsigned long startTime = millis();
  changeLights(green, amber, red, stop, walk);
  bool buttonPressedDuringThisSequence = false;

  while (millis() - startTime < duration) {
    if (green && digitalRead(buttonPin) == HIGH && !buttonPressed) {
      lastButtonPress = millis();
      buttonPressed = true;
      buttonPressedDuringGreen = true;
      activateBuzzer();
      return true;
    }
  }
  return buttonPressedDuringThisSequence;
}

void changeLights(bool green, bool amber, bool red, bool stop, bool walk) {
  digitalWrite(greenLightPin, green);
  digitalWrite(amberLightPin, amber);
  digitalWrite(redLightPin, red);
  digitalWrite(stopLedPin, stop);
  digitalWrite(walkLedPin, walk);
  
  Serial.println("Lights changed to:");
  Serial.print("  Traffic - Green: "); Serial.print(green ? "ON" : "OFF");
  Serial.print(", Amber: "); Serial.print(amber ? "ON" : "OFF");
  Serial.print(", Red: "); Serial.println(red ? "ON" : "OFF");
  Serial.print("  Pedestrian - Stop: "); Serial.print(stop ? "ON" : "OFF");
  Serial.print(", Walk: "); Serial.println(walk ? "ON" : "OFF");
}

void activateBuzzer() {
  buzzerActive = true;
  buzzerStartTime = millis();
  digitalWrite(buzzerPin, HIGH);
  Serial.println("Buzzer activated for pedestrian crossing request.");
}

void processSensorInput(char input) {
  if (input == '1') {
    sensorActive = true;
    sensorActivatedTime = millis();
    Serial.println("Sensor detected an object. Adjusting traffic lights accordingly...");
  } else {
    Serial.print("Received input: ");
    Serial.println(input);
    Serial.println("No action taken.");
  }
}
