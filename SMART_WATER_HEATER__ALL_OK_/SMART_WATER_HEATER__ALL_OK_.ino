
#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#define ONE_WIRE_BUS D4
#define LED_PIN D8 // Define the pin for the first LED
#define LED2_PIN D0 // Define the pin for the second LED
#define TRIGGER_PIN D7 // Define the trigger pin for the ultrasonic sensor
#define ECHO_PIN D6 // Define the echo pin for the ultrasonic sensor
#define LED_DURATION 30000 // LED duration in milliseconds
#define MAX_DISTANCE 100 // Maximum distance in centimeters

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

LiquidCrystal_I2C lcd(0x27, 16, 2);

bool ledOn = false; // Flag to track first LED state
bool led2On = false; // Flag to track second LED state
bool ledTurnedOn = false; // Flag to track if first LED has been turned on
unsigned long ledStartTime = 0; // Track time when first LED was turned on
unsigned long obstacleStartTime = 0; // Track time when obstacle was detected
unsigned long obstacleStopTime = 0; // Track time when obstacle was removed

void setup() {
  sensors.begin();

  lcd.init(); // Initialize the LCD
  lcd.backlight(); // Open the backlight

  Serial.begin(9600);
  Serial.println("Temperature");

  pinMode(LED_PIN, OUTPUT); // Set first LED pin as output
  pinMode(LED2_PIN, OUTPUT); // Set second LED pin as output
  pinMode(TRIGGER_PIN, OUTPUT); // Set trigger pin as output
  pinMode(ECHO_PIN, INPUT); // Set echo pin as input
}

void loop() {
  // Measure temperature and update LCD (always happens)
  sensors.requestTemperatures();
  float tempC = sensors.getTempCByIndex(0);

  // Display temperature on LCD
    lcd.clear(); // Clear the LCD screen
    lcd.setCursor(0, 0);
    lcd.print("Water Temp is");
    lcd.setCursor(0, 1);
    lcd.print(" W.T: ");
    lcd.print(tempC);
    lcd.print(" C");

  // Check ultrasonic sensor for object presence
  digitalWrite(TRIGGER_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN, LOW);
  
  long duration = pulseIn(ECHO_PIN, HIGH);
  float distance_cm = duration * 0.034 / 2; // Convert duration to distance in cm

  // Print distance to Serial Monitor
  Serial.print("Distance: ");
  Serial.print(distance_cm);
  Serial.println(" cm");

  // Control first LED based on temperature
  if (tempC >= 32 && tempC <= 35) {
    // LED only turns on once and for 30 seconds
    if (!ledTurnedOn && !ledOn) {
      ledTurnedOn = true; // Mark first LED as turned on
      ledOn = true;
      ledStartTime = millis(); // Record first LED turn-on time
      digitalWrite(LED_PIN, HIGH); // Turn on first LED
    }

    // Check first LED duration
    if (ledOn && (millis() - ledStartTime >= LED_DURATION)) {
      ledOn = false;
      digitalWrite(LED_PIN, LOW);
    }
  } else {
    // First LED turns off if temperature is out of range
    if (ledOn) {
      ledOn = false;
      digitalWrite(LED_PIN, LOW);
    }
  }

  // Control second LED (buzzer) based on obstacle detection
  if (distance_cm < MAX_DISTANCE) {
    // Obstacle detected
    if (!led2On) {
      led2On = true;
      digitalWrite(LED2_PIN, HIGH); // Turn on second LED (buzzer)
    }
    // Turn off first LED if obstacle detected
    if (ledOn) {
      ledOn = false;
      digitalWrite(LED_PIN, LOW); // Turn off first LED immediately
      obstacleStartTime = millis(); // Record obstacle detection time
    }
  } else {
    // Obstacle removed
    if (led2On) {
      led2On = false;
      digitalWrite(LED2_PIN, LOW); // Turn off second LED (buzzer)
    }
    if (!ledOn && obstacleStartTime > 0) {
      // Resume first LED timer from where it stopped before obstacle was detected
      ledOn = true;
      ledStartTime += millis() - obstacleStartTime;
      obstacleStartTime = 0; // Reset obstacle detection time
      digitalWrite(LED_PIN, HIGH); // Turn on first LED
    }
  }

  delay(1000); // Delay for 1 second
}
