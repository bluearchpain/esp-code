#include <DHT.h>
#include <ESP32Servo.h>
#include <Arduino.h>

#include <Firebase_ESP_Client.h>
#include "addons/TokenHelper.h" //Provide the token generation process info.
#include "addons/RTDBHelper.h"  //Provide the real-time database payload printing info and other helper functions.

#define API_KEY "AIzaSyA-fLHq8BT1xWlFNBPZseQyNag_wE3Aers"
#define DATABASE_URL "mazen-e8d54-default-rtdb.firebaseio.com"

#define WIFI_SSID "1"               //  WiFi SSID
#define WIFI_PASSWORD "123456789" //  WiFi password

FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;
unsigned long sendDataPrevMillis = 0;
int read_data;
bool signupSuccess = false;


#define OUTPUT_PIN 21 // Output pin to trigger when motion is detected
#define PIN_LED 4    // Define the pin number to which LED is connected
#define PIN_SENSOR 33 // Define the pin number to which MQ2 sensor is connected
#define TRIGGER_PIN 18  // Define the trigger pin for HC-SR04
#define ECHO_PIN 19    // Define the echo pin for HC-SR04
#define MOTION_LED 21 // Define the pin number to which lamp is connected
#define DHTPIN 14      // Digital pin connected to the DHT sensor
#define DHTTYPE DHT11 // DHT 11
#define BUZZER_PIN 2 // Define the pin number to which buzzer is connected

Servo servoMotor;
DHT dht(DHTPIN, DHTTYPE);

int thresholdValue = 275; // Define the threshold value for MQ2 sensor reading

void setup()
{

  pinMode(OUTPUT_PIN, OUTPUT);
  digitalWrite(OUTPUT_PIN, LOW); // Ensure output pin is initially low
  servoMotor.attach(15);              // Attaching servo motor to pin 14
  pinMode(PIN_LED, OUTPUT);           // Set pin 26 as output for LED
  pinMode(PIN_SENSOR, INPUT);         // Set pin 36 as input for MQ2 sensor
  pinMode(MOTION_LED, OUTPUT);        // Set pin 27 as output for lamp
  pinMode(TRIGGER_PIN, OUTPUT);       // Set trigger pin for HC-SR04 as output
  pinMode(ECHO_PIN, INPUT);           // Set echo pin for HC-SR04 as input
  pinMode(BUZZER_PIN, OUTPUT);        // Set pin for buzzer as output
  Serial.begin(9600);                 // Initialize serial communication
  dht.begin();
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(300);
  }
  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();

  /* Assign the api key (required) */
  config.api_key = API_KEY;

  /* Assign the RTDB URL (required) */
  config.database_url = DATABASE_URL;

  /* Sign up */
  Serial.println("Signing up...");
  if (Firebase.signUp(&config, &auth, "", ""))
  {
    Serial.println("ok");
    signupSuccess = true;
  }
  else
  {
    Serial.printf("%s\n", config.signer.signupError.message.c_str());
  }

  /* Assign the callback function for the long running token generation task */
  config.token_status_callback = tokenStatusCallback; // see addons/TokenHelper.h

  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);
}

void loop()
{
  int sensorValue = analogRead(PIN_SENSOR); // Read MQ2 sensor value
  Serial.print("MQ2 Sensor value: ");
  Serial.println(sensorValue); // Print MQ2 sensor value
  Firebase.RTDB.setInt(&fbdo, "/gasPercentage", sensorValue);

  if (sensorValue <= thresholdValue)
  { // Check if MQ2 sensor value is above threshold
    digitalWrite(PIN_LED, HIGH); // If MQ2 sensor value is above threshold, turn on LED
  }
  else
  {
    digitalWrite(PIN_LED, LOW); // If MQ2 sensor value is below threshold, turn off LED
  }

  // Ultrasonic sensor code
  digitalWrite(TRIGGER_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH);
  // Convert the time into a distance
  float distance = (duration * 0.034) / 2;

  Serial.print("Distance: ");
  Serial.println(distance);

  if (distance < 15)
  { // If an object is detected within 50cm range (adjust as needed)
    digitalWrite(MOTION_LED, HIGH); // Turn on lamp
    tone(BUZZER_PIN, 1000);         // Turn on buzzer at 1000Hz frequency
    delay(500);                     // Sound buzzer for half a second
    noTone(BUZZER_PIN);             // Turn off buzzer
  }
  else
  {
    digitalWrite(MOTION_LED, LOW); // Turn off lamp
  }

  delay(500); // Delay for stability, adjust as needed

  float humidity = dht.readHumidity();
  float temperature = dht.readTemperature();

  if (isnan(humidity) || isnan(temperature))
  {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }

  Serial.print("Humidity: ");
  Serial.print(humidity);
  Serial.print("%  Temperature: ");
  Serial.print(temperature);
  Serial.println("°C");
  Firebase.RTDB.setInt(&fbdo, "/temperature", temperature);
    Firebase.RTDB.setInt(&fbdo, "/humidity", humidity);


  // If the distance is less than or equal to 40cm, rotate the servo motor 90 degrees
  if (distance <= 15)
  {
    servoMotor.write(90);
    delay(500); // Wait for the servo to reach its position
     Serial.println("Door open"); // Print statement
  }
  else
  {
    // Otherwise, return the servo motor to its initial position
    servoMotor.write(0);
    delay(500); // Wait for the servo to reach its position
     Serial.println("Door closed"); // Print statement
  }

  delay(500); // Wait for 1 second before taking the next reading
}
