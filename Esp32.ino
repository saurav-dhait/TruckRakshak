#include <HardwareSerial.h>
#include <TinyGPS++.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// Create objects for GPS and SIM900A
HardwareSerial sim900Serial(1); // Use Serial1 for SIM900A (GPIO16=RX, GPIO17=TX)
HardwareSerial gpsSerial(2);    // Use Serial2 for GPS (GPIO22=RX, GPIO23=TX)
TinyGPSPlus gps;

// Server details
const char server[] = "ea57cbb1-b2fa-47dc-b1ec-d914e96007f3.mock.pstmn.io";
const int port = 80; // Port for HTTP

// Ultrasonic sensor pins
const int trigPin = 22;
const int echoPin = 23;

// Variables for ultrasonic readings
const int numReadings = 10; // Increased number of readings
float distanceReadings[numReadings];
int readingIndex = 0;

// Function declarations
void initializeSIM900A();
void sendPostToServer(float latitude, float longitude, float* distances, int numDistances);
float getDistance();
void resetHTTP();

// Task handles
TaskHandle_t TaskHandle_DataCollection;
TaskHandle_t TaskHandle_DataTransmission;

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  sim900Serial.begin(9600, SERIAL_8N1, 16, 17); // SIM900A serial at 9600 baud
  gpsSerial.begin(9600, SERIAL_8N1, 18, 19);    // GPS serial at 9600 baud

  // Ultrasonic sensor setup
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Initialize SIM900A
  initializeSIM900A();

  // Create tasks
  xTaskCreatePinnedToCore(
    taskDataCollection, "DataCollection", 4096, NULL, 1, &TaskHandle_DataCollection, 0
  );

  xTaskCreatePinnedToCore(
    taskDataTransmission, "DataTransmission", 4096, NULL, 1, &TaskHandle_DataTransmission, 1
  );
}

void loop() {
  // Empty loop, tasks are handled by FreeRTOS
}

// Task to collect data
void taskDataCollection(void *pvParameters) {
  while (true) {
    while (gpsSerial.available() > 0) {
      gps.encode(gpsSerial.read());
    }

    if (gps.location.isValid()) {
      // Collect ultrasonic readings
      float distance = getDistance();
      distanceReadings[readingIndex] = distance;
      readingIndex = (readingIndex + 1) % numReadings;

      // Continue without printing sensor data
    }

    // Wait for 500 ms before the next sensor reading
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}

// Task to handle data transmission
void taskDataTransmission(void *pvParameters) {
  static unsigned long lastSendTime = 0;

  while (true) {
    unsigned long currentMillis = millis();

    if (currentMillis - lastSendTime >= 60000) { // Send every 1 minute
      lastSendTime = currentMillis;

      // Reset the HTTP session (not GPRS)
      resetHTTP();
      
      // Send the data to the server
      sendPostToServer(gps.location.lat(), gps.location.lng(), distanceReadings, numReadings);
    }

    // Wait for 60 seconds between each send
    vTaskDelay(60000 / portTICK_PERIOD_MS);
  }
}

// Function to initialize the SIM900A module
void initializeSIM900A() {
  Serial.println("Initializing SIM900A...");
  sim900Serial.println("AT"); // Test communication with SIM900A
  delay(1000);

  sim900Serial.println("AT+CPIN?"); // Check SIM card status
  delay(1000);

  sim900Serial.println("AT+CREG?"); // Check network registration
  delay(1000);

  sim900Serial.println("AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\""); // Set GPRS mode
  delay(1000);

  sim900Serial.println("AT+SAPBR=3,1,\"APN\",\"airtelgprs.com\"");  // Set your APN
  delay(1000);

  sim900Serial.println("AT+SAPBR=1,1");  // Activate GPRS context
  delay(1000);

  sim900Serial.println("AT+HTTPINIT");  // Initialize HTTP service
  delay(1000);
}

// Function to reset HTTP session
void resetHTTP() {
  Serial.println("Resetting HTTP session...");
  sim900Serial.println("AT+HTTPTERM"); // Terminate any existing HTTP session
  delay(1000);

  sim900Serial.println("AT+HTTPINIT"); // Reinitialize HTTP session
  delay(1000);
}

// Function to send HTTP POST request
void sendPostToServer(float latitude, float longitude, float* distances, int numDistances) {
  Serial.println("Preparing data for POST...");
  
  // Prepare the data to send in the POST request body
  String postData = "{\"latitude\":" + String(latitude, 6) + ",\"longitude\":" + String(longitude, 6) + ",\"distances\":[";
  for (int i = 0; i < numDistances; i++) {
    postData += String(distances[i], 2);
    if (i < numDistances - 1) postData += ",";
  }
  postData += "]}";

  // Debug: Show data being sent
  Serial.println("Sending the following data to the server:");
  Serial.println(postData);

  // Start HTTP session
  sim900Serial.println("AT+HTTPPARA=\"CID\",1"); // Set GPRS connection ID
  delay(1000);

  // Set the server URL
  sim900Serial.println("AT+HTTPPARA=\"URL\",\"http://" + String(server) + "/post\"");
  delay(1000);

  // Specify content type (application/json)
  sim900Serial.println("AT+HTTPPARA=\"CONTENT\",\"application/json\"");
  delay(1000);

  // Set the POST data size
  sim900Serial.println("AT+HTTPDATA=" + String(postData.length()) + ",10000"); // Data length and timeout
  delay(2000);

  // Send the POST data
  sim900Serial.print(postData);
  delay(10000); // Wait for data to be sent

  // Start the HTTP POST request
  sim900Serial.println("AT+HTTPACTION=1"); // 1 indicates POST request
  delay(15000); // Wait for response

  // Debug: Read the response
  Serial.println("Reading server response...");
  sim900Serial.println("AT+HTTPREAD");
  delay(5000);

  // End HTTP session
  sim900Serial.println("AT+HTTPTERM");
  delay(1000);
}

// Function to measure distance using the ultrasonic sensor
float getDistance() {
  // Send a pulse from the trig pin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Read the echo pin
  long duration = pulseIn(echoPin, HIGH);

  // Calculate the distance in centimeters
  float distance = (duration * 0.034) / 2;

  return distance;
}
