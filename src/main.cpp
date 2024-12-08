#include "Arduino.h"
#include "esp_camera.h"
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <ESP32Servo.h>
#include <base64.h>

#define CAMERA_MODEL_AI_THINKER 

#include "camera_pins.h"
#include "secrets.h"

// Define constant
#define ESP32CAM_LED_FLASH 4
#define AWS_IOT_PUBLISH_TOPIC   "esp32/pub"      // Change data
#define AWS_IOT_SUBSCRIBE_TOPIC "esp32/sub"      // Change data
#define SOUND_SPEED 0.034
// #define CM_TO_INCH 0.393701
#define LIMIT_DISTANCE 50

const int trigPin = 14;
const int echoPin = 15;
const int servoPin = 13;
bool inDock = false;
bool isProcessing = false;
int countWaiting = 0;

// Declear methods
void initCamera();
void connectWiFi();
void connectAWS();
void controlServo(boolean);
boolean detectedObjectToCapture(boolean);
void messageHandler(char* topic, byte* payload, unsigned int length);
boolean publishMessage(String buffer);
void initServo();
void controlSystem();
boolean captureImageAndSend();
void autoCloseBarie(int);


WiFiClientSecure net = WiFiClientSecure();
PubSubClient client(net);
Servo servo;

void setup() {
  // Initialize camera
  initCamera();

  initServo();

  // Connect WiFi
  connectWiFi();

  // Connect AWS
  connectAWS();

  Serial.println("System setting successfully!");
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("------------------Start Loop------------------");
  controlSystem();
  client.loop();
  autoCloseBarie(countWaiting);
  Serial.println("------------------End   Loop------------------");
  countWaiting = countWaiting + 1;
  delay(1000);
}

void autoCloseBarie(int time) {
  if (time > 10) {
    controlServo(false);
    countWaiting = 0;
  }
}

boolean captureImageAndSend() {
  Serial.println("captureImageAndSend: start");
  bool status = false;
  digitalWrite(ESP32CAM_LED_FLASH, HIGH);
  camera_fb_t *fb = NULL;
  fb = esp_camera_fb_get();

  if (!fb){
    Serial.println("Camera capture failed");
    delay(2000);
    Serial.println("captureImageAndSend: end");
    return status;
  }
  else {
    Serial.println("Camera Captured");
  }

  digitalWrite(ESP32CAM_LED_FLASH, LOW);

  String buffer = base64::encode((uint8_t *)fb->buf, fb->len);

  unsigned int length();
  Serial.println("Buffer Length: ");
  Serial.print(buffer.length());
  Serial.println("");

  if (buffer.length() > 30000)
  {
    Serial.println("Image size too big");
    buffer = "";
    delay(2000);
    return status;
  }

  Serial.print("Publishing...");
  Serial.print("Publishing...");
  Serial.println("");
  if (publishMessage(buffer)) {
    status = true;
    Serial.print("Published");
  }
  else {
    Serial.println("Error");
  }

  buffer = "";
  esp_camera_fb_return(fb);

  Serial.println("captureImageAndSend: end");
  return status;
}

void controlSystem() {
  if (inDock == true) {
    Serial.println("Dock has been rented!");
    return;
  }

  if (isProcessing == true) {
    Serial.println("In processing...");
    return;
  }

  if (detectedObjectToCapture(true)) {
    if (captureImageAndSend()) {
      isProcessing = true;
    }
    return;
  }
}

void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.println("Connecting to Wi-Fi");
 
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.println(".");
  }

  Serial.println("Connectted to Wi-Fi");
}

void initServo() {
  servo.attach(servoPin, 500, 2400);

  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input
}

void messageHandler(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.println(topic);
  Serial.println("Processing message...");

  char jsonBuffer[length + 1];
  memcpy(jsonBuffer, payload, length);
  jsonBuffer[length] = '\0'; 

  StaticJsonDocument<200> doc;
  DeserializationError error = deserializeJson(doc, jsonBuffer);

  if (error) {
    Serial.print("JSON parse failed: ");
    Serial.println(error.c_str());
    return;
  }

  const char *controlBarie = doc["controlBarie"];
  if (controlBarie) {
    Serial.print("Control barie value: ");
    Serial.println(controlBarie);
    if (strcmp(controlBarie, "open") == 0) {
      controlServo(true);
      inDock = true;
    } else if (strcmp(controlBarie, "close") == 0) {
      controlServo(false);
    } else {
      Serial.print("Unknown command: ");
      Serial.println(controlBarie);
    }
  } else {
    Serial.println("Key 'control' not found in JSON.");
  }
  isProcessing = false;
}

boolean publishMessage(String buffer) {
  StaticJsonDocument<30100> doc;
  doc["slotId"] = SLOT_ID;
  doc["image"] = buffer;

  String json_payload;
  serializeJson(doc, json_payload);

  if (client.publish(AWS_IOT_PUBLISH_TOPIC, json_payload.c_str())) {
    Serial.println("JSON image sent");
    return true;
  } else {
    Serial.println("Failed to send JSON image");
    return false;
  }
}

void connectAWS() {
  // Configure WiFiClientSecure to use the AWS IoT device credentials
  net.setCACert(AWS_CERT_CA);
  net.setCertificate(AWS_CERT_CRT);
  net.setPrivateKey(AWS_CERT_PRIVATE);

  // Connect to the MQTT broker on the AWS endpoint we defined earlier
  client.setServer(AWS_IOT_ENDPOINT, 8883);

  // Create a message handler
  client.setCallback(messageHandler);

  Serial.println("Connecting to AWS IOT");

  while (!client.connect(THINGNAME))
  {
    Serial.print(".");
    delay(100);
  }
 
  if (!client.connected())
  {
    Serial.println("AWS IoT Timeout!");
    return;
  }

  client.subscribe(AWS_IOT_SUBSCRIBE_TOPIC);
 
  Serial.println("AWS IoT Connected!");
}

void initCamera() {
  Serial.begin(115200);

  Serial.setDebugOutput(true);
  Serial.println();
  Serial.println("MYCO CAMERA V1");
  Serial.println();
  void messageHandler(char* topic, byte* payload, unsigned int length);

  // FLASH LED
  pinMode(ESP32CAM_LED_FLASH, OUTPUT);
  digitalWrite(ESP32CAM_LED_FLASH, LOW);

  // buffer.reserve(32000);
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000; // was at 20
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = FRAMESIZE_QVGA; //800 x 600 necessary for Adafruit IO
  config.jpeg_quality = 10;
  config.fb_count = 2;

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK)
  {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }
  else {
    Serial.println("Camera init success");
  }
}

void controlServo(boolean isOpen) {
  if (isOpen) {
    servo.write(0);
    Serial.println("Open door!");
  } else {
    Serial.println("Close door!");
    servo.write(90);
  }
}

boolean detectedObjectToCapture(boolean test) {
  Serial.println("detectedObjectToCapture: start");
  long duration;
  float distanceCm;

  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);
  Serial.println(duration);
  
  // Calculate the distance
  distanceCm = duration * SOUND_SPEED/2;
  Serial.println(distanceCm);

  if (distanceCm < LIMIT_DISTANCE) {
    Serial.println("Detected object!!!");
    Serial.println("detectedObjectToCapture: end");
    return true;
  }
  Serial.println("detectedObjectToCapture: end"); 
  return false;
}
