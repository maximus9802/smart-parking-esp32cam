#include "Arduino.h"
#include "esp_camera.h"
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <ESP32Servo.h>

#define CAMERA_MODEL_AI_THINKER 

#include "camera_pins.h"
#include "secrets.h"

// Define constant
#define ESP32CAM_LED_FLASH 4
#define AWS_IOT_PUBLISH_TOPIC   ""      // Change data
#define AWS_IOT_SUBSCRIBE_TOPIC ""      // Change data
#define SOUND_SPEED 0.034
#define CM_TO_INCH 0.393701
#define LIMIT_DISTANCE 80

const int trigPin = 14;
const int echoPin = 15;

// Declear methods
int myFunction(int, int);
void initCamera();
void connectWiFi();
void connectAWS();
void controlServo(boolean);
boolean detectedObject();

WiFiClientSecure net = WiFiClientSecure();
PubSubClient client(net);
Servo servo;

void setup() {
  // Initialize camera
  initCamera();

  // Connect WiFi
  connectWiFi();

  // Connect AWS
  connectAWS();

  Serial.println("System setting successfully!");
}

void loop() {
  // put your main code here, to run repeatedly:



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

void messageHandler(char* topic, byte* payload, unsigned int length) {
  Serial.print("Incoming: ");
  Serial.println(topic);
 
  StaticJsonDocument<200> doc;
  deserializeJson(doc, payload);
  const char* message = doc["message"];
  Serial.println(message);
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
  config.frame_size = FRAMESIZE_SVGA; //800 x 600 necessary for Adafruit IO
  config.jpeg_quality = 30;
  config.fb_count = 1;

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK)
  {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }
  else {
    Serial.printf("Camera init success");
  }
}

void controlServo(boolean isOpen) {
  if (isOpen) {
    servo.write(0);
    Serial.println("Open door!");
  } else {
    servo.write(90);
  }
}

boolean detectedObject() {
  long duration;
  float distanceCm;

  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);
  
  // Calculate the distance
  distanceCm = duration * SOUND_SPEED/2;

  if (distanceCm < LIMIT_DISTANCE) {
    Serial.println("Detected object!!!");
    return true;
  } 
  return false;

}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}