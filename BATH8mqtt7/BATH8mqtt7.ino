/*********
  SG-This works with MQTT but need to get the payload to change the 
  currentButtonState to HIGH to mimic the button
   
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp32-cam-video-streaming-web-server-camera-home-assistant/
  
  IMPORTANT!!! 
   - Select Board "AI Thinker ESP32-CAM"
   - GPIO 0 must be connected to GND to upload a sketch
   - After connecting GPIO 0 to GND, press the ESP32-CAM on-board RESET button to put your board in flashing mode
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*********/

#include "esp_camera.h"
#include <WiFi.h>
#include "esp_timer.h"
#include "img_converters.h"
#include "Arduino.h"
#include "fb_gfx.h"
#include "soc/soc.h" //disable brownout problems
#include "soc/rtc_cntl_reg.h"  //disable brownout problems
#include "esp_http_server.h"
//#include <Ethernet.h>
#include <PubSubClient.h>
#include <PubSubClientTools.h>

#include <Thread.h>
#include <ThreadController.h>

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
    String messageTemp;
  for (int i=0;i<length;i++) {
    Serial.print((char)payload[i]);
    messageTemp += (char)payload[i];
  }
  Serial.println();
  if (String(topic) == "bath/data/test") {
    Serial.print("Changing output to ");
    if(messageTemp == "buttonps"){
      Serial.println("on");
//      digitalWrite(ledPin, HIGH);
    }
    else if(messageTemp == "off"){
      Serial.println("off");
//      digitalWrite(ledPin, LOW);
    }
  }
}

//Replace with your network credentials
//const char* ssid = "";
//const char* password = "";
#define WIFI_SSID ""
#define WIFI_PASS ""
#define MQTT_SERVER ""
const char* user = "";
const char* pass = "";

WiFiClient espClient;
PubSubClient client(MQTT_SERVER, 1883, espClient);
PubSubClientTools mqtt(client);

ThreadController threadControl = ThreadController();
Thread thread = Thread();

int value = 0;
const String s = "";

// Set web server port number to 80
//WiFiServer server(80);
#define PART_BOUNDARY "123456789000000000000987654321"

// This project was tested with the AI Thinker Model, M5STACK PSRAM Model and M5STACK WITHOUT PSRAM-I removed!
#define CAMERA_MODEL_AI_THINKER

// Not tested with this model
//#define CAMERA_MODEL_WROVER_KIT

#if defined(CAMERA_MODEL_WROVER_KIT)
  #define PWDN_GPIO_NUM    -1
  #define RESET_GPIO_NUM   -1
  #define XCLK_GPIO_NUM    21
  #define SIOD_GPIO_NUM    26
  #define SIOC_GPIO_NUM    27
  
  #define Y9_GPIO_NUM      35
  #define Y8_GPIO_NUM      34
  #define Y7_GPIO_NUM      39
  #define Y6_GPIO_NUM      36
  #define Y5_GPIO_NUM      19
  #define Y4_GPIO_NUM      18
  #define Y3_GPIO_NUM       5
  #define Y2_GPIO_NUM       4
  #define VSYNC_GPIO_NUM   25
  #define HREF_GPIO_NUM    23
  #define PCLK_GPIO_NUM    22

#elif defined(CAMERA_MODEL_AI_THINKER)
  #define PWDN_GPIO_NUM     32
  #define RESET_GPIO_NUM    -1
  #define XCLK_GPIO_NUM      0
  #define SIOD_GPIO_NUM     26
  #define SIOC_GPIO_NUM     27
  
  #define Y9_GPIO_NUM       35
  #define Y8_GPIO_NUM       34
  #define Y7_GPIO_NUM       39
  #define Y6_GPIO_NUM       36
  #define Y5_GPIO_NUM       21
  #define Y4_GPIO_NUM       19
  #define Y3_GPIO_NUM       18
  #define Y2_GPIO_NUM        5
  #define VSYNC_GPIO_NUM    25
  #define HREF_GPIO_NUM     23
  #define PCLK_GPIO_NUM     22
#else
  #error "Camera model not selected"
#endif
// constants won't change
const int BUTTON_PIN = 12; // Arduino pin connected to button's pin
const int RELAY_PIN  = 2; // Arduino pin connected to relay's pin
#define WATER_PIN 14
// variables will change:
int waterState = LOW;
int relayState = LOW;   // the current state of relay
int lastButtonState;    // the previous state of button
int currentButtonState; // the current state of button
int extbut;
//int outputState = LOW; // variable for what we want to output (already defined in relaystate above)

unsigned long timeOn = 0; // what time did we turn the output on?
const unsigned long timerSetting = 10000; // this is in milliseconds
//       note: 1000 milliseconds = 1 second
// therefore, 60000 milliseconds = 1 minute
static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

httpd_handle_t stream_httpd = NULL;

static esp_err_t stream_handler(httpd_req_t *req){
  camera_fb_t * fb = NULL;
  esp_err_t res = ESP_OK;
  size_t _jpg_buf_len = 0;
  uint8_t * _jpg_buf = NULL;
  char * part_buf[64];

  res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
  if(res != ESP_OK){
    return res;
  }

  while(true){
    fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Camera capture failed");
      res = ESP_FAIL;
    } else {
      if(fb->width > 400){
        if(fb->format != PIXFORMAT_JPEG){
          bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
          esp_camera_fb_return(fb);
          fb = NULL;
          if(!jpeg_converted){
            Serial.println("JPEG compression failed");
            res = ESP_FAIL;
          }
        } else {
          _jpg_buf_len = fb->len;
          _jpg_buf = fb->buf;
        }
      }
    }
    if(res == ESP_OK){
      size_t hlen = snprintf((char *)part_buf, 64, _STREAM_PART, _jpg_buf_len);
      res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
    }
    if(res == ESP_OK){
      res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
    }
    if(res == ESP_OK){
      res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
    }
    if(fb){
      esp_camera_fb_return(fb);
      fb = NULL;
      _jpg_buf = NULL;
    } else if(_jpg_buf){
      free(_jpg_buf);
      _jpg_buf = NULL;
    }
    if(res != ESP_OK){
      break;
    }
    //Serial.printf("MJPG: %uB\n",(uint32_t)(_jpg_buf_len));
  }
  return res;
}

void startCameraServer(){
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;

  httpd_uri_t index_uri = {
    .uri       = "/",
    .method    = HTTP_GET,
    .handler   = stream_handler,
    .user_ctx  = NULL
  };
  
  //Serial.printf("Starting web server on port: '%d'\n", config.server_port);
  if (httpd_start(&stream_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(stream_httpd, &index_uri);
  }
}

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector
 
  Serial.begin(115200);
  Serial.setDebugOutput(false);
  
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
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG; 
  
  if(psramFound()){
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }
  
  // Camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }
  // Wi-Fi connection
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
 // Serial.println("Camera Stream Ready! Go to: http://");
  Serial.println(WiFi.localIP());
  delay(100);
  // Start streaming web server
  startCameraServer();
  {   Serial.begin(115200);         // initialize serial
    //while (!Serial);            //wait for serial connection.          
  pinMode(BUTTON_PIN, INPUT_PULLUP); // set arduino pin to input pull-up mode
  pinMode(RELAY_PIN, OUTPUT);        // set arduino pin to output mode
  pinMode(4, OUTPUT);

  currentButtonState = (LOW);
  relayState = (LOW);
  digitalWrite (RELAY_PIN, LOW);
  currentButtonState = digitalRead(BUTTON_PIN);
}
  // Connect to MQTT
  Serial.print(s+"Connecting to MQTT: "+MQTT_SERVER+" ... ");
  if (client.connect("ESP32Client")) {
    Serial.println("connected");
 //   delay(100);
    mqtt.subscribe("bath/data/test",  topic1_subscriber);

    mqtt.publish("bath/data/test", "STARTED");
  } else {
    Serial.println(s+"failed, rc="+client.state());
  }
  // Enable Thread
  thread.onRun(publisher);
  thread.setInterval(2000);
  threadControl.add(&thread);
}

void loop() {
  client.loop();
  threadControl.run();
  {
  lastButtonState    = currentButtonState;      // save the last state
  currentButtonState = digitalRead(BUTTON_PIN); // read new state
  //delay(100); // debounce NEW
  waterState = digitalRead(WATER_PIN); //Read data from WATER_PIN
   
  if (waterState == HIGH){ Serial.println("FULL"); 
                 digitalWrite(RELAY_PIN, LOW);
                 relayState = HIGH;
                } 
 //   else if (extbut == HIGH) {currentButtonState = HIGH; extbut = LOW;}
    else
  if(lastButtonState == LOW && currentButtonState == HIGH |(extbut == HIGH)) {
    Serial.println("The button is pressed");

    if (relayState == LOW) { //if relay is off..
    digitalWrite(RELAY_PIN, HIGH);  //turn it on..
    digitalWrite(4, HIGH);
    relayState = HIGH; // remember we turned it on..
    timeOn = millis(); //and remember what time we turned it on..
    }
    else {            //otherwise..
      digitalWrite(RELAY_PIN, LOW); //turn it off.
      digitalWrite(4, LOW);
      relayState = LOW;
    }
  }
  else { // If no button press:
    // If the LED is on and has been on for such-and-such a length of time,
    if ((relayState == HIGH) && ((millis() - timeOn) > timerSetting)) {
      digitalWrite(RELAY_PIN, LOW);
      digitalWrite(4, LOW);
      //Serial.println("Bath ran");// we turn the LED off, ...
      relayState = LOW; // and remember that we turned it off.  
      timeOn = millis();   // remember when we turned it off
    }
  }
 }
 delay(100);
}

void publisher() {
  ++value;
  mqtt.publish("bath/hello_world", s+"Hello World! - No. "+value);
}
void topic1_subscriber(String topic, String message) {
    if(message == "buttonps"){
      extbut = HIGH;
      Serial.println("extbut pressed");

    }
    else {
    Serial.println(s+"Message arrived in function 1 ["+topic+"] "+message);
}
}
