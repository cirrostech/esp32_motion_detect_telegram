#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiServer.h>
#include <WiFiUdp.h>

#include <WiFiClientSecure.h>
#include <UniversalTelegramBot.h>
#include <ArduinoJson.h>

#define CAMERA_MODEL_AI_THINKER 

#include <FS.h>
#include <SPIFFS.h>

#include <EloquentArduino.h>
#include <eloquentarduino/io/serial_print.h>
#include <eloquentarduino/vision/camera/ESP32Camera.h>
#include <eloquentarduino/vision/io/writers/JpegWriter.h>
#include <eloquentarduino/vision/io/decoders/Red565RandomAccessDecoder.h>
#include <eloquentarduino/vision/processing/downscaling/Downscaler.h>
#include <eloquentarduino/vision/processing/MotionDetector.h>

#include <Arduino.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

#define SD_CS 5
#define FRAME_SIZE FRAMESIZE_QVGA
#define PIXFORMAT PIXFORMAT_RGB565
#define W 320
#define H 240
#define w 32
#define h 24
#define DIFF_THRESHOLD 40
#define MOTION_THRESHOLD 40

WiFiClientSecure client;

const unsigned long BOT_MTBS = 1000; // mean time between scan messages
unsigned long bot_lasttime;  

#include <EEPROM.h> 
#define EEPROM_SIZE 1
int pictureNumber = 0; 
 
#define timeit(label, code) { uint32_t start = millis(); code; uint32_t duration = millis() - start; eloquent::io::print_all("It took ", duration, " millis for ", label); }
#define timeit(label, code) code;

using namespace Eloquent::Vision;

camera_fb_t *frame;
Camera::ESP32Camera camera(PIXFORMAT);
uint8_t downscaled[w * h];
//IO::Decoders::GrayscaleRandomAccessDecoder decoder;
IO::Decoders::Red565RandomAccessDecoder decoder;
Processing::Downscaling::Center<W / w, H / h> strategy;
Processing::Downscaling::Downscaler<W, H, w, h> downscaler(&decoder, &strategy);
Processing::MotionDetector<w, h> motion;
IO::Writers::JpegWriter<W, H> jpegWriter;

const char* ssid = "AccessPoint";
const char* password = "APCredentials";

// Initialize Telegram BOT
String BOTtoken = "BOT:TOKEN";  // your Bot Token (Get from Botfather)

// Use @myidbot to find out the chat ID of an individual or a group
// Also note that you need to click "start" on a bot before it can
// message you

String CHAT_ID = "CHAT_ID";
bool flashState = LOW;

void capture();
void save();
void stream_downscaled();
void stream();

UniversalTelegramBot bot(BOTtoken, client);

File myFile;
bool isMoreDataAvailable();
byte getNextByte();

bool isMoreDataAvailable()
{
  return myFile.available();
}

byte getNextByte()
{
  return myFile.read();
}

void setup() {
    Serial.begin(115200);
    
    if(!SPIFFS.begin(true)){
      Serial.println("An Error has occurred while mounting SPIFFS");
      return;
    }
    delay(1000);
    Serial.print("Connecting Wifi: ");
    Serial.println(ssid);

    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);

    client.setCACert(TELEGRAM_CERTIFICATE_ROOT);  
    while (WiFi.status() != WL_CONNECTED) {
      Serial.print(".");
      delay(500);
    }

    Serial.println("WiFi connected");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    
    if(bot.sendMessage(CHAT_ID, "Wifi Connected", "Markdown")){
      Serial.println("TELEGRAM Successfully sent");
    } else {
      Serial.println("TELEGRAM Not sent");
    }
    
    camera.begin(FRAME_SIZE);
    // set how much a pixel value should differ to be considered as a change
    motion.setDiffThreshold(DIFF_THRESHOLD);
    // set how many pixels (in percent) should change to be considered as motion
    motion.setMotionThreshold(MOTION_THRESHOLD);
    // prevent consecutive triggers
    motion.setDebounceFrames(5);
    
}


void loop() {
    capture();
    //eloquent::io::print_all(motion.changes(), " pixels changed");

    if (motion.triggered()) {
    
        Serial.println("Motion detected");
        bot.sendMessage(CHAT_ID, "Motion detected", "");
        save();
        
        // uncomment to stream to the Python script for visualization
        // stream();
  
        // uncomment to stream downscaled imaged tp Python script
        // stream_downscaled();

        delay(300);
    }

    if (millis() - bot_lasttime > BOT_MTBS)
    {
    int numNewMessages = bot.getUpdates(bot.last_message_received + 1);

    while (numNewMessages)
    {
      handleNewMessages(numNewMessages);
      numNewMessages = bot.getUpdates(bot.last_message_received + 1);
    }

      bot_lasttime = millis();
    }
    delay(30);
}

void handleNewMessages(int numNewMessages)
{
  Serial.println(numNewMessages);
}

void capture() {
    timeit("capture frame", frame = camera.capture());

    // scale image from size H * W to size h * w
    timeit("downscale", downscaler.downscale(frame->buf, downscaled));

    // detect motion on the downscaled image
    timeit("motion detection", motion.detect(downscaled));
}

void save() {
    
    File imageFile = SPIFFS.open("/capture.jpg", "wb");
    
    if(!imageFile){
      Serial.println("There was an error opening the file for writing");
      return;
    }else{
      Serial.println("File open");
    }
    uint8_t quality = 30;

    eloquent::io::print_all("The image will be saved as /capture.jpg");
    jpegWriter.write(imageFile, frame->buf, PIXFORMAT, quality);
    imageFile.close();
    eloquent::io::print_all("Saved");

    myFile = SPIFFS.open("/capture.jpg", "r");
    if (myFile)
    {
      Serial.println("File opened");
      String sent = bot.sendPhotoByBinary(CHAT_ID, "image/jpeg", myFile.size(),
                                        isMoreDataAvailable,
                                        getNextByte, nullptr, nullptr);                                       
      myFile.close();
    }
    else
    {
      Serial.println("Error opening image");
    }  
        
}


void stream() {
    eloquent::io::print_all("START OF FRAME");

    jpegWriter.write(Serial, frame->buf, PIXFORMAT, 30);

    eloquent::io::print_all("END OF FRAME");
}


void stream_downscaled() {
    eloquent::io::print_all("START OF DOWNSCALED");
    eloquent::io::print_array(downscaled, w * h);
    eloquent::io::print_all("END OF DOWNSCALED");
}
