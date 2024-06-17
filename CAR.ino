/*******************************************************
 * BLEV7RC_CAR_VIDEO.ino - BLE Remote Control Car Arduino Code
 *
 * Author: Kevin Chen
 * Date: 2023/8/15
 *
 * Version: 1.0.0
 *
 * This code was written by Kevin Chen.
 *
 * This is an open-source program, and it can be freely used, modified, and distributed under the following conditions:
 *
 * 1. The original copyright notice must not be removed or modified.
 * 2. Projects or products using this code must acknowledge the original author's name and the source in applicable documentation, websites, or other related materials.
 * 3. Any derivative work based on this code must state its origin and retain the original copyright notice in its documentation.
 * 4. This code must not be used for any activities that may infringe upon the rights of others, be unlawful, or harmful, whether in a commercial or non-commercial environment.
 *
 * This code is provided "as is" with no warranty, expressed or implied. The author is not liable for any losses or damages caused by using the code.
 *
 * Users of BLEV7RC_CAR_VIDEO.ino assume all risks associated with its use, and the author shall not be held responsible for any consequences.
 *
 * For more information about our company, please visit: www.makdev.net
 *
 * Example guide:
 * https://www.amebaiot.com/en/amebapro2-arduino-ble-v7rc/
 *******************************************************/



#define UART_SERVICE_UUID      "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

#define STRING_BUF_SIZE 100
#define MaxNumValue     2

#define value1 0
#define value2 1
/*
#define MotoA_1A 16    // GPIO
#define MotoA_1B 7     // PWM
#define MotoB_1A 17    // GPIO
#define MotoB_1B 8     // PWM
*/


// TB6612 pin connection

// VM  connected to +5V 
// Vcc connected to +3.3V or +5V
// Gnd connected to Ground
#define PWMA  3
#define AIN2  9
#define AIN1  10
#define STBY  13
#define BIN1  11
#define BIN2  14
#define PWMB  4
#define LED   7

#define FULLSPEED 255
#define HALFSPEED 128
#define STOPSPEED 0



#define CHANNEL 1

// Default preset configurations for each video channel:
// Channel 0 : 1920 x 1080 30FPS H264
// Channel 1 : 1280 x 720  30FPS H264
// Channel 2 : 1920 x 1080 30FPS MJPEG

/*
    This example uses the on-board camera sensor (JX-F37P) to capture suspicious
   movements. Upon detection, the system captures an image, saves it to an SD
   Card, uploads it to Google Drive, and concurrently sends an alert through
   Line Notify to the user's mobile phone, ensuring swift response and
   heightened security.

    Example guide: https://www.amebaiot.com/en/amebapro2-arduino-motion-notify/
*/

#include <Arduino.h>
#include "BLEDevice.h"
#include "WiFi.h"
#include "VideoStream.h"
#include "StreamIO.h"
#include "RTSP.h"
#include "MotionDetection.h"
#include "VideoStreamOverlay.h"
#include "AmebaFatFS.h"
#include "Base64.h"
#include "NNObjectDetection.h"
#include "ObjectClassList.h"
#include "NNFaceDetectionRecognition.h"
#include <AmebaServo.h>
#include "AudioStream.h"
#include "AudioEncoder.h"
#include "MP4Recording.h"
#include "Base32.h"
#include "ArduinoJson.h"



#define FULLSPEED 255
#define HALFSPEED 128
#define STOPSPEED 0

#define CHANNELVID  0    // Channel for RTSP streaming
#define CHANNELJPEG 1    // Channel for taking snapshots
#define CHANNELNN   3    // RGB format video for NN only available on channel 3

// Customised resolution for NN
#define NNWIDTH  576
#define NNHEIGHT 320

// Pin Definition
#define RED_LED                3
#define GREEN_LED              4
#define BACKUP_FACE_BUTTON_PIN 5
#define EN_REGMODE_BUTTON_PIN  6
#define SERVO_PIN              8


// User Configuration
             // Video channel for streaming & snapshot
#define CHANNELMD 3              // RGB format video for motion detection only available on channel 3
#define FILENAME  "image.jpg"    // Save as jpg image in SD Card

VideoSetting config1(VIDEO_D1, CAM_FPS, VIDEO_H264_JPEG, 1);    // High resolution video for streaming
VideoSetting configMD(VIDEO_VGA, 10, VIDEO_RGB, 0);            // Low resolution RGB video for motion detection
RTSP rtsp;
StreamIO videoStreamer1(1, 1);
StreamIO videoStreamerMD(1, 1);
MotionDetection MD;
AmebaFatFS fs;
WiFiSSLClient wifiClient;

char buf[512];
char *p;
bool flag_motion = false;
bool doorOpen = false;
bool backupButtonState = false;
bool RegModeButtonState = false;
bool regMode = false;
uint32_t img_addr = 0;
uint32_t img_len = 0;
String fileName;
long counter = 0;

#define FILENAME "TestRecordingAudioOnly.mp4"

char server[] = "123.195.32.57";    // your server IP running HTTP server on PC
#define PORT 5000

WiFiClient wifiClient;



String filepath;
File file;


AudioSetting configA(0);
Audio audio;
AAC aac;
MP4Recording mp4;
StreamIO audioStreamer1(1, 1);    // 1 Input Audio -> 1 Output AAC
StreamIO audioStreamer2(1, 1);    // 1 Input AAC -> 1 Output MP4

const int buttonPin = 1;          // the number of the pushbutton pin
int buttonState;                          // variable for reading the pushbutton status
unsigned long buttonPressTime = 0;        // variable to store the time when button was pressed
bool buttonPressedFor2Seconds = false;    // flag to indicate if button is pressed for at least 2 seconds
int recordingstate = -1;
int previousRecordingState = -1;



VideoSetting configVID(VIDEO_FHD, 30, VIDEO_H264, 0);
VideoSetting configJPEG(VIDEO_FHD, CAM_FPS, VIDEO_JPEG, 1);
VideoSetting configNN(NNWIDTH, NNHEIGHT, 10, VIDEO_RGB, 0);
NNFaceDetectionRecognition facerecog;
RTSP rtsp;
StreamIO videoStreamer(1, 1);
StreamIO videoStreamerFDFR(1, 1);
StreamIO videoStreamerRGBFD(1, 1);
AmebaServo myservo;



// Enter your Google Script and Line Notify details
String myScript = "/macros/s/AKfycbw3TxERNlHtxdXkooGhrclHPbDmgstugKszK68AsdCCDrkdLbFR4Jh5Cltb8CoPV1PL/exec";    // Create your Google Apps Script and replace the "myScript" path.
String myFoldername = "&myFoldername=AMB82";                                    // Set the Google Drive folder name to store your file
String myFilename = "&myFilename=image.jpg";                                    // Set the Google Drive file name to store your data
String myImage = "&myFile=";

// Create objects

// VideoSetting config(CHANNEL);
VideoSetting config(VIDEO_D1, CAM_FPS, VIDEO_H264, 1);
RTSP rtsp1;
//RTSP rtsp2;
StreamIO videoStreamer(1, 2);    // 1 Input Video -> 1 Output RTSP


#define CHANNEL1 0
#define CHANNELNN 3 

// Lower resolution for NN processing
#define NNWIDTH  576
#define NNHEIGHT 320

VideoSetting config2(VIDEO_FHD, 30, VIDEO_H264, 0);
VideoSetting configNN(NNWIDTH, NNHEIGHT, 10, VIDEO_RGB, 0);
NNObjectDetection ObjDet;

IPAddress ip;
int rtsp_portnum;


char ssid[] = "Moto Z2 Play 7440";    // your network SSID (name)
char pass[] = "556e5e9024ef";        // your network password
int status = WL_IDLE_STATUS;

typedef struct {
    bool reciveCMDFlag;
    int ReciveValue;
} _rCMD;

BLEService UartService(UART_SERVICE_UUID);
BLECharacteristic Rx(CHARACTERISTIC_UUID_RX);
BLECharacteristic Tx(CHARACTERISTIC_UUID_TX);
BLEAdvertData advdata;
BLEAdvertData scndata;
bool notify = false;
uint8_t Count;

String CMDRefer[5] = {"SS2", "SS4", "SRT", "SR2", "SRV"};
_rCMD bleReciveData[MaxNumValue];

void backward()
{
  /*
    digitalWrite(MotoA_1A, 1);
    analogWrite(MotoA_1B, 5);

    digitalWrite(MotoB_1A, 1);
    analogWrite(MotoB_1B, 5);
*/
  
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);

    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    delay(50);
}

void forward()
{
  /*
    digitalWrite(MotoA_1A, 0);
    analogWrite(MotoA_1B, 250);

    digitalWrite(MotoB_1A, 0);
    analogWrite(MotoB_1B, 250);
*/
    
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);

    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    delay(50);
}

void turnRight()
{
  /*
    digitalWrite(MotoA_1A, 1);
    analogWrite(MotoA_1B, 5);

    digitalWrite(MotoB_1A, 0);
    analogWrite(MotoB_1B, 250);
  */

    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);

    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    delay(50);
}

void turnLeft()
{
  /*
    digitalWrite(MotoA_1A, 0);
    analogWrite(MotoA_1B, 250);

    digitalWrite(MotoB_1A, 1);
    analogWrite(MotoB_1B, 5);
  */
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);

    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    delay(50);
}

void BrakeAll()
{
  /*
    digitalWrite(MotoA_1A, 0);
    analogWrite(MotoA_1B, 0);

    digitalWrite(MotoB_1A, 0);
    analogWrite(MotoB_1B, 0);
*/
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);

    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
    delay(50);
}

void readCB(BLECharacteristic* chr, uint8_t connID)
{
    printf("Characteristic %s read by connection %d \n", chr->getUUID().str(), connID);
}

void writeCB(BLECharacteristic* chr, uint8_t connID)
{
    // printf("Characteristic %s write by connection %d :\n", chr->getUUID().str(), connID);
    if (chr->getDataLen() > 0) {
        ParseCMDString(chr->readString());
        // Serial.print("Received string: ");
        // Serial.print(chr->readString());
        // Serial.println();
    }
}

void notifCB(BLECharacteristic* chr, uint8_t connID, uint16_t cccd)
{
    if (cccd & GATT_CLIENT_CHAR_CONFIG_NOTIFY) {
        printf("Notifications enabled on Characteristic %s for connection %d \n", chr->getUUID().str(), connID);
        notify = true;
    } else {
        printf("Notifications disabled on Characteristic %s for connection %d \n", chr->getUUID().str(), connID);
        notify = false;
    }
}

void ParseCMDString(String cmd)
{
    int comdLength = cmd.length();
    int chkx;
    int CMDMaxNUM = sizeof(CMDRefer) / sizeof(String);

    for (chkx = 0; chkx < CMDMaxNUM; chkx++) {
        if (cmd.indexOf(CMDRefer[chkx].c_str()) > -1) {
            break;
        }
    }

    if (chkx >= CMDMaxNUM && cmd.charAt(comdLength - 1) != '#') {
        return;
    }

    if (cmd.indexOf("SRT") > -1) {
        int x = 3;
        int ValueIndex = 0;

        while (x < (comdLength - 1)) {
            if ((x + 3) < comdLength) {
                String _NumString = cmd.substring(x, (x + 4));
                // Serial.println(_NumString);
                if (ValueIndex < MaxNumValue) {
                    if (bleReciveData[ValueIndex].ReciveValue != _NumString.toInt()) {
                        bleReciveData[ValueIndex].ReciveValue = _NumString.toInt();
                        bleReciveData[ValueIndex].reciveCMDFlag = true;
                    }
                }
            }
            ValueIndex++;
            x += 4;
        }
    }
}

void printInfo(void)
{
    Serial.println("------------------------------");
    Serial.println("- Summary of Streaming -");
    Serial.println("------------------------------");
    Camera.printInfo();

    IPAddress ip = WiFi.localIP();

    Serial.println("- RTSP -");
    Serial.print("rtsp://");
    Serial.print(ip);
    Serial.print(":");
    rtsp1.printInfo();

    Serial.print("rtsp://");
    Serial.print(ip);
    Serial.print(":");
    //rtsp2.printInfo();
}

void setup()
{
    Serial.begin(115200);

    advdata.addFlags(GAP_ADTYPE_FLAGS_LIMITED | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED);
    advdata.addCompleteName("AMB82-TANK");
    scndata.addCompleteServices(BLEUUID(UART_SERVICE_UUID));

    Rx.setWriteNRProperty(true);
    Rx.setWritePermissions(GATT_PERM_WRITE);
    Rx.setWriteCallback(writeCB);
    Rx.setBufferLen(STRING_BUF_SIZE);

    Tx.setReadProperty(true);
    Tx.setReadPermissions(GATT_PERM_READ);
    Tx.setReadCallback(readCB);
    Tx.setNotifyProperty(true);
    Tx.setCCCDCallback(notifCB);
    Tx.setBufferLen(STRING_BUF_SIZE);

    UartService.addCharacteristic(Rx);
    UartService.addCharacteristic(Tx);

    BLE.init();
    BLE.configAdvert()->setAdvData(advdata);
    BLE.configAdvert()->setScanRspData(scndata);
    BLE.configServer(1);
    BLE.addService(UartService);

    BLE.beginPeripheral();

    // attempt to connect to Wifi network:
    while (status != WL_CONNECTED) {
        Serial.print("Attempting to connect to WPA SSID: ");
        Serial.println(ssid);
        status = WiFi.begin(ssid, pass);

        // wait 2 seconds for connection:
        delay(2000);
    }

    // Configure camera video channel with video format information
    Camera.configVideoChannel(CHANNEL, config1);
    Camera.videoInit();

    // Configure RTSP with identical video format information
    rtsp1.configVideo(config);
    rtsp1.begin();
    //rtsp2.configVideo(config);
    //rtsp2.begin();

    // Configure StreamIO object to stream data from video channel to RTSP
    videoStreamer1.registerInput(Camera.getStream(CHANNEL));
    videoStreamer1.registerOutput1(rtsp1);
    //videoStreamer.registerOutput2(rtsp2);
    if (videoStreamer1.begin() != 0) {
        Serial.println("StreamIO link start failed");
    }

    // Start data stream from video channel
    Camera.channelBegin(CHANNEL);

    delay(1000);
    printInfo();
/*
    pinMode(MotoA_1A, OUTPUT);
    pinMode(MotoA_1B, OUTPUT);
    pinMode(MotoB_1A, OUTPUT);
    pinMode(MotoB_1B, OUTPUT);

    digitalWrite(MotoA_1A, 0);
    digitalWrite(MotoA_1B, 0);
    digitalWrite(MotoB_1A, 0);
    digitalWrite(MotoB_1B, 0);
*/
    
    pinMode(PWMA, OUTPUT);  
    pinMode(AIN2, OUTPUT);
    pinMode(AIN1, OUTPUT);
    pinMode(STBY, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);
    pinMode(PWMB, OUTPUT);
    // set default value
    digitalWrite(AIN2, LOW);
    digitalWrite(AIN1, LOW);
    digitalWrite(STBY, HIGH); // LOW = standby
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
    
    Serial.println("Set PWM value for Motor Speed !");
    //set Motor Speed
    analogWrite(PWMA, 128);
    analogWrite(PWMB, 128);


    Serial.begin(115200);

    // attempt to connect to Wifi network:
    while (status != WL_CONNECTED) {
        Serial.print("Attempting to connect to WPA SSID: ");
        Serial.println(ssid);
        status = WiFi.begin(ssid, pass);
        delay(2000);
    }
    WiFiCon();

    // Configure camera video channels for required resolutions and format
    // outputs Adjust the bitrate based on your WiFi network quality
    // config.setBitrate(2 * 1024 * 1024);
    // Recommend to use 2Mbps for RTSP streaming to prevent network congestion
    Camera.configVideoChannel(CHANNEL, config1);
    Camera.configVideoChannel(CHANNELMD, configMD);
    Camera.videoInit();

    // Configure RTSP for high resolution video stream information
    rtsp.configVideo(config1);
    rtsp.begin();

    // Configure motion detection for low resolution RGB video stream
    MD.configVideo(configMD);
    MD.setResultCallback(mdPostProcess);
    MD.begin();

    // Configure StreamIO object to stream data from high res video channel to RTSP
    videoStreamer.registerInput(Camera.getStream(CHANNEL));
    videoStreamer.registerOutput(rtsp);
    if (videoStreamer.begin() != 0) {
        Serial.println("StreamIO link start failed");
    }

    // Start data stream from high resolution video channel
    Camera.channelBegin(CHANNEL);
    Serial.println("Video RTSP Streaming Start");

    // Configure StreamIO object to stream data from low res video channel to motion detection
    videoStreamerMD.registerInput(Camera.getStream(CHANNELMD));
    videoStreamerMD.setStackSize();
    videoStreamerMD.registerOutput(MD);
    if (videoStreamerMD.begin() != 0) {
        Serial.println("StreamIO link start failed");
    }

    // Start data stream from low resolution video channel
    Camera.channelBegin(CHANNELMD);

    // Configure OSD for drawing on high resolution video stream
    OSD.configVideo(CHANNEL, config1);
    OSD.begin();
    Serial.println("");
    Serial.println("================================");
    Serial.println("Motion Detecting");
    Serial.println("================================");
    delay(2000);

    Serial.begin(115200);

    // attempt to connect to Wifi network:
    while (status != WL_CONNECTED) {
        Serial.print("Attempting to connect to WPA SSID: ");
        Serial.println(ssid);
        status = WiFi.begin(ssid, pass);

        // wait 2 seconds for connection:
        delay(2000);
    }
    ip = WiFi.localIP();

    // Configure camera video channels with video format information
    // Adjust the bitrate based on your WiFi network quality
    config2.setBitrate(2 * 1024 * 1024);     // Recommend to use 2Mbps for RTSP streaming to prevent network congestion
    Camera.configVideoChannel(CHANNEL1, config2);
    Camera.configVideoChannel(CHANNELNN, configNN);
    Camera.videoInit();

    // Configure RTSP with corresponding video format information
    rtsp.configVideo(config2);
    rtsp.begin();
    rtsp_portnum = rtsp.getPort();

    // Configure object detection with corresponding video format information
    // Select Neural Network(NN) task and models
    ObjDet.configVideo(configNN);
    ObjDet.modelSelect(OBJECT_DETECTION, DEFAULT_YOLOV7TINY, NA_MODEL, NA_MODEL);
    ObjDet.begin();

    // Configure StreamIO object to stream data from video channel to RTSP
    videoStreamer.registerInput(Camera.getStream(CHANNEL1));
    videoStreamer.registerOutput(rtsp);
    if (videoStreamer.begin() != 0) {
        Serial.println("StreamIO link start failed");
    }

    // Start data stream from video channel
    Camera.channelBegin(CHANNEL1);

    // Configure StreamIO object to stream data from RGB video channel to object detection
    videoStreamerNN.registerInput(Camera.getStream(CHANNELNN));
    videoStreamerNN.setStackSize();
    videoStreamerNN.setTaskPriority();
    videoStreamerNN.registerOutput(ObjDet);
    if (videoStreamerNN.begin() != 0) {
        Serial.println("StreamIO link start failed");
    }

    // Start video channel for NN
    Camera.channelBegin(CHANNELNN);

    // Start OSD drawing on RTSP video channel
    OSD.configVideo(CHANNEL1, config2);
    OSD.begin();

    Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);  

  pinMode(LED, OUTPUT); 

  Serial.println("TB6612FNG Pin setup...");

  pinMode(PWMA, OUTPUT);  
  pinMode(AIN2, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(STBY, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  // set default value
  digitalWrite(AIN2, LOW);
  digitalWrite(AIN1, LOW);
  digitalWrite(STBY, HIGH); // LOW = standby
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);

  digitalWrite(LED_BUILTIN, HIGH);
  Serial.println("TB6612FNG Test begin !!!");

  Serial.println("Set PWM value for Motor Speed !");
  //set Motor Speed
  analogWrite(PWMA, 250);
  analogWrite(PWMB, 250);

  Serial.println("Forward...");

  forward();
  delay(1000);
  stop();
  delay(1000);

  Serial.println("Backward...");
  backward();
  delay(1000);
  stop();
  delay(1000);

  Serial.println("Turn-Left...");   
  turnleft();
  delay(1000);
  stop();
  delay(1000);

  Serial.println("Turn-Right...");  
  turnright();
  delay(1000);
  stop();  
  delay(1000);
  Serial.println("Test finished !!!");  
  digitalWrite(LED_BUILTIN, LOW);  

    // GPIO Initialisation
    pinMode(RED_LED, OUTPUT);
    pinMode(GREEN_LED, OUTPUT);
    pinMode(BACKUP_FACE_BUTTON_PIN, INPUT);
    pinMode(EN_REGMODE_BUTTON_PIN, INPUT);
    myservo.attach(SERVO_PIN);

    Serial.begin(115200);

    // Attempt to connect to Wifi network:
    while (status != WL_CONNECTED) {
        Serial.print("Attempting to connect to WPA SSID: ");
        Serial.println(ssid);
        status = WiFi.begin(ssid, pass);

        // wait 2 seconds for connection:
        delay(2000);
    }

    // Configure camera video channels with video format information
    Camera.configVideoChannel(CHANNELVID, configVID);
    Camera.configVideoChannel(CHANNELJPEG, configJPEG);
    Camera.configVideoChannel(CHANNELNN, configNN);
    Camera.videoInit();

    // Configure RTSP with corresponding video format information
    rtsp.configVideo(configVID);
    rtsp.begin();

    // Configure Face Recognition model
    facerecog.configVideo(configNN);
    facerecog.modelSelect(FACE_RECOGNITION, NA_MODEL, DEFAULT_SCRFD, DEFAULT_MOBILEFACENET);
    facerecog.begin();
    facerecog.setResultCallback(FRPostProcess);

    // Configure StreamIO object to stream data from video channel to RTSP
    videoStreamer.registerInput(Camera.getStream(CHANNELVID));
    videoStreamer.registerOutput(rtsp);
    if (videoStreamer.begin() != 0) {
        Serial.println("StreamIO link start failed");
    }

    // Start data stream from video channel
    Camera.channelBegin(CHANNELVID);
    Camera.channelBegin(CHANNELJPEG);

    // Configure StreamIO object to stream data from RGB video channel to face detection
    videoStreamerRGBFD.registerInput(Camera.getStream(CHANNELNN));
    videoStreamerRGBFD.setStackSize();
    videoStreamerRGBFD.setTaskPriority();
    videoStreamerRGBFD.registerOutput(facerecog);
    if (videoStreamerRGBFD.begin() != 0) {
        Serial.println("StreamIO link start failed");
    }

    // Start video channel for NN
    Camera.channelBegin(CHANNELNN);

    // Start OSD drawing on RTSP video channel
    OSD.configVideo(CHANNELVID, configVID);
    OSD.begin();

    // Restore any registered faces saved in flash
    facerecog.restoreRegisteredFace();

    // Servo moves to position the angle 180 degree (LOCK CLOSE)
    myservo.write(180);

        Serial.begin(115200);

    // Connection to internet
    while (status != WL_CONNECTED) {
        Serial.print("Attempting to connect to WPA SSID: ");
        Serial.println(ssid);
        status = WiFi.begin(ssid, pass);
        delay(2000);
    }

    // list files under root directory
    fs.begin();

    // initialize the pushbutton pin as an input:
    pinMode(buttonPin, INPUT);
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(LED_G, OUTPUT);

    // Configure audio peripheral for audio data output
    audio.configAudio(configA);
    audio.begin();
    // Configure AAC audio encoder
    aac.configAudio(configA);
    aac.begin();

    // Configure MP4 recording settings
    mp4.configAudio(configA, CODEC_AAC);
    mp4.setRecordingDuration(5);
    mp4.setRecordingFileCount(1);
    mp4.setRecordingFileName("TestRecordingAudioOnly");
    mp4.setRecordingDataType(STORAGE_AUDIO);    // Set MP4 to record audio only

    // Configure StreamIO object to stream data from audio channel to AAC encoder
    audioStreamer1.registerInput(audio);
    audioStreamer1.registerOutput(aac);
    if (audioStreamer1.begin() != 0) {
        Serial.println("StreamIO link start failed");
    }

    // Configure StreamIO object to stream data from AAC encoder to MP4
    audioStreamer2.registerInput(aac);
    audioStreamer2.registerOutput(mp4);
    if (audioStreamer2.begin() != 0) {
        Serial.println("StreamIO link start failed");
    }

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);  

  pinMode(LED, OUTPUT); 

  Serial.println("TB6612FNG Pin setup...");

  pinMode(PWMA, OUTPUT);  
  pinMode(AIN2, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(STBY, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  // set default value
  digitalWrite(AIN2, LOW);
  digitalWrite(AIN1, LOW);
  digitalWrite(STBY, HIGH); // LOW = standby
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);

  digitalWrite(LED_BUILTIN, HIGH);
  Serial.println("TB6612FNG Test begin !!!");

  Serial.println("Set PWM value for Motor Speed !");
  //set Motor Speed
  analogWrite(PWMA, 250);
  analogWrite(PWMB, 250);

  Serial.println("Forward...");

  forward();
  delay(1000);
  stop();
  delay(1000);

  Serial.println("Backward...");
  backward();
  delay(1000);
  stop();
  delay(1000);

  Serial.println("Turn-Left...");   
  turnleft();
  delay(1000);
  stop();
  delay(1000);

  Serial.println("Turn-Right...");  
  turnright();
  delay(1000);
  stop();  
  delay(1000);
  Serial.println("Test finished !!!");  
  digitalWrite(LED_BUILTIN, LOW);  


}

void forward(){
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);

  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
}

void backward(){
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);

  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
}

void turnleft(){
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);

  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
}

void turnright(){
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);

  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
}

void stop(){
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);

  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
}



void loop()
{
    while (Count < MaxNumValue) {
        if (bleReciveData[Count].reciveCMDFlag) {
            bleReciveData[Count].reciveCMDFlag = false;

            if (abs(bleReciveData[value1].ReciveValue - 1500) < 100 && abs(bleReciveData[value2].ReciveValue - 1500) < 100) {
                BrakeAll();
            } else if (abs(bleReciveData[value1].ReciveValue - 1500) > abs(bleReciveData[value2].ReciveValue - 1500)) {
                if (bleReciveData[value1].ReciveValue > 1500) {
                    turnRight();
                } else {
                    turnLeft();
                }
            } else {
                if (bleReciveData[value2].ReciveValue > 1500) {
                    forward();
                } else {
                    backward();
                }
            }
        }
        Count++;
    }
    Count = 0;
    delay(1);

     if (flag_motion) {
        Serial.println("Motion Detected");
        // SD card init
        if (!fs.begin()) {
            StreamEnd();
            pinMode(LED_B, OUTPUT);
            digitalWrite(LED_B, HIGH);
            Serial.println("");
            Serial.println("================================");
            Serial.println("[ERROR] SD Card Mount Failed !!!");
            Serial.println("================================");
            while (1)
                ;
        }

        // List root directory and put results in buf
        memset(buf, 0, sizeof(buf));
        fs.readDir(fs.getRootPath(), buf, sizeof(buf));
        String filepath = String(fs.getRootPath()) + String(FILENAME);
        File file = fs.open(filepath);
        if (!file) {
            Serial.println("");
            Serial.println("================================");
            Serial.println("[ERROR] Failed to open file for reading");
            Serial.println("================================");
            fs.end();
        }
        // Serial.println("Files under: " + String(fs.getRootPath()));
        // Serial.println("Read from file: " + filepath);
        // Serial.println("file size: " + String(file.size()));
        delay(100);

        // Taking Photo
        CamFlash();
        Camera.getImage(CHANNEL, &img_addr, &img_len);
        file.write((uint8_t *)img_addr, img_len);
        file.close();

        Serial.println("===================================");
        Serial.println("[INFO] Photo Captured ...");
        Serial.println("===================================");

        // File Processing
        /* the filenames are separated with '\0', so we scan one by one */
        p = buf;
        while (strlen(p) > 0) {
            /* list out file name image will be saved as "image.jpg" */
            if (strstr(p, FILENAME) != NULL) {
                Serial.println("[INFO] Found 'image.jpg' in the string.");
                Serial.println("[INFO] Processing file...");
            } else {
                // Serial.println("Substring 'image.jpg' not found in the
                // string.");
            }
            p += strlen(p) + 1;
        }

        uint8_t *fileinput;
        file = fs.open(filepath);
        unsigned int fileSize = file.size();
        fileinput = (uint8_t *)malloc(fileSize + 1);
        file.read(fileinput, fileSize);
        fileinput[fileSize] = '\0';
        file.close();
        fs.end();

        char *input = (char *)fileinput;
        String imageFile = "data:image/jpeg;base32,";
        char output[base64_enc_len(3)];
        for (unsigned int i = 0; i < fileSize; i++) {
            base64_encode(output, (input++), 3);
            if (i % 3 == 0) {
                imageFile += urlencode(String(output));
            }
        }

        // transfer file to Google Drive
        // https://github.com/fustyles/Arduino/tree/master/ESP32-CAM_GoogleDrive_Linenotify
        Serial.println("[INFO] Uploading file to Google Drive...");
        String Data = myFoldername + myFilename + myImage;
        const char *myDomain = "script.google.com";
        String getAll = "", getBody = "";
        Serial.println("[INFO] Connect to " + String(myDomain));

        if (wifiClient.connect(myDomain, 443)) {
            Serial.println("[INFO] Connection successful");

            wifiClient.println("POST " + myScript + " HTTP/1.1");
            wifiClient.println("Host: " + String(myDomain));
            wifiClient.println("Content-Length: " + String(Data.length() + imageFile.length()));
            wifiClient.println("Content-Type: application/x-www-form-urlencoded");
            wifiClient.println("Connection: keep-alive");
            wifiClient.println();

            wifiClient.print(Data);
            for (unsigned int Index = 0; Index < imageFile.length(); Index = Index + 1000) {
                wifiClient.print(imageFile.substring(Index, Index + 1000));
            }

            int waitTime = 10000;    // timeout 10 seconds
            unsigned int startTime = millis();
            boolean state = false;

            while ((startTime + waitTime) > millis()) {
                // Serial.print(".");
                delay(100);
                while (wifiClient.available()) {
                    char c = wifiClient.read();
                    if (state == true) {
                        getBody += String(c);
                    }
                    if (c == '\n') {
                        if (getAll.length() == 0) {
                            state = true;
                        }
                        getAll = "";
                    } else if (c != '\r') {
                        getAll += String(c);
                    }
                    startTime = millis();
                }
                if (getBody.length() > 0) {
                    break;
                }
            }
            wifiClient.stop();
            Serial.println(getBody);
        } else {
            getBody = "Connected to " + String(myDomain) + " failed.";
            Serial.println("[INFO] Connected to " + String(myDomain) + " failed.");
        }
        Serial.println("[INFO] File uploading done.");
        Serial.println("===================================");
    } else {    // no motion detected
        Serial.print(".");
    }

     std::vector<ObjectDetectionResult> results = ObjDet.getResult();

    uint16_t im_h = config2.height();
    uint16_t im_w = config2.width();

    Serial.print("Network URL for RTSP Streaming: ");
    Serial.print("rtsp://");
    Serial.print(ip);
    Serial.print(":");
    Serial.println(rtsp_portnum);
    Serial.println(" ");

    printf("Total number of objects detected = %d\r\n", ObjDet.getResultCount());
    OSD.createBitmap(CHANNEL1);

    if (ObjDet.getResultCount() > 0) {
        for (uint32_t i = 0; i < ObjDet.getResultCount(); i++) {
            int obj_type = results[i].type();
            if (itemList[obj_type].filter) {    // check if item should be ignored

                ObjectDetectionResult item = results[i];
                // Result coordinates are floats ranging from 0.00 to 1.00
                // Multiply with RTSP resolution to get coordinates in pixels
                int xmin = (int)(item.xMin() * im_w);
                int xmax = (int)(item.xMax() * im_w);
                int ymin = (int)(item.yMin() * im_h);
                int ymax = (int)(item.yMax() * im_h);

                // Draw boundary box
                printf("Item %d %s:\t%d %d %d %d\n\r", i, itemList[obj_type].objectName, xmin, xmax, ymin, ymax);
                OSD.drawRect(CHANNEL1, xmin, ymin, xmax, ymax, 3, OSD_COLOR_WHITE);

                // Print identification text
                char text_str[20];
                snprintf(text_str, sizeof(text_str), "%s %d", itemList[obj_type].objectName, item.score());
                OSD.drawText(CHANNEL1, xmin, ymin - OSD.getTextHeight(CHANNEL1), text_str, OSD_COLOR_CYAN);
            }
        }
    }
    OSD.update(CHANNEL1);

    // delay to wait for new results
    delay(100);

     backupButtonState = digitalRead(BACKUP_FACE_BUTTON_PIN);
    RegModeButtonState = digitalRead(EN_REGMODE_BUTTON_PIN);

    if ((backupButtonState == HIGH) && (regMode == true)) {    // Button is pressed when registration mode is on
        for (int count = 0; count < 3; count++) {
            digitalWrite(RED_LED, HIGH);
            digitalWrite(GREEN_LED, HIGH);
            delay(500);
            digitalWrite(RED_LED, LOW);
            digitalWrite(GREEN_LED, LOW);
            delay(500);
        }
        facerecog.backupRegisteredFace();    // back up registered faces to flash
        regMode = false;                     // Off registration mode
    }

    if (Serial.available() > 0) {
        String input = Serial.readString();
        input.trim();

        if (regMode == true) {
            if (input.startsWith(String("REG="))) {
                String name = input.substring(4);
                facerecog.registerFace(name);
            } else if (input.startsWith(String("DEL="))) {
                String name = input.substring(4);
                facerecog.removeFace(name);
            } else if (input.startsWith(String("RESET"))) {
                facerecog.resetRegisteredFace();
            } else if (input.startsWith(String("BACKUP"))) {
                facerecog.backupRegisteredFace();
            } else if (input.startsWith(String("RESTORE"))) {
                facerecog.restoreRegisteredFace();
            }
        }
    }

    if (regMode == false) {
        digitalWrite(RED_LED, LOW);
        digitalWrite(GREEN_LED, LOW);
        if ((RegModeButtonState == HIGH)) {
            regMode = true;
            digitalWrite(RED_LED, HIGH);
            digitalWrite(GREEN_LED, HIGH);
        }
    } else {
        digitalWrite(RED_LED, HIGH);
        digitalWrite(GREEN_LED, HIGH);
    }

    // Take snapshot and open door for 10 seconds
    if ((doorOpen == true) && (regMode == false)) {
        fs.begin();
        File file = fs.open(String(fs.getRootPath()) + fileName + String(++counter) + ".jpg");

        delay(1000);
        Camera.getImage(CHANNELJPEG, &img_addr, &img_len);
        file.write((uint8_t *)img_addr, img_len);
        file.close();
        fs.end();
        myservo.write(0);
        Serial.println("Opening Door!");

        delay(10000);
        myservo.write(180);
        digitalWrite(RED_LED, LOW);
        digitalWrite(GREEN_LED, HIGH);
        doorOpen = false;
    }

    delay(2000);
    OSD.createBitmap(CHANNELVID);
    OSD.update(CHANNELVID);

    // Button state
    int newButtonState = digitalRead(buttonPin);
    if (newButtonState != buttonState) {
        buttonPressTime = millis();
    }
    // update button state
    buttonState = newButtonState;

    // update recording state
    recordingstate = (int)(mp4.getRecordingState());

    // check if the button has been held for at least 2 seconds
    if (buttonState == HIGH && millis() - buttonPressTime >= 2000) {
        // button has been pressed for at least 2 seconds
        buttonPressedFor2Seconds = true;
    } else {
        // button was released before 2 seconds
        buttonPressedFor2Seconds = false;
    }
    // if button has been pressed for at least 2 seconds
    if (buttonPressedFor2Seconds) {
        if (recordingstate == 1) {
            digitalWrite(LED_BUILTIN, HIGH);
        } else {
            mp4.begin();
            Serial.println("Recording");
        }
    }
    if (recordingstate == 1 && previousRecordingState == 0) {
        // Change from 0 to 1
        digitalWrite(LED_BUILTIN, HIGH);
    } else if (recordingstate == 0 && previousRecordingState == 1) {
        encodeMP4andsendHttpPostRequest();
        // Change from 1 to 0
        digitalWrite(LED_BUILTIN, LOW);  
    }

    // Check if there are incoming bytes available from the server
    while (wifiClient.available()) {
        char c = wifiClient.read();
        Serial.write(c);
    }
    previousRecordingState = recordingstate;
    delay(10);

  Serial.println("Forward...");
  backward();
  digitalWrite(LED, HIGH);
  delay(1000);
  stop();
  digitalWrite(LED, LOW);
  delay(2000);

}



void encodeMP4andsendHttpPostRequest()
{
    memset(buf, 0, sizeof(buf));
    fs.readDir(fs.getRootPath(), buf, sizeof(buf));
    filepath = String(fs.getRootPath()) + String(FILENAME);
    p = buf;
    while (strlen(p) > 0) {
        /* list out file name image will be saved as "TestRecordingAudioOnly.mp4"*/
        if (strstr(p, FILENAME) != NULL) {
            Serial.println("[INFO] Found 'TestRecordingAudioOnly.mp4' in the string.");
            Serial.println("[INFO] Processing file...");
        } else {
            // Serial.println("Substring 'image.jpg' not found in the
            // string.");
        }
        p += strlen(p) + 1;
    }
    uint8_t *fileinput;
    file = fs.open(filepath);
    unsigned int fileSize = file.size();
    fileinput = (uint8_t *)malloc(fileSize + 1);
    file.read(fileinput, fileSize);
    fileinput[fileSize] = '\0';
    file.close();

    // Encode the file data as Base64
    int encodedLen = base64_enc_len(fileSize);
    char *encodedData = (char *)malloc(encodedLen);
    base64_encode(encodedData, (char *)fileinput, fileSize);

    JsonDocument doc;

    //Change "base64_string" to the key that you set in your server.
    doc["base64_string"] = encodedData;
    String jsonString;
    serializeJson(doc,jsonString);

    if (wifiClient.connect(server, PORT)) {
        wifiClient.println("POST /audio HTTP/1.1");
        wifiClient.println("Host: " + String(server));
        wifiClient.println("Content-Type: application/json");    // Use appropriate content type
        wifiClient.println("Content-Length: " + String(jsonString.length()));              // Specify the length of the content
        wifiClient.println("Connection: keep-alive");
        wifiClient.println();             // Empty line indicates the end of headers
        wifiClient.print(jsonString);    // Send the Base64 encoded audio data directly
        
        Serial.println("Binary sent");
    }
}

// User callback function for post processing of face recognition results
void FRPostProcess(std::vector<FaceRecognitionResult> results)
{
    uint16_t im_h = configVID.height();
    uint16_t im_w = configVID.width();

    printf("Total number of faces detected = %d\r\n", facerecog.getResultCount());
    OSD.createBitmap(CHANNELVID);

    if (facerecog.getResultCount() > 0) {
        if (regMode == false) {
            if (facerecog.getResultCount() > 1) {    // Door remain close when more than one face detected
                doorOpen = false;
                digitalWrite(RED_LED, HIGH);
                digitalWrite(GREEN_LED, LOW);
            } else {
                FaceRecognitionResult face = results[0];
                if (String(face.name()) == String("unknown")) {    // Door remain close when unknown face detected
                    doorOpen = false;
                    digitalWrite(RED_LED, HIGH);
                    digitalWrite(GREEN_LED, LOW);
                } else {    // Door open when a single registered face detected
                    doorOpen = true;
                    digitalWrite(RED_LED, LOW);
                    digitalWrite(GREEN_LED, HIGH);
                    fileName = String(face.name());
                }
            }
        }
    }

    for (int i = 0; i < facerecog.getResultCount(); i++) {
        FaceRecognitionResult item = results[i];
        // Result coordinates are floats ranging from 0.00 to 1.00
        // Multiply with RTSP resolution to get coordinates in pixels
        int xmin = (int)(item.xMin() * im_w);
        int xmax = (int)(item.xMax() * im_w);
        int ymin = (int)(item.yMin() * im_h);
        int ymax = (int)(item.yMax() * im_h);

        uint32_t osd_color;

        // Draw boundary box
        if (String(item.name()) == String("unknown")) {
            osd_color = OSD_COLOR_RED;
        } else {
            osd_color = OSD_COLOR_GREEN;
        }
        printf("Face %d name %s:\t%d %d %d %d\n\r", i, item.name(), xmin, xmax, ymin, ymax);
        OSD.drawRect(CHANNELVID, xmin, ymin, xmax, ymax, 3, osd_color);

        // Print identification text above boundary box
        char text_str[40];
        snprintf(text_str, sizeof(text_str), "Face:%s", item.name());
        OSD.drawText(CHANNELVID, xmin, ymin - OSD.getTextHeight(CHANNELVID), text_str, osd_color);
    }
    OSD.update(CHANNELVID);

    Serial.println("Forward...");
    backward();
    digitalWrite(LED, HIGH);
    delay(1000);
    stop();
    digitalWrite(LED, LOW);
    delay(2000);
}




void mdPostProcess(std::vector<MotionDetectionResult> md_results)
{
    // Motion detection results is expressed as an array
    // With 0 or 1 in each element indicating presence of motion
    // Iterate through all elements to check for motion
    // and calculate rectangles containing motion

    OSD.createBitmap(CHANNEL);
    if (MD.getResultCount() > 0) {
        for (uint16_t i = 0; i < MD.getResultCount(); i++) {
            MotionDetectionResult result = md_results[i];
            int xmin = (int)(result.xMin() * config1.width());
            int xmax = (int)(result.xMax() * config1.width());
            int ymin = (int)(result.yMin() * config1.height());
            int ymax = (int)(result.yMax() * config1.height());
            // printf("%d:\t%d %d %d %d\n\r", i, xmin, xmax, ymin, ymax);
            OSD.drawRect(CHANNEL, xmin, ymin, xmax, ymax, 3, COLOR_GREEN);
        }
        flag_motion = true;
    } else {
        flag_motion = false;
    }
    OSD.update(CHANNEL);
    delay(100);
}



// https://www.arduino.cc/reference/en/libraries/urlencode/
String urlencode(String str)
{
    const char *msg = str.c_str();
    const char *hex = "0123456789ABCDEF";
    String encodedMsg = "";
    while (*msg != '\0') {
        if (('a' <= *msg && *msg <= 'z') || ('A' <= *msg && *msg <= 'Z') || ('0' <= *msg && *msg <= '9') || *msg == '-' || *msg == '_' || *msg == '.' || *msg == '~') {
            encodedMsg += *msg;
        } else {
            encodedMsg += '%';
            encodedMsg += hex[(unsigned char)*msg >> 4];
            encodedMsg += hex[*msg & 0xf];
        }
        msg++;
    }
    return encodedMsg;
}

void CamFlash()
{
    pinMode(LED_G, OUTPUT);
    for (int i = 0; i < 5; i++) {
        digitalWrite(LED_G, HIGH);
        delay(100);
        digitalWrite(LED_G, LOW);
        delay(100);
    }
}

void WiFiCon()
{
    pinMode(LED_B, OUTPUT);
    for (int i = 0; i < 2; i++) {
        digitalWrite(LED_B, HIGH);
        delay(300);
        digitalWrite(LED_B, LOW);
        delay(300);
    }
}

void StreamEnd()
{
    videoStreamer1.pause();    // pause linkers
    videoStreamerMD.pause();
    rtsp.end();              // stop RTSP chaneel/module
    Camera.channelEnd();     // stop camera channel/module
    MD.end();                // close module
    Camera.videoDeinit();    // video deinit
    delay(1000);
}

