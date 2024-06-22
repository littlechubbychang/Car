## The "main code" of the project is "CAR.ino".<br />這個報告的「主要運行程式碼」在「CAR.ino」。  

***
 
# Stir-fried Minced Pork Car
### 1.人臉辨識：
>**註冊組員臉部相片，當車子偵測到組員的臉時，亮起LED燈**

### 2.聲音辨識：
>**當偵測到"yes"時，車子前進；當偵測到"no"時，車子後退；當偵測到拍手聲時，車子轉圈。**

### 3.物件偵測:
>**透過鏡頭看到畫面可以偵測出鏡頭裡的物體是什麼。**

### 4.訊息傳輸:
>**可將鏡頭所看到畫面傳輸至組員的LINE。**

### 5.機體遙控:
>**可以透過APP連結使用手機遙控車子行動。**

***
## Development Board Introduciton
### AMB82-MINI overview

![](https://camo.githubusercontent.com/2f3185a599037adeadd9b22bcc49a77b9a53198a5315207128641e1fa462ba26/68747470733a2f2f7777772e616d656261696f742e636f6d2f77702d636f6e74656e742f75706c6f6164732f323032332f30332f616d6238325f6d696e692e706e67)

https://www.amebaiot.com/en/amebapro2/

https://www.amebaiot.com/en/ameba-arduino-summary/

https://www.amebaiot.com/en/amebapro2-amb82-mini-arduino-peripherals-examples

https://www.amebaiot.com/en/amebapro2-amb82-mini-arduino-getting-started/

***

## System Block Diagram

![image](https://github.com/littlechubbychang/Car/assets/168112225/b05d2eb6-453c-440d-9dc6-694fba47bd45)

***

## Code

# Code:[CAR.ino](https://github.com/littlechubbychang/Car/blob/main/CAR.ino)<br />
以下為部分程式碼展示
```
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

```

# Google Apps Script的執行程式碼有兩種：
**可依情況自行選擇**<br /><br />
**第一種(未包含Line Notify)：**
```
function doPost(e) {
// Retrieve variables from the POST requests
var myFoldername = e.parameter.myFoldername;
var myFile = e.parameter.myFile;
var myFilename = Utilities.formatDate(new Date(), "GMT", "yyyyMMddHHmmss")+"-"+e.parameter.myFilename;
var myToken = e.parameter.myToken;

// Store the file type and Base64 encoded data
var contentType = myFile.substring(myFile.indexOf(":")+1, myFile.indexOf(";"));
var data = myFile.substring(myFile.indexOf(",")+1);
data = Utilities.base64Decode(data);
var blob = Utilities.newBlob(data, contentType, myFilename);

// Save a captured image to Google Drive.
var folder, folders = DriveApp.getFoldersByName(myFoldername);
if (folders.hasNext()) {
folder = folders.next();
} else {
folder = DriveApp.createFolder(myFoldername);
}
var file = folder.createFile(blob);
file.setDescription("Uploaded by " + myFilename);

// Returning Results
return ContentService.createTextOutput(myFoldername+"/"+myFilename+"\n"+imageUrl+"\n"+res);
}
```
**第二種(包含Line Notify)：**
```
function doPost(e) {
  var myFoldername = e.parameter.myFoldername;
  var myFile = e.parameter.myFile;
  //var myFilename = e.parameter.myFilename;
  var myFilename = Utilities.formatDate(new Date(), "GMT", "yyyyMMddHHmmss")+"-"+e.parameter.myFilename;
  var myToken = e.parameter.myToken;
  
  var contentType = myFile.substring(myFile.indexOf(":")+1, myFile.indexOf(";"));
  var data = myFile.substring(myFile.indexOf(",")+1);
  data = Utilities.base64Decode(data);
  var blob = Utilities.newBlob(data, contentType, myFilename);
  
  // Save a captured image to Google Drive.
  var folder, folders = DriveApp.getFoldersByName(myFoldername);
  if (folders.hasNext()) {
    folder = folders.next();
  } else {
    folder = DriveApp.createFolder(myFoldername);
  }
  var file = folder.createFile(blob);    
  file.setDescription("Uploaded by " + myFilename);
  
  var imageID = file.getUrl().substring(file.getUrl().indexOf("/d/")+3,file.getUrl().indexOf("view")-1);
  var imageUrl = "https://drive.google.com/uc?authuser=0&id="+imageID;
    
  // Send a link message to Line Notify.
  var res = "Line Notify: ";
  try {
    var url = 'https://notify-api.line.me/api/notify';
    var response = UrlFetchApp.fetch(url, {
      'headers': {
        'Authorization': 'Bearer ' + myToken,
      },
      'method': 'post',
      'payload': {
          'message': imageUrl
      }
    });
    res += response.getContentText();
  } catch(error) {
    res += error;
  } 
    
  return  ContentService.createTextOutput(myFoldername+"/"+myFilename+"\n"+imageUrl+"\n"+res);
}
```
**存取設定要給「所有人」：**
![image](https://github.com/littlechubbychang/Car/assets/161729298/c5fcf4bb-d576-4695-84fb-0b838d610867)



***

## Demo Video
>[![IMAGE ALT TEXT](http://img.youtube.com/vi/vr_V1QnVMts/0.jpg)](https://www.youtube.com/watch?v=vr_V1QnVMts "鴨嘴獸泰式打拋豬小車車專題報告 微控制器介面與驅動設計")

