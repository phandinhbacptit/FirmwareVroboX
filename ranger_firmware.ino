#include <VbotBoard.h>
#include <VbotDCMotor.h>
#include <Vn74hc595.h>
#include <VnBle.h>
#include <VnButton.h>
#include <VnBuzzer.h>
#include <VnConfig.h>
#include <VnDCMotor.h>
#include <VnIrRemote.h>
#include <VnLedMatrixDefine.h>
#include <VnLightSensor.h>
#include <VnLineFollower.h>
#include <VnMpu6050.h>
#include <VnPort.h>
#include <VnRGB.h>
#include <VnServo.h>
#include <VnSoundSensor.h>
#include <VnUltrasonicSensor.h>

#include <VbotBoard.h>
#include <VbotDCMotor.h>
#include <Vn74hc595.h>
#include <VnBle.h>
#include <VnButton.h>
#include <VnBuzzer.h>
#include <VnConfig.h>
#include <VnDCMotor.h>
#include <VnIrRemote.h>
#include <VnLedMatrixDefine.h>
#include <VnLightSensor.h>
#include <VnLineFollower.h>
#include <VnMpu6050.h>
#include <VnPort.h>
#include <VnRGB.h>
#include <VnServo.h>
#include <VnSoundSensor.h>
#include <VnUltrasonicSensor.h>

#include <VbotBoard.h>
#include <VbotDCMotor.h>
#include <Vn74hc595.h>
#include <VnBle.h>
#include <VnButton.h>
#include <VnBuzzer.h>
#include <VnConfig.h>
#include <VnDCMotor.h>
#include <VnIrRemote.h>
#include <VnLedMatrixDefine.h>
#include <VnLightSensor.h>
#include <VnLineFollower.h>
#include <VnMpu6050.h>
#include <VnPort.h>
#include <VnRGB.h>
#include <VnServo.h>
#include <VnSoundSensor.h>
#include <VnUltrasonicSensor.h>
#include <VnColorSensor.h>

/* Global Config */
/*---------------------------------------------------------------------------*/
#if (1) 
#define ROBOX_LOG(...)   Serial.printf(__VA_ARGS__)
#else
#define ROBOX_LOG(...)
#endif

//#define MOTOR_TYPE_3
//#define VBOT_CHAIN


/* include */
/*---------------------------------------------------------------------------*/
#include "VbotBoard.h"
#include "BluetoothSerial.h"
#include "FastLED.h"
#include <stdio.h>
#include <string.h>

/* Global Structure */
/*---------------------------------------------------------------------------*/
union{
  byte byteVal[2];
  short shortVal;
}valShort;

static unsigned int maxtrix_display[8] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};
static unsigned int ringled_display[12] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};
static unsigned int fbSensorData[9] = {0xff, 0x55, 0x01, 0x02, 0x03, 0x04, 0x05, 0x0D, 0x0A};
/* Function Prototype */
/*---------------------------------------------------------------------------*/
void robotInit(void);
void robotStartup(void);
void BleInit(void);
static void serialHandle();
static void soundEffect(void);
static void robotFollowingLine(void);
static void go_in_circle(void);
static void go_demo_srf05_lighsensor(void);

/* Global Define */
/*---------------------------------------------------------------------------*/
#define GET 1
#define RUN 2
#define RESET 4
#define START 5

#define ULTRASONIC_SENSOR       1
#define LINEFOLLOWER            2
#define LIGHT_SENSOR_VALUE      3
#define COLOR_SENSOR            4
#define JOYSTICK                5
#define BTN_MODE                6
#define LEDMATRIX               7
#define RGBLED                  8
#define TONE                    9
#define SOUND_SENSOR            10
#define RING_LED                11


#define LINE_DETECT_MODE      100
#define LINE_CIRCLE_MODE      101
#define SOUND_FOLLOW_MODE     102
#define NORMAL_MODE           103


#define NUM_LEDS              2
#define LED_DATA_PIN          23

#define NUM_RING_LED          12
#define LED_DATA_RING_LED     21 

#define DEVICE_NAME "Robox"
/* Hardware API */
/*---------------------------------------------------------------------------*/
VnDCMotor           DcMotorL(M2);
VnDCMotor           DcMotorR(M3);
CRGB                leds[NUM_LEDS];
CRGB                ring_leds[NUM_RING_LED];
VnUltrasonicSensor  Ultra(ULTRA);
VnBuzzer            Buzzer;
VnLineFollower      LINE(FLOWLINE);
VnLightSensor       LightSensor(LIGHT_SENSOR);
VnSoundSensor       SoundSensor(SOUND);
VnButton            Btn(BUTTON);
Vn74hc595           Matrix(MAXTRIX);
VnColorSensor       ColorSensor(COLOR);
/* Global Variable */
/*---------------------------------------------------------------------------*/
BluetoothSerial SerialBT;
TaskHandle_t Task1;

/* Setup function */
/*---------------------------------------------------------------------------*/
void convertint2byte(int value)
{
  byte buf[2];
  buf[0] = (byte) (value & 0xff);
  buf[1] = (byte) ((value >> 8) & 0xff);
  ROBOX_LOG("Byte1:%x ", buf[0]);
  ROBOX_LOG("Byte2:%x ", buf[1]); 
}
void setup()
{
  robotInit();
	Serial.begin(115200);
  robotStartup();
	SerialBT.begin(DEVICE_NAME);
//  BleInit();

//  xTaskCreatePinnedToCore(
//    Task1code, /* Function to implement the task */
//    "Task1", /* Name of the task */
//    8024,  /* Stack size in words */
//    NULL,  /* Task input parameter */
//    0,  /* Priority of the task */
//    &Task1,  /* Task handle. */
//    0); /* Core where the task should run */
}

/* Main Loop */
/*---------------------------------------------------------------------------*/

void led_matrix_test(void)
{
      Vn74hc595           Matrix(MAXTRIX);
      
      Matrix.displayImage(all_on_effect, 500);
      delay(500);
      for (int cnt = 0; cnt < 27; cnt++) {
         Matrix.displayImage(motion_effect3[cnt], 50);
      }
      delay(3000); 
}
int i = 0;
void loop() 
{
   //robotSetJoyStick(100, 100);
//    DcMotorL.run(150, MOTOR1);
//    DcMotorL.run(-150, MOTOR2);
//    DcMotorR.run(-150, MOTOR3);
    serialHandle();
//    color_sensor_test();
//    led_matrix_test();
}

int mode = NORMAL_MODE;
void Task1code( void * parameter) 
{
  while(1) {
    switch (mode)
    {
    case NORMAL_MODE:
      /* do nothing */
      break;
    case LINE_DETECT_MODE: {
        robotFollowingLine();
        //ROBOX_LOG("LINE_DETECT_MODE\n");
      }
      break;
    case LINE_CIRCLE_MODE: {
        go_in_circle();
        //ROBOX_LOG("LINE_CIRCLE_MODE\n");
      }
      break;
    case SOUND_FOLLOW_MODE: {
        soundEffect();
        //ROBOX_LOG("SOUND_FOLLOW_MODE\n");
      }
      break;
    default:
      break;
    }
    delay(100);
  }
}

void robotInit(void)
{
  FastLED.addLeds<NEOPIXEL, LED_DATA_PIN>(leds, NUM_LEDS); 
  FastLED.addLeds<NEOPIXEL, LED_DATA_RING_LED>(ring_leds, NUM_RING_LED);
  for (int i = 0; i < NUM_LEDS; i++)
    leds[i] = CRGB::Black;
  FastLED.show();  
}

void robotStartup(void) 
{
  for (i = 0; i < 5; i++) {
    robotSetLed(0, 255, 0, 0);
    delay(500);
    robotSetLed(0, 0, 0, 0);
    delay(500);
  }
}

void robotSetJoyStick(int leftSpeed, int rightSpeed)
{
  DcMotorL.run(leftSpeed, MOTOR2);
  DcMotorR.run(rightSpeed, MOTOR3);
  ROBOX_LOG("JOYSTICK leftSpeed=%d, rightSpeed=%d\n", leftSpeed, rightSpeed);
}

static int rgb2HexColor(int r, int g, int b)
{
    return (r<<16) | (g<<8) | b;
}

void robotSetLed(int idx, int r, int g, int b)
{
  if (idx < 0 || idx > NUM_LEDS)
    return;

  int color = rgb2HexColor(r, g, b);
  if (idx == 0) {
    for (int i = 0; i < NUM_LEDS; i++)
    leds[i] = color;
  } else {
      leds[idx - 1] = color;
  }
  FastLED.show();
  
  ROBOX_LOG("LEd idx=%d, r=%d, g=%d, b=%d\n", idx, r, g, b);
}
/*Effect ring led 1*/
void rainbow_march(uint8_t thisdelay, uint8_t deltahue)  
{
  uint8_t thishue = millis()*(255-thisdelay)/255;             // To change the rate, add a beat or something to the result. 'thisdelay' must be a fixed value.
  FastLED.setBrightness(50);
  fill_rainbow(ring_leds, NUM_RING_LED, thishue, deltahue);            // Use FastLED's fill_rainbow routine.
  Serial.println("");
  for (int i = 0;  i < NUM_RING_LED; i++) {
    Serial.print(ring_leds[i]);
  }
}
/*****************************************************************************************/
void robotSetRingLed(unsigned int affect[])
{
  switch (affect[0]) {
    case 1: {
      rainbow_march(200,20);
      break;
    }
    default:
      for (int i = 0; i < NUM_RING_LED; i++) {
         ring_leds[i] = affect[i]; 
      }
  }
  FastLED.show();

  ROBOX_LOG("\n Ring Led: %d %d %d %d %d %d %d %d %d %d %d %d ", affect[0], affect[1], affect[2], affect[3], affect[4], affect[5]
              , affect[6], affect[7], affect[8], affect[90], affect[10], affect[11]);
}
void robotSetTone(int freq, int duration)
{
  Buzzer.tone(freq, duration);
  ROBOX_LOG("TONE freq=%d, duration=%d\n", freq, duration);
}

void robotSetMatrix(unsigned int display[], int duration)
{
  Matrix.displayImage(display, duration);
//  ROBOX_LOG("Display: %d %d %d %d %d %d %d %d \n", 
//            display[0],display[1],display[2],display[3],display[4],display[5],display[6],display[7]);
}
float robotGetDistance(void)
{
  float distance = (float)Ultra.distanceCm(200);
  ROBOX_LOG("Distance = %.2f\n", distance);
  return distance;
}

float robotGetLineSensor(void)
{
  float line = (float)(LINE.readSensor1() * 2 + LINE.readSensor2());
  ROBOX_LOG(" LineSensor = %f\n", line);
  return line;
}

float robotGetLightSensor(void)
{
  float value = (float)(LightSensor.read() * 100 / 4095);
  ROBOX_LOG("LightSensor = %.1f   %d\n", value, LightSensor.read());
  return value;
}
float robotGetColorSensor(void) {
  float value = (float) (ColorSensor.detect_color());
  ROBOX_LOG("ColorSensor = %.1f \n", value);
  return value;
}

float robotGetSoundSensor(void)
{
  float value = (float)(SoundSensor.readSoundSignal());
  ROBOX_LOG("SoundSensor = %.1f", value);
  return value;
}

float robotGetButtonState(void)
{
  float value = (float)(Btn.read_mode());
  ROBOX_LOG("StateBtn = %.1f", value);
  return value;
}


/* Classic Bluetooth API for Android */
/*---------------------------------------------------------------------------*/
char buffer[64];
boolean isAvailable = false;
boolean isStart     = false;
char serialRead     = 0;
unsigned char prevc = 0;
byte index2          = 0;
byte dataLen        = 0;
uint8_t command_index = 0;
union
{
  byte byteVal[4];
  float floatVal;
  long longVal;
}val;

static void writeBuffer(int index1, unsigned char c){
  buffer[index1] = c;
}

static void readSerial(){
  isAvailable = false;
  if(SerialBT.available()>0){
    isAvailable = true;
    serialRead = SerialBT.read();
    //ROBOX_LOG("%x", serialRead);
  }
}

static void serialHandle(){
  readSerial();
  if(isAvailable){
    unsigned char c = serialRead&0xff;
    if(c == 0x55 && isStart == false){
     if(prevc == 0xff){
      index2 = 1; 
      isStart = true;
     }
    }else{
      prevc = c;
      if (isStart){
        if (index2 == 2){
         dataLen = c; 
        }else if (index2 > 2){
          dataLen--;
        }
        writeBuffer(index2,c);
      }
    }
     index2++;
     if(index2>51){
      index2=0; 
      isStart=false;
     }
     if (isStart && dataLen == 0 && index2 > 3){ 
        isStart = false;
        parseData();
        index2 = 0;
     }
  }
}

void writeHead()
{
  writeSerial(0xff);
  writeSerial(0x55);
}

static void writeSerial(unsigned char c){
  SerialBT.write(c);
  //ROBOX_LOG(" %x ", c);
}

static void writeEnd(){
  SerialBT.println();
  ROBOX_LOG("0D 0A"); 
}

static void callOK(){
    writeSerial(0xff);
    writeSerial(0x55);
    writeEnd();
}

static unsigned char readBuffer(int index){
  return buffer[index]; 
}

static short readShort(int idx){
  valShort.byteVal[0] = readBuffer(idx);
  ROBOX_LOG("Byte1:%x ", valShort.byteVal[0]);
  valShort.byteVal[1] = readBuffer(idx+1);
  ROBOX_LOG("Byte2:%x ", valShort.byteVal[1]);
  return valShort.shortVal; 
}

static void runModule(int device){
  //0xff 0x55 0x6 0x0 0x2 0x22 0x9 0x0 0x0 0xa
//  ROBOX_LOG("Received command\n");
  int port = readBuffer(6);
  int temp_buf[36];
  int pin = port;
  switch (device) {
    case JOYSTICK:{
      int leftSpeed = readShort(8);
      int rightSpeed = readShort(10);
      robotSetJoyStick(leftSpeed, rightSpeed);
      break;
    }
    case RGBLED:{
      int idx = readBuffer(8);
      int r = readBuffer(9);
      int g = readBuffer(10);
      int b = readBuffer(11);
      robotSetLed(idx, r, g, b);
      break;
    }
    case RING_LED: {
      ROBOX_LOG("\n");
      for (int i = 0; i < 36; i++) {
        temp_buf[i] = (int) readBuffer(i+9);
//        ringled_display[i] = (int) readBuffer(i+8);
//        ringled_display[i/3] = (int) rgb2HexColor(readBuffer(i + 9), readBuffer(i + 10), readBuffer(i + 11));
//          int color = rgb2HexColor(r, g, b);
          //ROBOX_LOG(" %x", readBuffer(i+9));
      }
//      for (int j = 0; j < 36; j++) {
//         ROBOX_LOG(" %x", temp_buf[j]);
//      }
      for (int j = 0; j < 12; j++) {
        ringled_display[j] = (int) rgb2HexColor(temp_buf[(j*3) + 0], temp_buf[(j*3) + 1], temp_buf[(j*3) + 2]); 
      }
      robotSetRingLed(ringled_display);
      break;
    }
    
    case TONE:{
      int freq = readShort(8);  
      int duration = readShort(10);
      robotSetTone(freq, duration);
      break;
    }
    case LEDMATRIX: {
      for (int i = 0; i < 8; i++) {
        maxtrix_display[i] = (int)readBuffer(i+ 8); 
      }
      ROBOX_LOG("\n matrix code: %x %x %x %x %x %x %x ", 
      maxtrix_display[0],maxtrix_display[1], maxtrix_display[2], maxtrix_display[3],
      maxtrix_display[4], maxtrix_display[5], maxtrix_display[6], maxtrix_display[7]);
      robotSetMatrix(maxtrix_display, readBuffer(16));
      break;
    }
    case LINE_DETECT_MODE:
    case LINE_CIRCLE_MODE:
    case SOUND_FOLLOW_MODE:
    case NORMAL_MODE:
      ROBOX_LOG("Mode=%d\n", mode);
      mode = device;
      break;
  }
}

void sendFloat(float value)
{  
  val.floatVal = value;
  writeSerial(val.byteVal[0]);
  writeSerial(val.byteVal[1]);
  writeSerial(val.byteVal[2]);
  writeSerial(val.byteVal[3]);
}

void readSensor(int device)
{
  /**************************************************
      ff    55      len idx action device port slot data a
      0     1       2   3   4      5      6    7    8
      0xff  0x55   0x4 0x3 0x1    0x1    0x1  0xa 
  ***************************************************/
  float value = 0.0;
  int port,slot,pin;
  port = readBuffer(6);
  pin = port;

  switch(device)
  {
    case ULTRASONIC_SENSOR:
    {
      ROBOX_LOG("\n Read ultrasonic sensor -- ");
      value = (float)robotGetDistance();
      break;
    }
    case LINEFOLLOWER:
    {     
      ROBOX_LOG(" \n Read line sensor -- ");
      value = (float)robotGetLineSensor();
      break;
    }
    case LIGHT_SENSOR_VALUE:
    {     
      ROBOX_LOG("\n Read light sensor -- ");
      value = (float)robotGetLightSensor();
      break;
    }
    case COLOR_SENSOR: 
    {        
      ROBOX_LOG("\n Read color sensor -- "); 
      value = robotGetColorSensor();
      break;
    }
    case BTN_MODE: {
      ROBOX_LOG("\n Read state Button --");
      value = (float)robotGetButtonState();
      break;
    }
    case SOUND_SENSOR: {
      ROBOX_LOG("\n Read Sound sensor --");
      value = (float)robotGetSoundSensor();
      break;
    }
  }
  writeHead();
  writeSerial(device);
  sendFloat(value);
}

static void parseData(){
  isStart = false;
  int idx = readBuffer(3);
  command_index = (uint8_t)idx;
  int action = readBuffer(4);
  int device = readBuffer(5);
  ROBOX_LOG(" device: %x", device);
  switch(action){
    case GET:{
       readSensor(device);
       writeEnd();
    } 
     break;
     case RUN:{
       runModule(device);
       //callOK();
     }
      break;
      case RESET:{
        //reset
        callOK();
      }
     break;
     case START:{
        //start
        callOK();
      }
     break;
  }
}

/* Bluetooth Low Energy API for iOs */
/*---------------------------------------------------------------------------*/
#define SERVICE_UUID      "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define ROBOT_UUID        "beb5483e-36e1-4688-b7f5-ea07361b26a8"

class BleServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      uint8_t address[6] = {0xe0, 0x99, 0x71, 0xb6, 0x02, 0xc4};
      ROBOX_LOG("BLE Connected\n");
    };

    void onDisconnect(BLEServer* pServer) {
      ROBOX_LOG("BLE Disconnected\n");
    }
};

class BLERobotCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      uint8_t charBuf[64];      
      std::string value = pCharacteristic->getValue();

      ROBOX_LOG("BLE write: ");
      for (int i = 0; i < value.length(); i++) {
        charBuf[i] = value[i];
        ROBOX_LOG("%x ", charBuf[i]);
      }
      ROBOX_LOG("\n");
            
      if (value.length() >= 2) {
        if (value[0] == RUN) {
          if (value[1] == JOYSTICK) {
            joystick(charBuf + 2, value.length() - 2);
          } else if (value[1] == RGBLED) {
            ledRgb(charBuf + 2, value.length() - 2);
          } else if (value[1] == TONE) {
            buzzer(charBuf + 2, value.length() - 2);
          }
        }
      }
    }

    void onRead(BLECharacteristic *pCharacteristic) {
      uint8_t buf[5] = {1, 2, 3, 4, 5};
      ROBOX_LOG("reading...\n");
      pCharacteristic->setValue("anh yeu em");
    }
    
    void ledRgb(uint8_t *value, uint8_t len) {
      if (value && len == 4) {
        uint8_t idx = value[0];
        uint8_t r = value[1];
        uint8_t g = value[2];
        uint8_t b = value[3];
        robotSetLed(idx, r, g, b);
      }
    }

    void joystick(uint8_t *value, uint8_t len) {
          if (value && len == 4) {
            int16_t leftSpeed = (int16_t)(value[0] << 8) + value[1];
            int16_t rightSpeed = (int16_t)(value[2] << 8) + value[3];
            robotSetJoyStick(leftSpeed, rightSpeed);
          }
     }
    
    void buzzer(uint8_t *value, uint8_t len) {
          if (value && len == 4) {
            int16_t freq = (int16_t)(value[0] << 8) + value[1];
            int16_t duration = (int16_t)(value[2] << 8) + value[3];
            robotSetTone(freq, duration);
          }
     }
};


void BleInit(void)
{
  BLEDevice::init(DEVICE_NAME);
  BLEDevice::setPower(ESP_PWR_LVL_P9);
  BLEDevice::setMTU(512);
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new BleServerCallbacks());
  
  BLEService *pService = pServer->createService(SERVICE_UUID);
  BLECharacteristic *pCharacteristic = pService->createCharacteristic(
                                         ROBOT_UUID,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE);                                 
  pCharacteristic->setCallbacks(new BLERobotCallbacks());
  pService->start();
  
  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  BLEAdvertisementData adv1;
  adv1.setName(DEVICE_NAME);
  pAdvertising->setAdvertisementData(adv1);
  
  BLEAdvertisementData adv;
  adv.setCompleteServices(BLEUUID(SERVICE_UUID));
  pAdvertising->setScanResponseData(adv);
  
  pAdvertising->start();
}
static void soundEffect(void)
{
  int sound_detect = 0;
  int state = 0;
  
  for (int i = 0; i < NUM_LEDS; i++){
    leds[i] = CRGB::Blue;
  }
  FastLED.show();
  
  int time = millis();
  while ((millis() - time) < 2500) {
    sound_detect = SoundSensor.readSoundSignal();
    if (sound_detect == 0) {
      delay(100);
      state++;  
    } 
  }
  for (int i = 0; i < NUM_LEDS; i++){
    leds[i] = CRGB::Black;
  }
  FastLED.show();
  switch (state) {
    case 1:
      DcMotorL.run(-170, MOTOR2);
      DcMotorR.run(170, MOTOR3);
      delay(1000);
      break;
    case 2:
      DcMotorL.run(170, MOTOR2);
      DcMotorR.run(-170, MOTOR3);
      delay(1000);
      break;
    case 3:
      DcMotorL.run(-170, MOTOR2);
      DcMotorR.run(-170, MOTOR3);
      delay(1000);
      break;
    case 4:  
      DcMotorL.run(170, MOTOR2);
      DcMotorR.run(170, MOTOR3);
      delay(1000);
      break;
    case 5:  
      for (int i = 0; i < NUM_LEDS; i++){
        leds[i] = CRGB::Green;
      }
      FastLED.show();
      delay(500);
      for (int i = 0; i < NUM_LEDS; i++){
        leds[i] = CRGB::Black;
      }
      FastLED.show();
      
      go_demo_srf05_lighsensor();
      delay(1000);
      break;
    default:
      delay(1000);
      break;  
  }
  DcMotorL.run(0, MOTOR2);
  DcMotorR.run(0, MOTOR3);
}
static void robotFollowingLine(void)
{
  if ((LINE.readSensor1() == 0) && (LINE.readSensor2() == 0)) {
      //Serial.println("find");
      DcMotorL.run(190, MOTOR2);
      DcMotorR.run(170, MOTOR3);
  }
  else if ((LINE.readSensor1() == 1) && (LINE.readSensor2() == 0)) {
      //Serial.println("left");
      DcMotorL.run(-180, MOTOR2);
      DcMotorR.run(-160, MOTOR3);
  }
  else if ((LINE.readSensor1() == 0) && (LINE.readSensor2() == 1)) {
       //Serial.println("right");
      DcMotorL.run(190, MOTOR2);
      DcMotorR.run(160, MOTOR3);
  }
  else if ((LINE.readSensor1() == 1) && (LINE.readSensor2() == 1)) {
      //Serial.println("forward");
      DcMotorL.run(-170, MOTOR2);
      DcMotorR.run(160, MOTOR3);
  }
}

static void go_in_circle(void)
{
    if ((LINE.readSensor1() == 0) && (LINE.readSensor2() == 0)) {
      DcMotorL.run(-160, MOTOR2);
      DcMotorR.run(160, MOTOR3);
    }
    else if ((LINE.readSensor1() == 1) && (LINE.readSensor2() == 1)) {
      Serial.println("forward");
      DcMotorL.run(160, MOTOR2);
      DcMotorR.run(-160, MOTOR3);
      delay(500);
      DcMotorL.run(170, MOTOR2);
      DcMotorR.run(190, MOTOR3);
      delay(1000);
    }
}
static void go_demo_srf05_lighsensor(void)
{
  while (SoundSensor.readSoundSignal()) {
    if (Ultra.distanceCm(200) <= 10) {
      Buzzer.tone(NOTE_A4, 700);
      Buzzer.tone(NOTE_F5, 700);
    }
    Serial.println(LightSensor.read());
    if (LightSensor.read() < 1000) {
      for (int i = 0; i < NUM_LEDS; i++){
        leds[i] = CRGB::White;
      }
      FastLED.show();
    }
    else{
      for (int i = 0; i < NUM_LEDS; i++){
        leds[i] = CRGB::Black;
      }
      FastLED.show();
    }
  }
}
