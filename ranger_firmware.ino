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
#include <EEPROM.h>

#define FLASH_MEMORY_SIZE 13

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

#define SERVICE_UUID      "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define ROBOT_UUID        "beb5483e-36e1-4688-b7f5-ea07361b26a8"
BLECharacteristic *pCharacteristic;
bool stateBle = false;
uint8_t robotFeedback[9] = {0xff, 0x55, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0D, 0x0A};

volatile int cntEnterSleepMode;
hw_timer_t * timerCntEnterSleepMode = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

/* Global Structure */
/*---------------------------------------------------------------------------*/
bool stateConnected = false;
bool oldstateConnected = false;
union{
  byte byteVal[2];
  short shortVal;
}valShort;

static unsigned int maxtrix_display[8] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};
static unsigned int ringled_display[12] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};
static unsigned int fbSensorData[9] = {0xff, 0x55, 0x01, 0x02, 0x03, 0x04, 0x05, 0x0D, 0x0A};
String defaultName = "VROBOX_000000";
int addrSaveNameRobot = 0;
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
static void go_demo_srf05(void);

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
#define SERVO_RUN               12
#define HEARTBIT                15


#define LINE_DETECT_MODE      100
#define LINE_CIRCLE_MODE      101
#define SOUND_FOLLOW_MODE     102
#define SRF05_RUN_MODE        103
#define NORMAL_MODE           104
#define IDE_MODE              0


#define NUM_LEDS              2
#define LED_DATA_PIN          23

#define NUM_RING_LED          12
#define LED_DATA_RING_LED     13 

#define DEVICE_NAME  "VROBOX_00000"
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
VnServo             _servo(SERVO_PORT);
/* Global Variable */
/*---------------------------------------------------------------------------*/
BluetoothSerial SerialBT;
TaskHandle_t Task1;

/* Setup function */
/*---------------------------------------------------------------------------*/
void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  cntEnterSleepMode++;
//  Serial.println(cntEnterSleepMode);
  portEXIT_CRITICAL_ISR(&timerMux);
 
}

void print_wakeup_reason(){
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch(wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup caused by ULP program"); break;
    default : Serial.printf("Wakeup was not caused by deep sleep: %d\n",wakeup_reason); break;
  }
}

void convertint2byte(int value)
{
  byte buf[2];
  buf[0] = (byte) (value & 0xff);
  buf[1] = (byte) ((value >> 8) & 0xff);
  ROBOX_LOG("Byte1:%x ", buf[0]);
  ROBOX_LOG("Byte2:%x ", buf[1]); 
}
void writeStringToEEPROM(int addrOffset, const String &strToWrite)
{
  byte len = strToWrite.length();
  EEPROM.write(addrOffset, len);
  for (int i = 0; i < len; i++)
  {
    EEPROM.write(addrOffset + 1 + i, strToWrite[i]);
  }
}

String readStringFromEEPROM(int addrOffset)
{
  int newStrLen = EEPROM.read(addrOffset);
  char data[newStrLen + 1];
  for (int i = 0; i < newStrLen; i++)
  {
    data[i] = EEPROM.read(addrOffset + 1 + i);
  }
  data[newStrLen] = '\ 0'; // !!! NOTE !!! Remove the space between the slash "/" and "0" (I've added a space because otherwise there is a display bug)
  return String(data);
}

void actionBeforeSleep() {
    robotSetLed(0, 255, 0, 0);
    Buzzer.tone(830, 250);    
    robotSetLed(0, 0, 0, 0);
    delay(100);      
    robotSetLed(0, 255, 0, 0);
    Buzzer.tone(830, 250);  
    robotSetLed(0, 0, 0, 0);
    delay(100);      
    robotSetLed(0, 255, 0, 0);
    Buzzer.tone(830, 250);  
    robotSetLed(0, 0, 0, 0);
    delay(100);    
}
void setup()
{
  String tmpName;
  robotStartup();
	Serial.begin(115200);
  EEPROM.begin(FLASH_MEMORY_SIZE);
  BleInit();
  _servo.attach(1);

/*__________For Sleep mode___________________________*/
//  timerCntEnterSleepMode = timerBegin(1, 80, true);
//  timerAttachInterrupt(timerCntEnterSleepMode, &onTimer, true);
//  timerAlarmWrite(timerCntEnterSleepMode, 1000000, true);
//  timerAlarmEnable(timerCntEnterSleepMode);
//  print_wakeup_reason();
/*_________________________________________________*/

//  writeStringToEEPROM(addrSaveNameRobot, defaultName);
//  EEPROM.commit();
//  delay(2);
//  tmpName = readStringFromEEPROM(addrSaveNameRobot);
//  if (tmpName != "") {
//    Serial.println(tmpName);
//    DEVICE_NAME = tmpName;
//  }
//  else
//    DEVICE_NAME = defaultName;
       
  SerialBT.begin(DEVICE_NAME);
  xTaskCreatePinnedToCore(
    Task1code, /* Function to implement the task */
    "Task1", /* Name of the task */
    8024,  /* Stack size in words */
    NULL,  /* Task input parameter */
    0,  /* Priority of the task */
    &Task1,  /* Task handle. */
    0); /* Core where the task should run */
}

/* Main Loop */
/*---------------------------------------------------------------------------*/

void led_matrix_test(void)
{
      Vn74hc595           Matrix(MAXTRIX);
      
      robotSetMatrix(all_on_effect, 500);     
      delay(500);
      for (int cnt = 0; cnt < 27; cnt++) {
         robotSetMatrix(motion_effect3[cnt], 50);
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
//    _servo.attach(1);
//    for (int i = 0; i < 200; i++) {
//      _servo.rotate(i);
//      delay(1);
//    }
//    robotGetColorSensor();
//    led_matrix_test();
//       FastLED.addLeds<NEOPIXEL, LED_DATA_RING_LED>(ring_leds, NUM_RING_LED);
//       Serial.println("before: ");
//       Serial.print(ring_leds);
//       Vn74hc595           Matrix(MAXTRIX);
//       Serial.println("after: ");
//       Serial.print(ring_leds);

  // disconnecting
  if (!stateConnected && oldstateConnected) {
    delay(500); // give the bluetooth stack the chance to get things ready
    BleInit();
    Serial.println("start advertising");
    oldstateConnected = stateConnected;
  }
  // connecting
  if (stateConnected && !oldstateConnected) {
    // do stuff here on connecting
    oldstateConnected = stateConnected;
  }


/*___________For sleep mode______________________*/
//  if (cntEnterSleepMode >= 180) {
//    actionBeforeSleep();
//    esp_sleep_enable_ext0_wakeup(GPIO_NUM_39,0); //1 = High, 0 = Low
//    Serial.println("Going to sleep now");
//    esp_deep_sleep_start();
//  }     
/*________________________________________________*/
}

int mode = IDE_MODE;
void Task1code( void * parameter) 
{
  while(1) {
    switch (mode)
    {
    case NORMAL_MODE: {
      robotSetJoyStick(0,0);
      ROBOX_LOG("IDE_MODE\n");
      break;
    }
    case LINE_DETECT_MODE: {
        robotFollowingLine();
        ROBOX_LOG("LINE_DETECT_MODE\n");
        break;
    }
    case LINE_CIRCLE_MODE: {
        go_in_circle();
        ROBOX_LOG("LINE_CIRCLE_MODE\n");
        break;
    }
    case SOUND_FOLLOW_MODE: {
        soundEffect();
        ROBOX_LOG("SOUND_FOLLOW_MODE\n");
        break;
      }
    case SRF05_RUN_MODE: {
        go_demo_srf05();
        ROBOX_LOG("SRF05_RUN_MODE\n");
        break;
    }
    default:
      break;
    }
    delay(100);
  }
}


void robotStartup(void) 
{
    FastLED.addLeds<NEOPIXEL, LED_DATA_PIN>(leds, NUM_LEDS); 
    FastLED.addLeds<NEOPIXEL, LED_DATA_RING_LED>(ring_leds, NUM_RING_LED);
    for (int i = 0; i < NUM_LEDS; i++)
    leds[i] = CRGB::Black;
    FastLED.show();  
  
    robotSetLed(0, 255, 0, 0);
    Buzzer.tone(830, 250);
    delay(100);    
    robotSetLed(0, 0, 255, 0);
    Buzzer.tone(554, 250);
    delay(100);    
    robotSetLed(0, 0, 0, 255);
    Buzzer.tone(740, 250);
    delay(100);
    robotSetLed(0, 0, 0, 0);
    robotSetJoyStick(1, 1);
    robotSetJoyStick(0, 0);
    robotSetRingLed(ringled_display);
    robotSetMatrix(maxtrix_display, 0);
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
  int _duration = duration * 250;
  Buzzer.tone(freq, _duration);
  ROBOX_LOG("TONE freq=%d, duration=%d\n", freq, _duration);
}

void robotSetMatrix(unsigned int display[], int duration)
{
  int _duration = duration * 0x42;
  Matrix.displayImage(display, _duration);
  ROBOX_LOG("Duration: %d Display: %d %d %d %d %d %d %d %d \n", 
            _duration, display[0],display[1],display[2],display[3],display[4],display[5],display[6],display[7]);
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
    ROBOX_LOG("%x", serialRead);
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
//  ROBOX_LOG("0D 0A"); 
}

static void callOK(){    
    writeSerial(0xff);
    writeSerial(0x55);
    writeSerial(HEARTBIT);
    sendFloat(12345);
    writeEnd();
    
    robotSetLed(0, 0x00, 0xff, 0x00);
    robotSetJoyStick(200, 200);
    Buzzer.tone(830, 250);
    robotSetLed(0, 0x00, 0x00, 0xff);
    robotSetJoyStick(-200, -200);
    Buzzer.tone(622, 250);
    robotSetLed(0, 0x00, 0x00, 0x00);
    robotSetJoyStick(0, -0);
    SerialBT.flush();
  if(SerialBT.available()>0){
    serialRead = SerialBT.read();
    ROBOX_LOG("start: %x", serialRead);
  }
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
int count = 0;
static void runModule(int device){
  //0xff 0x55 0x6 0x0 0x2 0x22 0x9 0x0 0x0 0xa
//  ROBOX_LOG("Received command\n");
  int port = readBuffer(6);
  int temp_buf[36];
  int pin = port;
  switch (device) {
    case JOYSTICK:{
      mode = IDE_MODE;
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
      }
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
       Vn74hc595           Matrix(MAXTRIX);

      for (int i = 0; i < 8; i++) {
        maxtrix_display[i] = (int)readBuffer(i+ 8); 
      }
      ROBOX_LOG("\n matrix code: %x %x %x %x %x %x %x %x ", 
      maxtrix_display[0],maxtrix_display[1], maxtrix_display[2], maxtrix_display[3],
      maxtrix_display[4], maxtrix_display[5], maxtrix_display[6], maxtrix_display[7]);
      robotSetMatrix(maxtrix_display, readBuffer(16));
      break;
    }
    case LINE_DETECT_MODE:
    case LINE_CIRCLE_MODE:
    case SOUND_FOLLOW_MODE:
    case SRF05_RUN_MODE:
    case NORMAL_MODE:
      ROBOX_LOG("Mode=%d\n", mode);
      mode = device;
      break;
    case SERVO_RUN:
      int angle = (readBuffer(8)) << 8 | readBuffer(9);
      ROBOX_LOG("Angle=%d\n", angle);
      _servo.rotate(angle);
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
  int stateModule = readBuffer(6);
  
  port = readBuffer(7);
  pin = port;

  switch(device)
  {
    case ULTRASONIC_SENSOR:
    {       
      cntEnterSleepMode = 0;
      ROBOX_LOG("\n Read ultrasonic sensor -- ");
      value = (float)robotGetDistance();
      break;
    }
    case LINEFOLLOWER:
    {    
      cntEnterSleepMode = 0; 
      ROBOX_LOG(" \n Read line sensor -- ");
      value = (float)robotGetLineSensor();
      break;
    }
    case LIGHT_SENSOR_VALUE:
    {     
      cntEnterSleepMode = 0;
      ROBOX_LOG("\n Read light sensor -- ");
      value = (float)robotGetLightSensor();
      break;
    }
    case COLOR_SENSOR: 
    { 
      cntEnterSleepMode = 0;
      if (stateModule) {       
        ROBOX_LOG("\n Read color sensor -- "); 
        value = robotGetColorSensor();
      }
      else {
        ColorSensor.disable_color_sensor();              
        ROBOX_LOG("\n Stop read color sensor -- "); 
      }
      break;
    }
    case BTN_MODE: {
      ROBOX_LOG("\n Read state Button --");
      cntEnterSleepMode = 0;
      value = (float)robotGetButtonState();
      break;
    }
    case SOUND_SENSOR: {
      ROBOX_LOG("\n Read Sound sensor --");
      cntEnterSleepMode = 0;
      value = (float)robotGetSoundSensor();
      break;
    }
    case HEARTBIT: {
      ROBOX_LOG("\n Get HeartBit Request");
      value = (float)12345; 
      break;
    }    
    default:
      break;
  }
  if (stateBle) {
    val.floatVal = value;
    robotFeedback[2] = device;
    robotFeedback[3] = val.byteVal[0];
    robotFeedback[4] = val.byteVal[1];
    robotFeedback[5] = val.byteVal[2];
    robotFeedback[6] = val.byteVal[3];
  }
  else {
    writeHead();
    writeSerial(device);
    sendFloat(value);
  }
}

static void parseData(){
  isStart = false;
  int idx = readBuffer(3);
  command_index = (uint8_t)idx;
  int action = readBuffer(4);
  int device = readBuffer(5);
  ROBOX_LOG(" idx action device: %x %x %x", idx, action, device);
  switch(action){
    case GET:{
       readSensor(device);
       writeEnd();
    } 
     break;
     case RUN:{       
       cntEnterSleepMode = 0;
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
        ROBOX_LOG("\n Receive Start CMD -- ");
        break;
     }
     default:
       break;
  }
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
    if (sound_detect == 1) {
      delay(100);
      state++;  
    } 
  }
  for (int i = 0; i < NUM_LEDS; i++){
    leds[i] = CRGB::Black;
  }
  FastLED.show();
  Serial.println(state);
  switch (state) {
    case 1:
      DcMotorL.run(170, MOTOR2);
      DcMotorR.run(-170, MOTOR3);
      delay(1000);
      break;
    case 2:
      DcMotorL.run(-170, MOTOR2);
      DcMotorR.run(170, MOTOR3);
      delay(1000);
      break;
    case 3:
      DcMotorL.run(170, MOTOR2);
      DcMotorR.run(170, MOTOR3);
      delay(1000);
      break;
    case 4:  
      DcMotorL.run(-170, MOTOR2);
      DcMotorR.run(-170, MOTOR3);
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
      DcMotorL.run(-170, MOTOR2);
      DcMotorR.run(-150, MOTOR3);
  }
  else if ((LINE.readSensor1() == 1) && (LINE.readSensor2() == 0)) {
      //Serial.println("left");
      DcMotorL.run(160, MOTOR2);
      DcMotorR.run(140, MOTOR3);
  }
  else if ((LINE.readSensor1() == 0) && (LINE.readSensor2() == 1)) {
       //Serial.println("right");
      DcMotorL.run(-170, MOTOR2);
      DcMotorR.run(-140, MOTOR3);
  }
  else if ((LINE.readSensor1() == 1) && (LINE.readSensor2() == 1)) {
      //Serial.println("forward");
      DcMotorL.run(150, MOTOR2);
      DcMotorR.run(-140, MOTOR3);
  }
}

static void go_in_circle(void)
{
    if ((LINE.readSensor1() == 0) && (LINE.readSensor2() == 0)) {
      DcMotorL.run(140, MOTOR2);
      DcMotorR.run(-140, MOTOR3);
    }
    else if ((LINE.readSensor1() == 1) && (LINE.readSensor2() == 1)) {
      DcMotorL.run(-140, MOTOR2);
      DcMotorR.run(140, MOTOR3);
      delay(500);
      DcMotorL.run(-150, MOTOR2);
      DcMotorR.run(-170, MOTOR3);
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

static void go_demo_srf05(void)
{
    if (Ultra.distanceCm(200) >= 10) {
      DcMotorL.run(245, MOTOR2);
      DcMotorR.run(-245, MOTOR3);
    }
    else{
      DcMotorL.run(-245, MOTOR2);
      DcMotorR.run(255, MOTOR3);
      delay(500);
      DcMotorL.run(-255, MOTOR2);
      DcMotorR.run(-255, MOTOR3);
      delay(1000);
    }
}
/* Bluetooth Low Energy API for iOs */
/*---------------------------------------------------------------------------*/
class BleServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      uint8_t address[6] = {0xe0, 0x99, 0x71, 0xb6, 0x02, 0xc4};
      ROBOX_LOG("BLE Connected\n");
      stateConnected  = true;
    };

    void onDisconnect(BLEServer* pServer) {
      ROBOX_LOG("BLE Disconnected\n");
      stateConnected  = false;
    }
};

class BLERobotCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      uint8_t charBuf[64];  
      int length = 0;    
      std::string value = pCharacteristic->getValue();

      ROBOX_LOG("BLE write: %d", value);
      for (int i = 0; i < value.length(); i++) {
        charBuf[i] = value[i];
        ROBOX_LOG("%x ", charBuf[i]);
      }
      ROBOX_LOG("\n");
    if (value[0] == 0xff && value[1] == 0x55) {
        length = value[2];
        for (int index = 2; index < value.length(); index++) {
          writeBuffer(index, value[index]);
        }
        stateBle = true;
        parseData();
      }
    }

    void onRead(BLECharacteristic *pCharacteristic) {
        pCharacteristic->setValue(robotFeedback, 9);
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
  pCharacteristic = pService->createCharacteristic(
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
