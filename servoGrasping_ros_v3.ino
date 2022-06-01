/* Modify History of servoGrasping_ros_v3
 *  1) copy from servoGrasping_ros_v2  
 *  2) issue: cannot autoGrasping  
 *      manualServoGrasping + rgb_pot + forceSensor (work)
 *      autoServoGrasping + rgb_pot + forceSensor (doesn't work)
 *      --> connect to real robot, and then try again.
 *  
 *  writye by AHC
 *----------------------------------------------------------------------------------*/

// ******************************** LIBRARIES ******************************** //
#include <Wire.h>               //for Color.Sensor I²C communication.
#include <Adafruit_TCS34725.h>  //Color.Sensor.Board
#include <FastLED.h>            //LED Strip
#include <Servo.h>              //for servo
#include <SPI.h>                //for Force.Sensor SPI communication.
#include <math.h>
#include <ros.h>                              //for rosserial
#include <std_msgs/Int16MultiArray.h>         //for colorSensor-ros
#include <std_msgs/UInt16.h>                  //for forceSensor-ros and manual-grasping
#include <geometry_msgs/PoseStamped.h>        //for auto-grasping

// ****************************** COLOR & LED.Strip SETUP ****************************** //
#define LED_TYPE    WS2812B   // type of RGB LEDs 
#define COLOR_ORDER GRB       // sequence of colors in data stream
#define NUM_LEDS    20        // 60 LEDs numbered [0..59]
#define DATA_PIN    6         // LED data pin
#define BRIGHTNESS  150       // brightness range [off..on] = [0..255]
CRGB leds[NUM_LEDS];  // Define the array of RGB control data for each LED
Adafruit_TCS34725 color = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_700MS, TCS34725_GAIN_1X);

// ******************************* SERVO SETUP ******************************** //
#define servoPin  7
#define servoFeedbackPin  A0
Servo HS645MG;

// ****************************** FORCE SENSOR SETUP ****************************** //
#define finger1Pin  10  //int Finger1 = 10;  
#define finger2Pin  9   //int Finger2 = 9;
#define finger3Pin  8   //int Finger3 = 8;
#define MOSIpin   11    //int PinMOSI = 11;
#define MISOpin   12    //int PinMISO = 12;
#define SCKpin    13    //int PinSCK = 13;
int weight;
int avgFinger1, avgFinger2, avgFinger3;
SPISettings settingsA(SPI_CLOCK_DIV32, MSBFIRST, SPI_MODE1); //18000000 MODE1

// ************************** creat ROS handle, msg, and pub/sub ******************************* //
ros::NodeHandle  nh;

//for colorSensor_publish
std_msgs::Int16MultiArray rgb;
ros::Publisher rgb_pub("rgb_pot", &rgb);
unsigned long start;

//for forceSensor_publish
std_msgs::UInt16 finger1;
std_msgs::UInt16 finger2;
std_msgs::UInt16 finger3;
ros::Publisher finger1_pub("weight_finger1", &finger1);
ros::Publisher finger2_pub("weight_finger2", &finger2);
ros::Publisher finger3_pub("weight_finger3", &finger3);

////for servoGrasping_subscribe
//void servo_cb(const geometry_msgs::PoseStamped& msgPose)  {
//  float pos = msgPose.pose.position.z;
//  if (pos <= 0.015)  {
////    delay(3000);
//    HS645MG.write(30);
//  }
//  else  {
//    HS645MG.write(0);
//  }
//}
//ros::Subscriber<geometry_msgs::PoseStamped> haha_sub("j2n6s300_driver/out/tool_pose", servo_cb);



//for servo_subscribe
void servo_cb( const std_msgs::UInt16& cmd_msg){
  HS645MG.write(cmd_msg.data);
}
ros::Subscriber<std_msgs::UInt16> haha_sub("servo_HS645MG", servo_cb);





// ****************************** MAIN SETUP ****************************** //
void setup() {
  Serial.begin(115200); //57600//74880//115200
  nh.getHardware()->setBaud(115200);
  
  //for ledStrip
//  FastLED.addLeds<LED_TYPE, DATA_PIN, COLOR_ORDER>(leds, NUM_LEDS);


  //for colorSensor_publish
  color.begin();
  rgb.layout.dim = (std_msgs::MultiArrayDimension *)
  malloc(sizeof(std_msgs::MultiArrayDimension) * 2);
  rgb.data_length = 4;
  rgb.layout.dim[0].label = "rgb";
  rgb.layout.dim[0].size = 4;
  rgb.layout.dim[0].stride = 1*4;
  rgb.layout.data_offset = 0;
  rgb.data = (int *)malloc(sizeof(int)*4);
  

  //for forceSensor_publish
  pinMode (MOSIpin, OUTPUT);
  pinMode (MISOpin, INPUT);
  pinMode (SCKpin, OUTPUT);
  pinMode (finger1Pin, OUTPUT);
  pinMode (finger2Pin, OUTPUT);
  pinMode (finger3Pin, OUTPUT);
  digitalWrite(finger1Pin, HIGH);   //digitalWrite means OUTPUT.
  digitalWrite(finger2Pin, HIGH);
  digitalWrite(finger3Pin, HIGH);
  SPI.beginTransaction(settingsA);
  avgFinger1 = avgBnorm(finger1Pin);
  avgFinger2 = avgBnorm(finger2Pin);
  avgFinger3 = avgBnorm(finger3Pin);


  //for servo_subscribe
  HS645MG.attach(servoPin);
  HS645MG.writeMicroseconds(553);


  //creating node
  nh.initNode(); 
  nh.advertise(rgb_pub);
  nh.advertise(finger1_pub);
  nh.advertise(finger2_pub);
  nh.advertise(finger3_pub);
  nh.subscribe(haha_sub);
}

// ****************************** MAIN LOOP ****************************** //
void loop() {
  colorSensor();
  sensorRead(finger1Pin); 
  sensorRead(finger2Pin); 
  sensorRead(finger3Pin);
  
  nh.spinOnce();
  delay(1);
}

// ****************************** CALL FUNCTIONS() ****************************** //
void colorSensor() {
  uint16_t red, green, blue, clearSensor;
  getRawData_noDelay(&red, &green, &blue, &clearSensor);
  uint16_t servoFeedbackVal = analogRead(A0);
  
  float sum = red;
  sum += green;
  sum += blue;
  sum = clearSensor;
  float r = red; r /= sum;
  float g = green; g /= sum;
  float b = blue; b /= sum;
  r *= 256; g *= 256; b *= 256;
  Serial.print(r);  Serial.print("\t");
  Serial.print(g);  Serial.print("\t");
  Serial.print(b);  Serial.print("\n");
  rgb.data[0] = r;
  rgb.data[1] = g;
  rgb.data[2] = b;
  rgb.data[3] = servoFeedbackVal;
  rgb_pub.publish( &rgb);
  
//  fill_solid( &(leds[0]), 10, CRGB(r, g, b));
//  FastLED.setBrightness(BRIGHTNESS);  
//  FastLED.show();
}

void getRawData_noDelay(uint16_t *red, uint16_t *green, uint16_t *blue, uint16_t *clearSensor)  {
  // getRawData() would cause a delay of the duration of the selected
  // integration time, thus blocking loop() like a delay() in the code;
  // but using the ISR (= interrupt(break)) method, there is no more delay,
  // because one receives an interrupt once the integration is done and
  // loop() is no longer waiting for the sensor (* creates a pointer to
  // the memory address of a variable)
  *clearSensor = color.read16(TCS34725_CDATAL);
  *red = color.read16(TCS34725_RDATAL); 
  *green = color.read16(TCS34725_GDATAL); 
  *blue = color.read16(TCS34725_BDATAL);  
}

int sensorRead(int slavePin)  {
  sensorRead(slavePin, false);
}

int sensorRead (int slavePin, bool calibration) {
  uint8_t readBuffer[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  uint8_t writeBuffer[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  uint8_t errorBits, CRC, rollingCounter;
  int16_t Bx, By, Bz;
  int Bnorm;

  digitalWrite(slavePin, LOW);
  writeBuffer[0] = 0x00;
  writeBuffer[1] = 0x00;
  writeBuffer[2] = 0xFF; // Timeout value is set as 65 ms
  writeBuffer[3] = 0xFF; // Timeout value is set as 65 ms
  writeBuffer[4] = 0x00;
  writeBuffer[5] = 0x00;
  writeBuffer[6] = 0x93; // Marker is set as 2 to get XYZ measurement. OP Code for GET1 message: 19 in Decimal.
  writeBuffer[7] = ComputeCRC(0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x93); // CRC
  delayMicroseconds(1);
  for (int i = 0; i < 8; i++) {
    readBuffer[i] = SPI.transfer(writeBuffer[i]);
  }
  delayMicroseconds(1500);
  digitalWrite(slavePin, HIGH);

  Bx = (readBuffer[1] & 0x3F) << 8;
  Bx += readBuffer[0];
  if (Bx >= 8192) {
    Bx -= 16384;
  }
  By = (readBuffer[3] & 0x3F) << 8;
  By += readBuffer[2];
  if (By >= 8192) {
    By -= 16384;
  }
  Bz = (readBuffer[5] & 0x3F) << 8;
  Bz += readBuffer[4];
  if (Bz >= 8192) {
    Bz -= 16384;
  }

  errorBits = readBuffer[0] >> 6;
  CRC = readBuffer[7];
  rollingCounter = readBuffer[6] & 0x3F;

  Bnorm = sqrt( (double)Bx * (double)Bx + (double)By * (double)By + (double)Bz * (double)Bz );  //Serial.print(Bnorm);  Serial.print("\t");
  weight = 5.44*Bnorm + 17.8;  //Serial.print("weight.raw="); Serial.print(weight);  Serial.print("\t"); 

  if (calibration == false & slavePin==finger1Pin) { 
    weight = weight - avgFinger1; //Serial.print(weight);  Serial.print("\t");  //Serial.print("weight.cali=");  
    EPfliter(finger1Pin); 
    finger1.data = weight;
    finger1_pub.publish(&finger1); 
  }
  if (calibration == false & slavePin==finger2Pin) { 
    weight = weight - avgFinger2; //Serial.print(Bnorm);  Serial.print("\t");
    EPfliter(finger2Pin);
    finger2.data = weight;
    finger2_pub.publish(&finger2);
  }
  if (calibration == false & slavePin==finger3Pin) { 
    weight = weight - avgFinger3;  //Serial.print(Bnorm);  Serial.print("\t");
    EPfliter(finger3Pin);
    finger3.data = weight; 
    finger3_pub.publish(&finger3);
  }
}

int avgBnorm(int slavePin)  {
  static int readings[6];
  static int average, sum; 
  for (int i=0; i<6; i++) {
    sensorRead(slavePin, true);
    readings[i] = weight;  //Serial.print("array= ");  Serial.print(readings[i]); Serial.print("\t");
    delay(1);
  }
  sum = readings[3]+readings[4]+readings[5];  //Serial.print("sum= ");  Serial.print(sum); Serial.print("\t");
  average = sum/3.0;  //Serial.print("avg= ");  Serial.print(average); Serial.print("\t");
  return average;
}

uint8_t ComputeCRC(uint8_t Byte0, uint8_t Byte1, uint8_t Byte2, uint8_t Byte3, uint8_t Byte4, uint8_t Byte5, uint8_t Byte6) {
  uint8_t CRC = 0xFF;
  char CRCArray[] = {
  0x00, 0x2F, 0x5E, 0x71, 0xBC, 0x93, 0xE2, 0xCD, 0x57, 0x78, 0x09, 0x26,
  0xEB, 0xC4, 0xB5, 0x9A, 0xAE, 0x81, 0xF0, 0xDF, 0x12, 0x3D, 0x4C, 0x63,
  0xF9, 0xD6, 0xA7, 0x88, 0x45, 0x6A, 0x1B, 0x34, 0x73, 0x5C, 0x2D, 0x02,
  0xCF, 0xE0, 0x91, 0xBE, 0x24, 0x0B, 0x7A, 0x55, 0x98, 0xB7, 0xC6, 0xE9,
  0xDD, 0xF2, 0x83, 0xAC, 0x61, 0x4E, 0x3F, 0x10, 0x8A, 0xA5, 0xD4, 0xFB,
  0x36, 0x19, 0x68, 0x47, 0xE6, 0xC9, 0xB8, 0x97, 0x5A, 0x75, 0x04, 0x2B,
  0xB1, 0x9E, 0xEF, 0xC0, 0x0D, 0x22, 0x53, 0x7C, 0x48, 0x67, 0x16, 0x39,
  0xF4, 0xDB, 0xAA, 0x85, 0x1F, 0x30, 0x41, 0x6E, 0xA3, 0x8C, 0xFD, 0xD2,
  0x95, 0xBA, 0xCB, 0xE4, 0x29, 0x06, 0x77, 0x58, 0xC2, 0xED, 0x9C, 0xB3,
  0x7E, 0x51, 0x20, 0x0F, 0x3B, 0x14, 0x65, 0x4A, 0x87, 0xA8, 0xD9, 0xF6,
  0x6C, 0x43, 0x32, 0x1D, 0xD0, 0xFF, 0x8E, 0xA1, 0xE3, 0xCC, 0xBD, 0x92,
  0x5F, 0x70, 0x01, 0x2E, 0xB4, 0x9B, 0xEA, 0xC5, 0x08, 0x27, 0x56, 0x79,
  0x4D, 0x62, 0x13, 0x3C, 0xF1, 0xDE, 0xAF, 0x80, 0x1A, 0x35, 0x44, 0x6B,
  0xA6, 0x89, 0xF8, 0xD7, 0x90, 0xBF, 0xCE, 0xE1, 0x2C, 0x03, 0x72, 0x5D,
  0xC7, 0xE8, 0x99, 0xB6, 0x7B, 0x54, 0x25, 0x0A, 0x3E, 0x11, 0x60, 0x4F,
  0x82, 0xAD, 0xDC, 0xF3, 0x69, 0x46, 0x37, 0x18, 0xD5, 0xFA, 0x8B, 0xA4,
  0x05, 0x2A, 0x5B, 0x74, 0xB9, 0x96, 0xE7, 0xC8, 0x52, 0x7D, 0x0C, 0x23,
  0xEE, 0xC1, 0xB0, 0x9F, 0xAB, 0x84, 0xF5, 0xDA, 0x17, 0x38, 0x49, 0x66,
  0xFC, 0xD3, 0xA2, 0x8D, 0x40, 0x6F, 0x1E, 0x31, 0x76, 0x59, 0x28, 0x07,
  0xCA, 0xE5, 0x94, 0xBB, 0x21, 0x0E, 0x7F, 0x50, 0x9D, 0xB2, 0xC3, 0xEC,
  0xD8, 0xF7, 0x86, 0xA9, 0x64, 0x4B, 0x3A, 0x15, 0x8F, 0xA0, 0xD1, 0xFE,
  0x33, 0x1C, 0x6D, 0x42
  };

  CRC = CRCArray[CRC ^ Byte0];
  CRC = CRCArray[CRC ^ Byte1];
  CRC = CRCArray[CRC ^ Byte2];
  CRC = CRCArray[CRC ^ Byte3];
  CRC = CRCArray[CRC ^ Byte4];
  CRC = CRCArray[CRC ^ Byte5];
  CRC = CRCArray[CRC ^ Byte6];
  CRC = ~CRC;
  return CRC;
}

void EPfliter(int slavePin)  {
  if (slavePin == finger1Pin)  {
    double alp = 0.3;
    static int weight1;
    weight = alp * weight + (1 - alp) * weight1;  Serial.print(weight);  Serial.print("\t");//BTSerial.print(weight);    //Serial.print("weight.filter.finger1Pin=");   
    weight1 = weight; 
  }
  if (slavePin == finger2Pin)  {
    double alp = 0.3;
    static int weight1;
    weight = alp * weight + (1 - alp) * weight1;  Serial.print(weight);  Serial.print("\t");//BTSerial.print(weight);  Serial.print("\t");  //Serial.print("weight.filter.finger2Pin=");  
    weight1 = weight; 
  }
  if (slavePin == finger3Pin)  {
    double alp = 0.3;
    static int weight1;
    weight = alp * weight + (1 - alp) * weight1;  Serial.print(weight); Serial.print("\n");//BTSerial.print(weight);  Serial.print("\t");  //Serial.print("weight.filter.finger3Pin=");  
    weight1 = weight; 
  }
}









//
