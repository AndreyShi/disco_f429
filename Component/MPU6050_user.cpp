
#include "main.h"
#include <stdio.h>
#include "MPU6050_6Axis_MotionApps20.h"

MPU6050 mpu;
#define accelgyro mpu
#define M_PI           3.14159265358979323846  /* pi */
extern I2C_HandleTypeDef hi2c2;


uint16_t adc_buff[5][10];
uint16_t buf[500];
int buf_itera;
int ifg;
//#define OUTPUT_READABLE_QUATERNION
//#define OUTPUT_READABLE_EULER
#define OUTPUT_READABLE_YAWPITCHROLL
//#define OUTPUT_READABLE_REALACCEL
//#define OUTPUT_READABLE_WORLDACCEL
//#define OUTPUT_TEAPOT
#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}
   int16_t ax, ay, az;
   int16_t gx, gy, gz;
   int16_t offset = 0;
     uint32_t timer1ms;
  uint32_t timer1s;
  uint32_t timer1m;
    int16_t gxyz[3];//, gy, gz;
int checkBuffer(void)
{
  uint16_t summ = 0;
  for(int i = 0; i < packetSize;i++)
    { summ += fifoBuffer[i];}

  return summ;
}
void MPU6050_Initialize(void){
  ifg = 1;//timer7 ready
  //ResetDisplay();
  //DisplayON();
  // Turn off buffers, so I/O occurs immediately
  setvbuf(stdin, NULL, _IONBF, 0);  
  setvbuf(stdout, NULL, _IONBF, 0);
  setvbuf(stderr, NULL, _IONBF, 0);

  I2Cdev::init(&hi2c2);
  mpu.initialize();
  // verify connection
  Serial.println("Testing device connections...");

  while(1){
      bool tst = accelgyro.testConnection();
      Serial.println(tst ? "MPU6050 connection successful" : "MPU6050 connection failed");
      if(tst == false)
          { HAL_Delay(5000);}
      else
          {break;}
  }
  
  mpu.PrintActiveOffsets();
  #ifdef DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();
    mpu.PrintActiveOffsets();
  if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        //mpu.CalibrateAccel(6);//old 6
        //mpu.CalibrateGyro(6);//old 6
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
  #endif
}

void MPU6050_Process(void){
        mpu.resetFIFO();
        HAL_Delay(100);

        if (!dmpReady) {
            printf("dmp is not ready!\n");
            return;
        }
        mpuIntStatus = mpu.getIntStatus();
        fifoCount = mpu.getFIFOCount();
        #ifndef BUILD_LIB
        printf("Int: %d  Cnt:%d  ",mpuIntStatus, fifoCount);
        #endif

        if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
          mpu.resetFIFO();
          Serial.println(F("FIFO overflow!"));
        } else if (mpuIntStatus & 0x02 || mpuIntStatus & 0x01) {
          if(fifoCount < packetSize || fifoCount % packetSize) 
          {
            printf("bad packet, reset FIFO...!\n");
            return;
          }
          if(mpu.getFIFOBytes(fifoBuffer, packetSize) == 0)
          {
            printf("read fifo problem..!\n");
            return;
          }
          if(checkBuffer() == 0)
          {
            printf("fifo is zero..!\n");
            printf("setFIFOEnabled...\n");
            mpu.setFIFOEnabled(true);
            delay(50);
            return;
          }
          if(fifoBuffer[40] != 0xB2 && fifoBuffer[41] != 0x6A)
          {
            printf("wrong packet, reset FIFO...!\n");
            return;
          }

          fifoCount -= packetSize;
          mpu.dmpGetQuaternion(&q, fifoBuffer);
          mpu.dmpGetGravity(&gravity, &q);
          mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
          //printf("%2ld:%2ld:%3ld  ", timer1m,timer1s,timer1ms);
          #ifndef BUILD_LIB
          printf("ypr  %.3f %.3f %.3f", ypr[0] * 180/M_PI, ypr[1] * 180/M_PI, ypr[2] * 180/M_PI);
          #endif

          //AFS_SEL Full Scale Range LSB Sensitivity 0 ±2g 16384 LSB/g, 1 ±4g 8192 LSB/g, 2 ±8g 4096 LSB/g, 3 ±16g 2048 LSB/g
          mpu.dmpGetAccel(&aa, fifoBuffer);//for linear
          mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);//for linear
          #ifndef BUILD_LIB          
          printf("  acc %.3f %.3f %.3f  ",(float)aaReal.x/16384.0,(float)aaReal.y/16384.0,(float)aaReal.z/16384.0);// todo view LinearAccel on display  
          #endif

          //FS_SEL Full Scale Range LSB Sensitivity 0 ±250 °/s 131 LSB/°/s, 1 ±500 °/s65.5 LSB/°/s, 2 ±1000 °/s 32.8 LSB/°/s, 3 ±2000 °/s 16.4 LSB/°/s
          mpu.dmpGetGyro(gxyz, fifoBuffer);//for uglova9 speed
          #ifndef BUILD_LIB
          printf("spd %.3f %.3f %.3f",(float)gxyz[0]/16.4,(float)gxyz[1]/16.4,(float)gxyz[2]/16.4);//todo view uglova9 speed on display  
          printf("\n");
          #endif
        } else if (mpuIntStatus == 1 && fifoCount == 0) {
            printf("resetting DMP...\n");
            mpu.resetDMP();
            mpu.setDMPEnabled(false);
            delay(50);
            mpu.setDMPEnabled(true);
        } else
              { printf("skip...\n");}
    #ifdef RAW
     if(accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz) == -1)
         { printf("mpu.getMotion6 false\n");}
     else
         { printf("   %d %d %d %d %d %d\n",ax, ay, az, gx, gy, gz);}
     delay(100);
    #endif   
}