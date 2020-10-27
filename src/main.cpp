#include <Arduino.h>

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include <PID_v1.h>
#include <Servo.h>

Servo x_Servo;
Servo y_Servo;
Servo Parachute_Servo;

MPU6050 mpu;

//Define Variables we'll be connecting to
double xSetpoint, xInput, xOutput;
double ySetpoint, yInput, yOutput;


//Specify the links and initial tuning parameters
double xKp=0.6, xKi=0.1, xKd=0.1;
double yKp=0.6, yKi=0.1, yKd=0.1;

double i = 0;
PID xPID(&xInput, &xOutput, &xSetpoint, xKp, xKi, xKd, DIRECT);
PID yPID(&yInput, &yOutput, &ySetpoint, yKp, yKi, yKd, DIRECT);

#define x_Obergrenze   144
#define x_Untergrenze  10
#define x_Nullstellung 67

#define y_Obergrenze   89
#define y_Untergrenze  25
#define y_Nullstellung 61

#define Parachute_Nullstellung 30
#define Parachute_Ausgefahren 120

#define OUTPUT_READABLE_YAWPITCHROLL


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



// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

// ================================================================
// ===               VOID UPSETTING                             ===
// ================================================================
void Telemetry();
void TVC_test();
void PID_Initialisieren();
void Attach_Servo();
void TVC_steering();
void Parachute_deploying();
void Parachute_Servo_Test();
// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
    Serial.begin(115200);
    
    Telemetry();
    Attach_Servo();
    Parachute_Servo_Test();
    TVC_test();
    PID_Initialisieren();



    // join I2C bus (I2Cdev library doesn't do this automatically)

    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties


    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // initialize device

    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
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

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);

  Serial.println("SYSTEM: READY FOR TAKE OFF");  
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        if (mpuInterrupt && fifoCount < packetSize) {
          // try to get out of the infinite loop
          fifoCount = mpu.getFIFOCount();
        }

      TVC_steering();
      // other program behavior stuff here
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();
    if(fifoCount < packetSize){
            //Lets go back and wait for another interrupt. We shouldn't be here, we got an interrupt from another event
        // This is blocking so don't do it   while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    }
    // check for overflow (this should never happen unless our code is too inefficient)
    else if ((mpuIntStatus & (0x01 << MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
      //  fifoCount = mpu.getFIFOCount();  // will be zero after reset no need to ask
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & (0x01 << MPU6050_INTERRUPT_DMP_INT_BIT)) {

        // read a packet from FIFO
        while(fifoCount >= packetSize){ // Lets catch up to NOW, someone is using the dreaded delay()!
          mpu.getFIFOBytes(fifoBuffer, packetSize);
          // track FIFO count here in case there is > 1 packet available
          // (this lets us immediately read more without waiting for an interrupt)
          fifoCount -= packetSize;
        }

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            /*Serial.print("ypr\t");
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(ypr[2] * 180/M_PI);*/

        #endif


        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }
}

void TVC_test(){
  Serial.println(F("Testing the TVC-Mount..."));
  y_Servo.write(y_Nullstellung);
  x_Servo.write(x_Nullstellung);
  delay(100);
  y_Servo.write(y_Nullstellung);
  x_Servo.write(x_Obergrenze);
  delay(500);
  y_Servo.write(y_Nullstellung);
  x_Servo.write(x_Untergrenze);
  delay(500);
  x_Servo.write(x_Nullstellung);
  y_Servo.write(y_Obergrenze);
  delay(500);
  x_Servo.write(x_Nullstellung);
  y_Servo.write(y_Untergrenze);
  delay(500);
  x_Servo.write(x_Nullstellung);
  y_Servo.write(y_Nullstellung);
  delay(500);
  Serial.println(F("SYSTEM CHECK: OK"));
}

void PID_Initialisieren(){
    Serial.println(F("PID initialising..."));
    xInput = ypr[2] * 180/M_PI;
    yInput = ypr[1] * 180/M_PI;
    xSetpoint = 0;
    ySetpoint = 0;
    xPID.SetMode(AUTOMATIC);
    yPID.SetMode(AUTOMATIC);
    xPID.SetOutputLimits(x_Untergrenze-x_Nullstellung, x_Obergrenze-x_Nullstellung);
    yPID.SetOutputLimits(y_Untergrenze-y_Nullstellung, y_Obergrenze-y_Nullstellung);
    xPID.SetSampleTime(20);
    yPID.SetSampleTime(20);
    Serial.println(F("SYSTEM CHECK: OK"));
}

void Attach_Servo(){
  Serial.println(F("Attaching Servos..."));
  x_Servo.attach(4);
  y_Servo.attach(5);
  Parachute_Servo.attach(3);
  Serial.println(F("SYSTEM CHECK: OK"));
}

void Parachute_Servo_Test(){
  Serial.println(F("Testing the parachute servo..."));
 /* Parachute_Servo.write(Parachute_Nullstellung);
  delay(500);
  Parachute_Servo.write(Parachute_Ausgefahren);
  delay(500);
  Parachute_Servo.write(Parachute_Nullstellung);
  delay(500);
  Parachute_Servo.detach();
  Serial.println(F("SYSTEM CHECK: OK"));
*/
}

void TVC_steering(){
    xInput = ypr[2] * 180/M_PI;
    yInput = ypr[1] * 180/M_PI;
    xPID.Compute();
    yPID.Compute();
    Serial.print("xOutput: ");
    Serial.print(xOutput);
    Serial.print("\t");
    Serial.print("yOutput: ");
    Serial.print(yOutput);
    Serial.print("\t");
    Serial.print("y/p: ");
    Serial.print(yInput);
    Serial.print("\t");
    Serial.print("x/r: ");
    Serial.println(xInput);

    x_Servo.write(xOutput +x_Nullstellung);
    y_Servo.write(yOutput +y_Nullstellung);
}

void Parachute_deploying(){
  Parachute_Servo.attach(3);
  Parachute_Servo.write(Parachute_Ausgefahren);
  delay(500);
}

void Telemetry(){
  double gravityacc = 9.81;
  double mass = 0.550;
  double time = 2.1;
  double downforce = gravityacc*mass;
  double upforce = 9-downforce;
  double upacc = upforce/mass; 
  double height = 0.5*upacc*pow(time,2);
  double maxspeed = upacc*time;

  Serial.println("Telemetry Data...");
  delay(50);
  Serial.println("Groundacceleration in m/s^2");
  Serial.println(gravityacc);
  Serial.println("Mass in kg");
  Serial.println(mass);
  Serial.println("Downforce in N");
  Serial.println(downforce);
  Serial.println("Upforce in N");
  Serial.println(upforce);
  Serial.println("Upacceleration in m/s^2");
  Serial.println(upacc);
  Serial.println("Height in m");
  Serial.println(height);
  Serial.println("Max. Speed in m/s");
  Serial.println(maxspeed);
  delay(1000);

  if(height > 15){
      Serial.println(F("SYSTEM CHECK: OK"));
  }else{
      Serial.println(F("SYSTEM CHECK: PROBLEM: HEIGHT BELOW 15M"));
  }
}