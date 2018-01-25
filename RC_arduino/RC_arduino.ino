/******************************************************************************
  CSV_Logger_TinyGPSPlus.ino
  Log GPS data to a CSV file on a uSD card
  By Jim Lindblom @ SparkFun Electronics
  February 9, 2016
  https://github.com/sparkfun/GPS_Shield

  This example uses SoftwareSerial to communicate with the GPS module on
  pins 8 and 9, then communicates over SPI to log that data to a uSD card.

  It uses the TinyGPS++ library to parse the NMEA strings sent by the GPS module,
  and prints interesting GPS information - comma separated - to a newly created
  file on the SD card.
 
  Resources:
  TinyGPS++ Library  - https://github.com/mikalhart/TinyGPSPlus/releases
  SD Library (Built-in)
  SoftwareSerial Library (Built-in)

//9DOF IMU 
  The LSM9DS1 is a versatile 9DOF sensor. It has a built-in
  accelerometer, gyroscope, and magnetometer. Very cool! Plus it
  functions over either SPI or I2C.

  Hardware setup: This library supports communicating with the
  LSM9DS1 over either I2C or SPI. This example demonstrates how
  to use I2C. The pin-out is as follows:
  LSM9DS1 --------- Arduino
   SCL ---------- SCL (A5 on older 'Duinos')
   SDA ---------- SDA (A4 on older 'Duinos')
   VDD ------------- 3.3V
   GND ------------- GND
  (CSG, CSXM, SDOG, and SDOXM should all be pulled high. 
  Jumpers on the breakout board will do this for you.)

  The LSM9DS1 has a maximum voltage of 3.6V. Make sure you power it
  off the 3.3V rail! I2C pins are open-drain, so you'll be 
  (mostly) safe connecting the LSM9DS1's SCL and SDA pins 
  directly to the Arduino.
  
 # the DC Motor Driver 2x15A_lite module.
 # Product: DC Motor Driver 2x15A_lite
 # SKU    : DRI0018
   
 #Steps:
 1.Connect the M1_PWM & M2_PWM to UNO digital 5 & 6
 2.Connect the M1_EN & M2_EN to UNO digital 4 & 7
 3.Connect +5V & GND to UNO 5V & GND
 
 # Function for current sense and diagnosis,if you want to use
 please connect the IS pins to Arduino
 Connect LA_IS & RA_IS to Arduino digital 2
 Connect LB_IS & RB_IS to Arduino digital 3
 */
******************************************************************************/
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <TinyGPS++.h>
#include <SparkFunLSM9DS1.h>

class Farmbot
{
  public:
    Oriantation();
    //Initial condition
    float initRoll;
    float initPitch;
    float initHeading;
    float initLat;
    float initLon;
    
    //Heading,Pitch,Roll in Degree
    float roll_deg;
    float pitch_deg;
    float heading_deg;
  
    //Heading,Pitch,Roll in Radian
    float roll;
    float pitch;
    float heading;

    // Acc Gyro Mag
    float ax;
    float ay;
    float az;
    float gx;
    float gy;
    float gz;
    float mx;
    float my;    
    float mz;
  protected:
  private:  
}
Farmbot::Farmbot()
{
    initRoll = 0.0;
    initPitch = 0.0;
    initHeading = 0.0;
    roll_deg = 0.0;
    pitch_deg = 0.0;
    heading_deg = 0.0;
    roll = 0.0;
    pitch = 0.0;
    heading = 0.0;
    ax = 0.0;
    ay = 0.0;
    az = 0.0;
    gx = 0.0;
    gy = 0.0;
    gz = 0.0;
    mx = 0.0;
    my = 0.0;    
    mz = 0.0;
}
Farmbot Bot;
//Status of Arduino
static int farmbotStatus = 0;
#define SENSOR_INITIALIZATION       1
#define ROBOT_INITIAL_ORIANTATION   2
#define ADVANCE                     3
#define COLLECTING_DATA             4
#define WEEDING                     5

// index for calculating the initial oriantation
static unsigned long iInitialOriantation = 0;
static unsigned long iInitialGPS = 0;  
static unsigned long iAdvance = 0;
static unsigned long iCollecting = 0;  
#define ARDUINO_USD_CS 10 // uSD card CS pin (pin 10 on SparkFun GPS Logger Shield)

//////////////////////////
// LSM9DS1 Library Init //
//////////////////////////
// Use the LSM9DS1 class to create an object. [imu] can be
// named anything, we'll refer to that throught the sketch.
LSM9DS1 imu;
///////////////////////
// Example I2C Setup //
///////////////////////
// SDO_XM and SDO_G are both pulled high, so our addresses are:
#define LSM9DS1_M    0x1E // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG  0x6B // Would be 0x6A if SDO_AG is LOW

////////////////////////////
// Sketch Output Settings //
////////////////////////////
//#define PRINT_CALCULATED
#define PRINT_RAW
#define PRINT_SPEED 50  // 50ms -> 20hz update rate


#define MINUTE_INITIAL_ORIANTATION 5   // 5minute
#define MSECONDE_INITIAL_ORIANTATION MINUTE_INITIAL_ORIANTATION *60000 //ms of TIME_MINUTE
#define MAXINDEX_INITIAL_ORIANTATION MSECONDE_INITIAL_ORIANTATION/PRINT_SPEED

#define MSECONDE_ADVANCE 60000 //ms
#define MAXINDEX_ADVANCE MSECONDE_ADVANCE/PRINT_SPEED

#define MINUTE_COLLECTING 5   // 5minute
#define MSECONDE_COLLECTING MINUTE_COLLECTING*60000 //ms
#define MAXINDEX_COLLECTING MSECONDE_COLLECTING/PRINT_SPEED

static unsigned long lastPrint = 0; // Keep track of print time

// Earth's magnetic field varies by location. Add or subtract
// a declination to get a more accurate heading. Calculate
// your's here:
// http://www.ngdc.noaa.gov/geomag-web/#declination
#define DECLINATION -11.41 //Bundoora 14/11/2017 //-8.58 Declination (degrees) in Boulder, CO.

/////////////////////////
// Log File Defintions //
/////////////////////////
// Keep in mind, the SD library has max file name lengths of 8.3 - 8 char prefix,
// and a 3 char suffix.
// Our log files are called "gpslogXX.csv, so "gpslog99.csv" is our max file.
#define LOG_FILE_PREFIX "imulog" // Name of the log file.
#define MAX_LOG_FILES 100 // Number of log files that can be made
#define LOG_FILE_SUFFIX "csv" // Suffix of the log file
char logFileName[13]; // Char string to store the log file name
// Data to be logged:
#define LOG_COLUMN_COUNT 9
char * log_col_names[LOG_COLUMN_COUNT] = {
  "ax(g)","ay(g)","az(g)","gx(deg/s)","gy(deg/s)","gz(deg/s)","mx(gauss)","my(gauss)","mz(gauss)",
}; // log_col_names is printed at the top of the file.


// Offsets applied to raw x/y/z values
float mag_offsets[3]            = { 37793.5207402, -6321.2426109, 6891.2431500 };

// Soft iron error compensation matrix
float mag_softiron_matrix[3][3] = { { 0.97196, 0.00833, 0.00444 },
                                    { 0.00833, 1.04714, -0.00693 },
                                    { 0.00444, -0.00693, 1.00345 } }; 

/////////////////////////
// TinyGPS Definitions //
/////////////////////////
TinyGPSPlus tinyGPS; // tinyGPSPlus object to be used throughout
#define GPS_BAUD 9600 // GPS module's default baud rate

/////////////////////////////////
// GPS Serial Port Definitions //
/////////////////////////////////
// If you're using an Arduino Uno, Mega, RedBoard, or any board that uses the
// 0/1 UART for programming/Serial monitor-ing, use SoftwareSerial:
// Set gpsPort to either ssGPS if using SoftwareSerial or Serial1 if using an
// Arduino with a dedicated hardware serial port
#define gpsPort Serial1  // Alternatively, use Serial1 on the Leonardo

// Define the serial monitor port. On the Uno, Mega, and Leonardo this is 'Serial'
//  on other boards this may be 'SerialUSB'
#define SerialMonitor Serial


/////////////////////////////////
// Motor Driver                //
/////////////////////////////////
int E1 = 5;     //M1 Speed Control
int E2 = 6;     //M2 Speed Control
int M1 = 4;     //M1 Direction Control
int M2 = 7;     //M1 Direction Control
int counter=0;

void setup()
{
  Serial.begin(115200);
  gpsPort.begin(GPS_BAUD);

  Serial.println("Setting up SD card.");
  // see if the card is present and can be initialized:
  if (!SD.begin(ARDUINO_USD_CS))
  {
    Serial.println("Error initializing SD card.");
  }
  updateFileName(); // Each time we start, create a new file, increment the number
  printHeader(); // Print a header at the top of the new file
  // Before initializing the IMU, there are a few settings
  // we may need to adjust. Use the settings struct to set
  // the device's communication mode and addresses:
  imu.settings.device.commInterface = IMU_MODE_I2C;
  imu.settings.device.mAddress = LSM9DS1_M;
  imu.settings.device.agAddress = LSM9DS1_AG;
  
  // The above lines will only take effect AFTER calling
  // imu.begin(), which verifies communication with the IMU
  // and turns it on.
  if (!imu.begin())
  {
    Serial.println("Failed to communicate with LSM9DS1.");
    while (1);
  }
  // Motor Driver setup
  int i;
  for(i=4;i<=7;i++)
  pinMode(i, OUTPUT);  
  digitalWrite(E1,LOW);   
  digitalWrite(E2,LOW); 
  pinMode(2,INPUT);
  pinMode(3,INPUT);
}

void loop()
{
  int i;
  if ((lastLog + LOG_RATE) <= millis())
  { // If it's been LOG_RATE milliseconds since the last log:
     switch(farmbotStatus){
      case SENSOR_INITIALIZATION:
        if (gpsPort.available()) // When GPS start available go to the next step
           farmbotStatus = ROBOT_INITIAL_ORIANTATION;      
      break;
      case ROBOT_INITIAL_ORIANTATION:
          if(MAXINDEX_INITIAL_ORIANTATION < iInitialOriantation++)
          { 
            // Calculate initial oriatation by averaging IMU and GPS
            // recusrive mean calculation
            InitialOriantationUpdate(iInitialOriantation);
            if(tinyGPS.location.isUpdated())
            {
             InitialGPSUpdate(++iInitialGPS);
            }
          }
          else{
            // After calculting initial oriantation move the next step
            farmbotStatus = ADVANCE;
            iInitialOriantation = 0;
            iInitialGPS =0;
          }        
      break;
      case ADVANCE:
          if(MAXINDEX_ADVANCE < iAdvance++)
          { 
              motoradvance (255,255);   //move forward in max speed
          }
          else{
             farmbotStatus = COLLECTING_DATA;
             iAdvance =0;
          }
      break;     
      case COLLECTING_DATA:
          if(MAXINDEX_COLLECTING < iCollecting++)
          { 
              logFrambot();
              motorstop();
          }
          else{
             farmbotStatus = WEEDING;
             iCollecting = 0;
          }
      break;
      case WEEDING:
          // weeding task
          // need to code
      break;     
     }
     lastLog = millis(); // Update the lastLog variable
  }    
    // If we're not logging, continue to "feed" the tinyGPS object:
  while (gpsPort.available())
    tinyGPS.encode(gpsPort.read());
}

void updateFrambot()
{
  updateIMU();
  Bot.ax = imu.calcAccel(imu.ax);
  Bot.ay = imu.calcAccel(imu.ay);
  Bot.az = imu.calcAccel(imu.az);
  Bot.gx = imu.calcGyro(imu.gx);
  Bot.gy = imu.calcGyro(imu.gy);
  Bot.gz = imu.calcGyro(imu.gz);
  Bot.mx = imu.calcMag(imu.mx)*100000;
  Bot.my = imu.calcMag(imu.my)*100000;
  Bot.mz = imu.calcMag(imu.mz)*100000;
}
void updateOriantation()
{
  updateFrambot(); // update the current measurement of sensors on the Farmbot
  
  float x =  Bot.mx - mag_offsets[0];
  float y =  Bot.my - mag_offsets[1];
  float z =  Bot.mz - mag_offsets[2];
  
  // Apply mag soft iron error compensation
  float mx = x * mag_softiron_matrix[0][0] + y * mag_softiron_matrix[0][1] + z * mag_softiron_matrix[0][2];
  float my = x * mag_softiron_matrix[1][0] + y * mag_softiron_matrix[1][1] + z * mag_softiron_matrix[1][2];
  float mz = x * mag_softiron_matrix[2][0] + y * mag_softiron_matrix[2][1] + z * mag_softiron_matrix[2][2];

  // Pitch and roll
  Bot.roll  = atan2(Bot.ay, Bot.az);
  Bot.pitch = atan2(-Bot.ax, Bot.ay*sin(Bot.roll)+Bot.az*cos(Bot.roll));
  Bot.roll_deg = Bot.roll*180.0/M_PI;
  Bot.pitch_deg = Bot.pitch*180.0/M_PI;
  
  // Tilt compensated magnetic sensor measurements
  float mx_comp = mx*cos(Bot.pitch)+my*sin(Bot.pitch)*sin(Bot.roll)+mz*sin(Bot.pitch)*cos(Bot.roll);
  float my_comp = mz*sin(Bot.roll)-my*cos(Bot.roll);
  
  // Arctangent of y/x
  Bot.heading = atan2(my_comp,mx_comp);
  Bot.heading_deg = Bot.heading*180.0/M_PI;
  if (heading < 0)
  Bot.heading_deg += 360;
}

void InitialOriantationUpdate(int n)
{
  updateOriantation();
  Bot.initRoll  = ((Bot.initRoll * n) + Bot.roll) / (n+1);
  Bot.initPitch = ((Bot.initPitch * n) + Bot.pitch) / (n+1);
  Bot.initHeading = ((Bot.initHeading * n) + Bot.heading) / (n+1);
}

void InitialGPSUpdate(int n)
{
  Bot.initLat  = ((Bot.initLat * n) + tinyGPS.location.lat()) / (n+1);
  Bot.initLon  = ((Bot.initLon * n) + tinyGPS.location.lng()) / (n+1);
}

int updateIMU()
{
  // Update the sensor values whenever new data is available
  if ( imu.gyroAvailable() )
  {
    // To read from the gyroscope,  first call the
    // readGyro() function. When it exits, it'll update the
    // gx, gy, and gz variables with the most current data.
    imu.readGyro();
  }
  if ( imu.accelAvailable() )
  {
    // To read from the accelerometer, first call the
    // readAccel() function. When it exits, it'll update the
    // ax, ay, and az variables with the most current data.
    imu.readAccel();
  }
  if ( imu.magAvailable() )
  {
    // To read from the magnetometer, first call the
    // readMag() function. When it exits, it'll update the
    // mx, my, and mz variables with the most current data.
    imu.readMag();
  }
}
byte logFrambot()
{
  // Need to check for logging GPS and IMU with 20hz update rate.
  updateIMU();
  File logFile = SD.open(logFileName, FILE_WRITE); // Open the log file
  if (logFile)
  {
    logFile.print(Bot.heading,10);
    logFile.print(' ');
    logFile.print(Bot.roll,10);
    logFile.print(' ');
    logFile.print(Bot.pitch,10);
    logFile.print(' ');
    logFile.print(Bot.ax,10);
    logFile.print(' ');
    logFile.print(Bot.ay,10);
    logFile.print(' ');
    logFile.print(Bot.az,10);
    logFile.print(' ');
    logFile.print(tinyGPS.location.lng(), 6);
    logFile.print(',');
    logFile.print(tinyGPS.location.lat(), 6);
    logFile.print(',');
    logFile.print(tinyGPS.altitude.feet(), 1);
    logFile.print(',');
    logFile.print(tinyGPS.speed.mph(), 1);
    logFile.print(',');
    logFile.print(tinyGPS.course.deg(), 1);
    logFile.print(',');
    logFile.print(tinyGPS.date.value());
    logFile.print(',');
    logFile.print(tinyGPS.time.value());
    logFile.print(',');
    logFile.print(tinyGPS.satellites.value());
    logFile.print(',');
    logFile.println();
    logFile.close();

    return 1; // Return success
  }

  return 0; // If we failed to open the file, return fail
}

// printHeader() - prints our eight column names to the top of our log file
void printHeader()
{
  File logFile = SD.open(logFileName, FILE_WRITE); // Open the log file

  if (logFile) // If the log file opened, print our column names to the file
  {
    int i = 0;
    for (; i < LOG_COLUMN_COUNT; i++)
    {
      logFile.print(log_col_names[i]);
      if (i < LOG_COLUMN_COUNT - 1) // If it's anything but the last column
        logFile.print(','); // print a comma
      else // If it's the last column
        logFile.println(); // print a new line
    }
    logFile.close(); // close the file
  }
}

// updateFileName() - Looks through the log files already present on a card,
// and creates a new file with an incremented file index.
void updateFileName()
{
  int i = 0;
  for (; i < MAX_LOG_FILES; i++)
  {
    memset(logFileName, 0, strlen(logFileName)); // Clear logFileName string
    // Set logFileName to "gpslogXX.csv":
    sprintf(logFileName, "%s%d.%s", LOG_FILE_PREFIX, i, LOG_FILE_SUFFIX);
    if (!SD.exists(logFileName)) // If a file doesn't exist
    {
      break; // Break out of this loop. We found our index
    }
    else // Otherwise:
    {
      Serial.print(logFileName);
      Serial.println(" exists"); // Print a debug statement
    }
  }
  Serial.print("File name: ");
  Serial.println(logFileName); // Debug print the file name
}


void motorstop(void)                    //Stop
{
  digitalWrite(E1,0); 
  digitalWrite(M1,LOW);    
  digitalWrite(E2,0);   
  digitalWrite(M2,LOW);    
}   
void motoradvance(char a,char b)          //Move forward
{
  analogWrite (E1,a);      //PWM Speed Control
  digitalWrite(M1,HIGH);    
}  
void back_off (char a,char b)          //Move backward
{
  analogWrite (E1,a);
  digitalWrite(M1,LOW);   
}
void turn_L (char a,char b)             //Turn Left
{
  analogWrite (E2,b);    
  digitalWrite(M2,HIGH);
}
void turn_R (char a,char b)             //Turn Right
{
  analogWrite (E2,b);    
  digitalWrite(M2,LOW);
}

