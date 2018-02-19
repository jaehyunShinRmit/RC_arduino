/******************************************************************************
  # GPS/SD card code is adopted from
  https://github.com/sparkfun/GPS_Shield
  It uses the TinyGPS++ library to parse the NMEA strings sent by the GPS module,
  and prints interesting GPS information - comma separated - to a newly created
  file on the SD card.
  TinyGPS++ Library  - https://github.com/mikalhart/TinyGPSPlus/releases

  # 9DOF IMU
  The LSM9DS1 is a versatile 9DOF sensor.
  Hardware setup: This library supports communicating with the
  LSM9DS1 over either I2C.
  The pin-out is as follows:
  LSM9DS1 --------- Arduino
   SCL    --------- SCL (A5 on older 'Duinos')
   SDA    --------- SDA (A4 on older 'Duinos')
   VDD    --------- 3.3V
   GND    --------- GND

  # DC Motor Driver 2x15A_lite module.
  The pin-out is as follows:
  Driver  --------- Arduino
   M1_PWM --------- PIN5
   M2_PWM --------- PIN6
   M1_EN  --------- PIN4
   M2_EN  --------- PIN7
   +5v    --------- +5v
   GND    --------- GND
******************************************************************************/
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <TinyGPS++.h>
#include <SparkFunLSM9DS1.h>
#include "FarmbotSensor.h"
#include "Command.h"
//#include "GCodeProcessor.h" //need to add to use Gcode
#include "KCodeProcessor.h"
#include "TimerOne.h"
#include "pins.h"
#include "Config.h"

#include "TimerOne.h"
#include "MemoryFree.h"
#include "Debug.h"


FarmbotSensor Bot; // Frambot Sensor state variables
LSM9DS1 imu; //IMU
static char commandEndChar = 0x0A;
static KCodeProcessor *kCodeProcessor = new KCodeProcessor();
//static GCodeProcessor *gCodeProcessor = new GCodeProcessor();

unsigned long lastAction;
unsigned long currentTime;
unsigned long cycleCounter = 0;
bool previousEmergencyStop = false;

unsigned long pinGuardLastCheck;
unsigned long pinGuardCurrentTime;

int lastParamChangeNr = 0;

String commandString = "";
char incomingChar = 0;
char incomingCommandArray[INCOMING_CMD_BUF_SIZE];
int incomingCommandPointer = 0;

// Blink led routine used for testing
bool blink = false;
void blinkLed()
{
  blink = !blink;
  digitalWrite(LED_PIN, blink);
}
// Interrupt handling for:
//   - movement
//   - encoders
//   - pin guard

//Status of Arduino
static int farmbotStatus = 1;
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


///////////////////////
// Example I2C Setup //
///////////////////////
// SDO_XM and SDO_G are both pulled high, so our addresses are:
#define LSM9DS1_M   0x1E // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG  0x6B // Would be 0x6A if SDO_AG is LOW

////////////////////////////
// Sketch Output Settings //
////////////////////////////
//#define PRINT_CALCULATED
#define PRINT_RAW
#define PRINT_SPEED 50  // 50ms -> 20hz update rate


#define MINUTE_INITIAL_ORIANTATION 0.5   // 1minute
#define MSECONDE_INITIAL_ORIANTATION MINUTE_INITIAL_ORIANTATION*60000 //ms of TIME_MINUTE
#define MAXINDEX_INITIAL_ORIANTATION MSECONDE_INITIAL_ORIANTATION/PRINT_SPEED

#define MSECONDE_ADVANCE 6000 //ms
#define MAXINDEX_ADVANCE MSECONDE_ADVANCE/PRINT_SPEED

#define MINUTE_COLLECTING 1   // 1minute
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
#define LOG_COLUMN_COUNT 15
char * log_col_names[LOG_COLUMN_COUNT] = {
  "ms", "heading(deg)", "roll(deg)", "pitch(deg)", "ax(g)", "ay(g)", "az(g)", "lag", "lat", "alt", "speed(m/h)", "course", "data", "time", "satellites"
}; // log_col_names is printed at the top of the file.


// Offsets applied to raw x/y/z values
float mag_offsets[3]            = { 37793.5207402, -6321.2426109, 6891.2431500 };

// Soft iron error compensation matrix
float mag_softiron_matrix[3][3] = { { 0.97196, 0.00833, 0.00444 },
  { 0.00833, 1.04714, -0.00693 },
  { 0.00444, -0.00693, 1.00345 }
};

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
int counter = 0;

//////////////////////
// Log Rate Control //
//////////////////////
unsigned long lastLog = 0; // Global var to keep of last time we logged


unsigned long interruptStartTime = 0;
unsigned long interruptStopTime = 0;
unsigned long interruptDuration = 0;
unsigned long interruptDurationMax = 0;

bool interruptBusy = false;
int interruptSecondTimer = 0;
void interrupt(void)
{
  if (!debugInterrupt)
  {
    interruptSecondTimer++;

    if (interruptBusy == false)
    {
      interruptBusy = true;
      // triggered once per 10 ms
      if (interruptSecondTimer >= 100000 / MOVEMENT_INTERRUPT_SPEED)
      {    
        interruptSecondTimer = 0;
        updateOriantation();
        Serial.print("Initial - Orientation: ");
        Serial.print(Bot.roll_deg);
        Serial.print(" ");
        Serial.print(Bot.pitch_deg);
        Serial.print(" ");
        Serial.println(Bot.heading_deg);
      }
      interruptBusy = false;
    }
  }
}


void setup()
{
  
  Serial.begin(115200);
  /*
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
 
  // Motor Driver setup
  int i;
  for (i = 4; i <= 7; i++)
    pinMode(i, OUTPUT);
  digitalWrite(E1, LOW);
  digitalWrite(E2, LOW);
  pinMode(2, INPUT);
  pinMode(3, INPUT);
*/
  // IMU initialization
  imu.settings.device.commInterface = IMU_MODE_I2C;
  imu.settings.device.mAddress = LSM9DS1_M;
  imu.settings.device.agAddress = LSM9DS1_AG;
  if (!imu.begin())
  {
    Serial.println("Failed to communicate with LSM9DS1.");
    while (1);
  } 
  
  // Start the interrupt used for moving
  // Interrupt management code library written by Paul Stoffregen
  // The default time 100 micro seconds

 
  // The default time 100 micro seconds
  Timer1.attachInterrupt(interrupt);
  Timer1.initialize(MOVEMENT_INTERRUPT_SPEED);
  Timer1.start();

  // Initialize the inactivity check
  lastAction = millis();
  Serial.print("R99 ARDUINO STARTUP COMPLETE\r\n");
}


char commandChar[INCOMING_CMD_BUF_SIZE + 1];

void loop()
{
  /*
  int i;
  if ((lastLog + PRINT_SPEED) <= millis())// If it's been LOG_RATE milliseconds since the last log:
  {
    switch (farmbotStatus) {
      case SENSOR_INITIALIZATION:
        //   Serial.println("Sensor initializing.");
        if (gpsPort.available()) // When GPS start available go to the next step
          farmbotStatus = ROBOT_INITIAL_ORIANTATION;
        break;
      case ROBOT_INITIAL_ORIANTATION:
        //    Serial.println("Calculating Initial Oriantation - start.");
        if (MAXINDEX_INITIAL_ORIANTATION > iInitialOriantation++)
        {
          //            Serial.print("Initial - Orientation: ");
          //            Serial.print(Bot.heading_deg);
          //            Serial.print(" ");
          //            Serial.print(Bot.pitch_deg);
          //            Serial.print(" ");
          //            Serial.println(Bot.roll_deg);
          // Calculate initial oriatation by averaging IMU and GPS
          // recusrive mean calculation
          InitialOriantationUpdate(iInitialOriantation);
          if (tinyGPS.location.isUpdated())
          {
            InitialGPSUpdate(++iInitialGPS);
          }
        }
        else {
          Serial.println("Calculating Initial Oriantation - completed.");
          Serial.print("Initial - Orientation: ");
          Serial.print(Bot.initHeading);
          Serial.print(" ");
          Serial.print(Bot.initPitch);
          Serial.print(" ");
          Serial.println(Bot.initRoll);
          // After calculting initial oriantation move the next step
          farmbotStatus = ADVANCE;
          iInitialOriantation = 0;
          iInitialGPS = 0;
        }
        break;
      case ADVANCE:
        Serial.println("GO!!");
        if (MAXINDEX_ADVANCE > iAdvance++)
        {
          motoradvance (255, 255);  //move forward in max speed
          logFarmbot();
        }
        else {
          farmbotStatus = COLLECTING_DATA;
          iAdvance = 0;
        }
        break;
      case COLLECTING_DATA:
        Serial.println("Stop!!");
        if (MAXINDEX_COLLECTING > iCollecting++)
        {
          motorstop();
        }
        else {
          farmbotStatus = WEEDING;
          iCollecting = 0;
        }
        break;
      case WEEDING:
        // weeding task
        // need to code
        File logFile = SD.open(logFileName, FILE_WRITE); // Open the log file

        logFile.print(Bot.initHeading, 6);
        logFile.print(',');
        logFile.print(Bot.initPitch, 6);
        logFile.print(',');
        logFile.print(Bot.initRoll, 1);
        logFile.println();
        logFile.close();
        break;
    }
    lastLog = millis(); // Update the lastLog variable
  }
  // If we're not logging, continue to "feed" the tinyGPS object:
  while (gpsPort.available())
    tinyGPS.encode(gpsPort.read());
*/
  if ((lastPrint + PRINT_SPEED) < millis())
  {
   // updateOriantation();
   ///updateIMU();
    updateFarmbot();
    lastPrint = millis(); // Update lastPrint time
  }

  if (Serial.available()) {

    // Save current time stamp for timeout actions
    lastAction = millis();

    // Get the input and start processing on receiving 'new line'
    incomingChar = Serial.read();

    // Filter out emergency stop.
    if (!(incomingChar == 69 || incomingChar == 101))
    {
      incomingCommandArray[incomingCommandPointer] = incomingChar;
      incomingCommandPointer++;
     
    }
    else
    {
      KCurrentState::getInstance()->setEmergencyStop();
    }

    // If the string is getting to long, cap it off with a new line and let it process anyway
    if (incomingCommandPointer >= INCOMING_CMD_BUF_SIZE - 1)
    {
      incomingChar = '\n';
      incomingCommandArray[incomingCommandPointer] = incomingChar;
      incomingCommandPointer++;
    }

    if (incomingChar == '\n' || incomingCommandPointer >= INCOMING_CMD_BUF_SIZE)
    {

      //char commandChar[incomingCommandPointer + 1];
      for (int i = 0; i < incomingCommandPointer - 1; i++)
      {
          commandChar[i] = incomingCommandArray[i];
      }
      commandChar[incomingCommandPointer-1] = '\0';

      if (incomingCommandPointer > 1)
      {

        // Report back the received command
        Serial.print(COMM_REPORT_CMD_ECHO);
        Serial.print(" ");
        Serial.print("*");
        Serial.print(commandChar);
        Serial.print("*");
        Serial.print("\r\n");

        // Create a command and let it execute
        Command *command = new Command(commandChar);
      
        // Log the values if needed for debugging
        if (LOGGING || debugMessages)
        {
          command->print();
        }

        if(command->getCodeEnum()> 300) //K command starts from 311
        {
          kCodeProcessor->execute(command);     
        }
        else{ //G command 
          //gCodeProcessor->execute(command);
        }

        free(command);

      }

      incomingCommandPointer = 0;
    }
  }

}

void updateFarmbot()
{
  updateIMU();
  Bot.ax = (long)(imu.calcAccel(imu.ax)*1000);
  Bot.ay = (long)(imu.calcAccel(imu.ay)*1000);
  Bot.az = (long)(imu.calcAccel(imu.az)*1000);
  Bot.gx = (long)(imu.calcGyro(imu.gx)*1000);
  Bot.gy = (long)(imu.calcGyro(imu.gy)*1000);
  Bot.gz = (long)(imu.calcGyro(imu.gz)*1000);
  Bot.mx = (long)(imu.calcMag(imu.mx)*100000);
  Bot.my = (long)(imu.calcMag(imu.my)*100000);
  Bot.mz = (long)(imu.calcMag(imu.mz)*100000);
}
void updateOriantation()
{
  //updateFarmbot(); // update the current measurement of sensors on the Farmbot

  float x =  (float)Bot.mx - mag_offsets[0];
  float y =  (float)Bot.my - mag_offsets[1];
  float z =  (float)Bot.mz - mag_offsets[2];

  // Apply mag soft iron error compensation
  float mx = x * mag_softiron_matrix[0][0] + y * mag_softiron_matrix[0][1] + z * mag_softiron_matrix[0][2];
  float my = x * mag_softiron_matrix[1][0] + y * mag_softiron_matrix[1][1] + z * mag_softiron_matrix[1][2];
  float mz = x * mag_softiron_matrix[2][0] + y * mag_softiron_matrix[2][1] + z * mag_softiron_matrix[2][2];

  // Pitch and roll
  float roll  = atan2((float)Bot.ay, (float)Bot.az);
  float pitch = atan2(-(float)Bot.ax, (float)Bot.ay * sin(roll) + (float)Bot.az * cos(roll));
  float roll_deg = roll * 180.0 / M_PI;
  float pitch_deg = pitch * 180.0 / M_PI;

  // Tilt compensated magnetic sensor measurements
  float mx_comp = mx * cos(pitch) + my * sin(pitch) * sin(roll) + mz * sin(pitch) * cos(roll);
  float my_comp = mz * sin(roll) - my * cos(roll);

  // Arctangent of y/x
  float heading = atan2(my_comp, mx_comp);
  float heading_deg = heading * 180.0 / M_PI;
  if (heading_deg  < 0.0)
    heading_deg += 360.0;

  // Change to long type
  Bot.roll  = (long) roll; 
  Bot.pitch = (long) pitch;
  Bot.roll_deg = (long) roll_deg;
  Bot.pitch_deg = (long) pitch_deg;
  Bot.heading = (long) heading;
  Bot.heading_deg = (long) heading_deg;
}

void InitialOriantationUpdate(int n)
{
  updateOriantation();
  Bot.initRoll  = ((Bot.initRoll * n) + Bot.roll_deg) / (n + 1);
  Bot.initPitch = ((Bot.initPitch * n) + Bot.pitch_deg) / (n + 1);
  Bot.initHeading = ((Bot.initHeading * n) + Bot.heading_deg) / (n + 1);
}

void InitialGPSUpdate(int n)
{
  Bot.initLat  = ((Bot.initLat * n) + tinyGPS.location.lat()) / (n + 1);
  Bot.initLon  = ((Bot.initLon * n) + tinyGPS.location.lng()) / (n + 1);
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
byte logFarmbot()
{
  // Need to check for logging GPS and IMU with 20hz update rate.
  updateOriantation();
  File logFile = SD.open(logFileName, FILE_WRITE); // Open the log file
  if (logFile)
  {
    logFile.print(millis(), 10);
    logFile.print(',');
    logFile.print(Bot.heading_deg, 10);
    logFile.print(',');
    logFile.print(Bot.roll_deg, 10);
    logFile.print(',');
    logFile.print(Bot.pitch_deg, 10);
    logFile.print(',');
    logFile.print(Bot.ax, 10);
    logFile.print(',');
    logFile.print(Bot.ay, 10);
    logFile.print(',');
    logFile.print(Bot.az, 10);
    logFile.print(',');
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
  digitalWrite(E1, 0);
  digitalWrite(M1, LOW);
  digitalWrite(E2, 0);
  digitalWrite(M2, LOW);
}
void motoradvance(char a, char b)         //Move forward
{
  analogWrite (E1, a);     //PWM Speed Control
  digitalWrite(M1, HIGH);
}
void back_off (char a, char b)         //Move backward
{
  analogWrite (E1, a);
  digitalWrite(M1, LOW);
}
void turn_L (char a, char b)            //Turn Left
{
  analogWrite (E2, b);
  digitalWrite(M2, HIGH);
}
void turn_R (char a, char b)            //Turn Right
{
  analogWrite (E2, b);
  digitalWrite(M2, LOW);
}

