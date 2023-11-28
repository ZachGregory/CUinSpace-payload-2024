#include <SD.h>
#include <SPI.h>
#include <Wire.h>
#include "ICM_20948.h" //ICM Library
#include <SparkFun_u-blox_GNSS_v3.h> //GPS Library
#include <Adafruit_Sensor.h> //Altimeter Library
#include "Adafruit_BMP3XX.h" //Also Altimeter Library
#include <SparkFun_I2C_Mux_Arduino_Library.h> //Mux Library

// The value of the last bit of the I2C address.
// On the SparkFun 9DoF IMU breakout the default is 1, and when the ADR jumper is closed the value becomes 0
#define AD0_VAL 1
#define SEALEVELPRESSURE_HPA (1009.3)
#define TIMEZONE (-5) //Ottawa
//#define TIMEZONE (-7) //New Mexico

File myFile;
QWIICMUX myMux;
SFE_UBLOX_GNSS myGNSS;
ICM_20948_I2C myICM;
Adafruit_BMP3XX bmp;

int pinCS = 10; // Pin 10 on Arduino Uno
const int chipSelect = 4;
long lastTime = 0; //For GPS interval
bool headingWritten = false;
long int runTime = 0;
String hour;
String minute;
String second;

void setup()
{
  
  Serial.begin(115200);
  delay(2000);  //Give time for serial

  Wire.begin();
  Wire.setClock(400000);

    //Initialize Mux
  if (myMux.begin() == false)
  {
    Serial.println("Mux not detected. Freezing...");
    while (1);
  }
  Serial.println("Mux detected");
  myMux.enablePort(5); //Alt=5
  myMux.enablePort(1); //GPS=1
  myMux.enablePort(7); //IMU=7

  //Initialize IMU
  bool initialized = false;
  while (!initialized)
  {
    myICM.begin(Wire, AD0_VAL);
    Serial.print(F("Initialization of the sensor returned: "));
    Serial.println(myICM.statusString());
    if (myICM.status != ICM_20948_Stat_Ok)
    {
      Serial.println("Trying again...");
      delay(500);
    }
    else
    {
      initialized = true;

    }
  }

  //Initialize Altimeter
  if (!bmp.begin_I2C()) {   // hardware I2C mode, can pass in address & alt Wire
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
    while (1);
  }
  // Set up oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

  //Initialize GPS
  if (myGNSS.begin() == false) 
  {
    Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
    while (1);
  }
  myGNSS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
  myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); //Save (only) the communications port settings to flash and BBR

  //Initialize SD Card
    Serial.print("Initializing SD card...");

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    while (1);
  }
  Serial.println("card initialized.");
}


//main
void loop()
{

  //Update IMU data
  if (myICM.dataReady()){
    myICM.getAGMT();
  } else{
    Serial.println("Waiting for data");
  }

  runTime = millis();

  // Data Logging
  String dataString = "";
  dataString += runTime;
  dataString += ", ";
  hour = String(myGNSS.getHour()+TIMEZONE);
  minute = String(myGNSS.getMinute());
  second = String(myGNSS.getSecond());
  if (second.length()<2){second="0"+second;}
  if (minute.length()<2){minute="0"+minute;}
  if (hour.length()<2){hour="0"+hour;}
  dataString += hour + ":" + minute + ":" + second;
  dataString += ", ";
  dataString += bmp.temperature;
  dataString += ", ";
  dataString += bmp.pressure/100;
  dataString += ", ";
  dataString += bmp.readAltitude(SEALEVELPRESSURE_HPA);
  dataString += ", ";
  dataString += myGNSS.getLatitude();
  dataString += ", ";
  dataString += myGNSS.getLongitude();
  dataString += ", ";
  dataString += String(myICM.agmt.acc.axes.x) + ", " + String(myICM.agmt.acc.axes.y) + ", " + String(myICM.agmt.acc.axes.z) + ", "; //gives raw data
  dataString += String(myICM.agmt.gyr.axes.x) + ", " + String(myICM.agmt.gyr.axes.y) + ", " + String(myICM.agmt.gyr.axes.z) + ", "; //gives raw data
  dataString += String(myICM.agmt.mag.axes.x) + ", " + String(myICM.agmt.mag.axes.y) + ", " + String(myICM.agmt.mag.axes.z); //gives raw data
  



  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  File dataFile = SD.open("datalog.csv", FILE_WRITE);


  // if the file is available, write to it:
  if (dataFile) {
    if (!headingWritten){
      dataFile.println("Time (ms), Time (hh:mm:ss), Temperature (C), Pressure (hPa), Altitude (m), Latitude, Longitude, Acc X (mg), Acc Y (mg), Acc Z (mg), Gyr X (DPS), Gyr Y (DPS), Gyr Z (DPS), Mag X (uT), Mag Y (uT), Mag Z (uT)");
      headingWritten = true;
    }
    dataFile.println(dataString);
    dataFile.close();
    // print to the serial port too:
    dataString.replace(", ", "\t\t");
    Serial.println("Time (ms) \tTime (hh:mm:ss) \tTemperature (C) Pressure (hPa) \tAltitude (m) \tLatitude \t\tLongitude \t\tAcc X (mg) \tAcc Y (mg) \tAcc Z (mg) \tGyr X (DPS) \tGyr Y (DPS) \tGyr Z (DPS) \tMag X (uT) \tMag Y (uT) \tMag Z (uT)");
    Serial.println(dataString);
    Serial.println();
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening datalog.csv");
  }
}
