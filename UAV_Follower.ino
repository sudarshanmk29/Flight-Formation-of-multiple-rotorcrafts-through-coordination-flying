#include <SPI.h>
#include <SoftwareSerial.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <TinyGPS++.h>
#include <Servo.h>
#include "MPU9250.h"
//nrf24e global declarations
RF24 radio(9,8); //CE,CSN
const byte address[6] = "00001";

//gps global declarations

static const int RXPin = 2, TXPin = 3;
static const uint32_t GPSBaud = 9600;
TinyGPSPlus gps; // The TinyGPS++ object
SoftwareSerial ss(RXPin, TXPin); // The serial connection to the GPS device
//global declarations for mpu
MPU9250 mpu;

struct dataStruct //for receiver
{
  double Latitude;
  double Longitude;
  double Altitude;
  float Roll;
  float Pitch;
  float Yaw;
  int Throttle;
}getData;

struct GPSData //for the gps used on the follower.
{
  double L_latitude;
  double L_longitude;
  double L_Altitude;
}gpsData;

double gap= 10; //assign temporary token
double gap_in_DMS ;

//////////////////////// Global Variable Declarations /////////////////////////////////////////////

int rollF1, pitchF1, yawF1;
double Error_Altitude,Longitude_error , expected_longitude,Latitude_error , expected_latitude;
double Matrix_position_row = 1 , Matrix_position_coloumn =2 ,Altitude_control1 ,Pitch_control1, Roll_control1,yaw_control1;
// for PWM using Servo Library
Servo throttle;
Servo roll;
Servo pitch;
Servo yaw;

void setup()
{
  Serial.begin(9600);
  Serial.println("setup");

  radio.begin();
  //set the address
  radio.openReadingPipe(0, address);
  //Set module as receiver
  radio.startListening();
  ///////////////////////////////MPU/////////////////////////////
  Wire.begin();
  Serial.print("mpu setup initiated");
  mpu.setup(0x68);
  mpu.update();
  Serial.print("roll (x-forward (north)) : ");
  Serial.println(mpu.getRoll());
  Serial.print("pitch (y-right (east)) : ");
  Serial.println(mpu.getPitch());
  Serial.print("yaw (z-down (down)) : ");
  Serial.println(mpu.getYaw());
  Serial.println("roll/pitch adjustment");
  throttle.attach(4,1000,2000);
  roll.attach(5,1000,2000);
  pitch.attach(6,1000,2000);
  yaw.attach(7,1000,2000);
  throttle.write(1000),roll.write(1000),pitch.write(1000), yaw.write(1000);
}
void loop()
{
  Serial.println("main");
  Recieve_data();
  IMU();
  Calculate_Desired_position() ;
  gps1();
  Calculate_error();
  Calculate_setpoints();
  ///////////////////By Default in coordination mode write data to Ardu Pilot from Arduino /////////////
  Write_data_Arduino();
}
void Recieve_data() //nrf24e
{
  if (radio.available())
  {
    radio.read(&getData, sizeof(getData));
    Serial.print("Location: ");
    Serial.print(getData.Latitude, 6);
    Serial.print(", ");
    Serial.print(getData.Longitude, 6);
    Serial.print(",");
    Serial.print(getData.Altitude);
    Serial.print(",");
    Serial.print(getData.Throttle);
    Serial.print(",");
    Serial.print(getData.Roll);
    Serial.print(",");
    Serial.print(getData.Pitch);
    Serial.print(",");
    Serial.print(getData.Yaw);
    Serial.println();
    Serial.println("**************** Recieved ********************");
  }
  else
  {
    if(getData.Throttle>1000)
    {
      getData.Throttle-=5;
    }
    else
    {
      getData.Throttle=1000;
    }
    getData.Roll=1500;
    getData.Pitch=1500;
  }
}

void Calculate_Desired_position() //matrix positioning
{
  gap= 10; //assign temporary token
  gap_in_DMS = gap*(100) ;
  expected_longitude = getData.Longitude+ (Matrix_position_coloumn* gap_in_DMS);
  expected_latitude = getData.Latitude+ (Matrix_position_row* gap_in_DMS);
  Serial.println(expected_longitude);
  Serial.println(expected_latitude);
}
void IMU()
{
    mpu.update();
    Serial.print("roll (x-forward (north)) : ");
    Serial.println(mpu.getRoll());
    Serial.print("pitch (y-right (east)) : ");
    Serial.println(mpu.getPitch());
    Serial.print("yaw (z-down (down)) : ");
    Serial.println(mpu.getYaw());
    rollF1 = mpu.getRoll(); 
    pitchF1 = mpu.getPitch();
    yawF1 = mpu.getYaw();
}
void gps1() //gps location of follower
{
  while (ss.available() > 0)
  {
    if (gps.encode(ss.read()))
    {
      if (gps.location.isUpdated())
      {
        gpsData.L_longitude = gps.location.lng();
        gpsData.L_latitude = gps.location.lat();
        gpsData.L_Altitude=gps.altitude.meters();
      }
      else
      {
        gpsData.L_longitude = 0.0;
        gpsData.L_latitude = 0.0;
        gpsData.L_Altitude = 0.0;
      }
    }
  }
}
void Calculate_error()
{
    Error_Altitude= 0;
    Longitude_error = 0;
    Latitude_error = 0;
    Error_Altitude= getData.Altitude - gpsData.L_Altitude;
    Longitude_error = expected_longitude- gpsData.L_longitude;
    Latitude_error = expected_latitude - gpsData.L_latitude;
    Serial.println("ERROR VALUES");
    Serial.println(Error_Altitude);
    Serial.println(Longitude_error);
    Serial.println(Latitude_error);
}
void Calculate_setpoints()
{
    Altitude_control1 = getData.Throttle;
    if (Longitude_error>50 && Longitude_error<100) rollF1 = -7;
    if (Longitude_error>=100) rollF1 = -15;
    if (Longitude_error<(-100)) rollF1 = 15;
    if (Longitude_error<(-50) && Longitude_error>=(-100)) rollF1 = 7;
    if (Longitude_error == 0) rollF1 = 0;
    Roll_control1 = rollF1;
    if (Latitude_error>=0.0001 && Latitude_error<100) pitchF1 = -7;
    if (Latitude_error>=100) pitchF1 = -15;
    if (Latitude_error<(-100)) pitchF1 = 15;
    if (Latitude_error<0 && Latitude_error>=(-100)) pitchF1 = 15;
    if (Latitude_error == 0) pitchF1 = 0;
    Pitch_control1 = pitchF1;
    yaw_control1 = ((-yawF1)*2)+ getData.Yaw;
    Serial.println("SETPOINTS");
    Serial.println(Altitude_control1);
    Serial.println(Pitch_control1);
    Serial.println(Roll_control1);
    Serial.println(yaw_control1);
}  

void Write_data_Arduino()
{
    if(Altitude_control1>1000)
    {
      throttle.write(Altitude_control1);
      pitch.write(Pitch_control1);
      roll.write(Roll_control1);
      yaw.write(yaw_control1);
      Serial.println("OUTPUT");
      Serial.println(Altitude_control1);
      Serial.println(Pitch_control1);
      Serial.println(Roll_control1);
      Serial.println(1500+yaw_control1);
    }
    else
    {
      throttle.write(1000);
      pitch.write(1500);
      roll.write(1500);
      yaw.write(1500);
      Serial.println("OUTPUT");
      Serial.println(Altitude_control1);
      Serial.println(Pitch_control1);
      Serial.println(Roll_control1);
      Serial.println(yaw_control1);
    }
}
