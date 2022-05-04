//Libraries for GPS
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

//Libraries for nrf24l01

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

//Global Declarations for GPS

static const int RxPin=4, TxPin=3; //initializing Rx and Tx Pins
static const uint32_t GPSBaud=9600; //Setting up the gps baud rate
TinyGPSPlus gps; //assigning an object to the TinyGPSPlus function
SoftwareSerial ss(RxPin,TxPin); //assigning an object for the SoftwareSerial function
float L_Altitude,Latitude,Longitude; //Declaring the variable for storing the values detected by gps

String a;
//Global Declarations for Nrf24L01

RF24 radio(9, 8); // CE, CSN
const byte address[6] = "00001";//address through which two modules communicate.

/*$$$$$$$$$$$$$$$$$$$$$$$$$$$$ GPS Function $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$*/

String GPS_program()
{ 
  String a;
  String k;
  if (ss.available()!=0) // check for gps data
  {
    Serial.println("GPS Start");
    gps.encode(ss.read());
    if(gps.location.isUpdated()) // encode gps data
    {
      Serial.println("updated");
      Latitude=gps.location.lat(); //Stroring the gps latitude in Latitude Variable
      Longitude=gps.location.lng(); //Stroring the gps longitude in Longitude Variable
      L_Altitude=gps.altitude.meters(); //Stroring the gps altitude in Altitude Variable
      Serial.println(gps.location.lat());
      Serial.println(gps.location.lng());
      Serial.println(gps.altitude.meters());
      String L_latitude=String(Latitude); //Converting the latitude value multiplied by 10^6 back to string format
      String L_longitude=String(Longitude); //Converting the longitude value multiplied by 10^6 back to string format
      a=String(L_latitude+","+L_longitude+","+L_Altitude);//Combining the String Values
      return a; //Additional, can be removed
    }
  } 
  Serial.println(a);
}

/*$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$ Nrf24L01 Function $$$$$$$$$$$$$$$$$$$$$$$$$$$$$*/

void nrf24L01_program(const char mystr) //Nrf24L01 Program
{
  radio.write(&mystr, sizeof(mystr));
}
void setup()
{
  //Setup for Nrf24L01
  radio.begin();
  radio.openWritingPipe(address);//set the address
  radio.stopListening();//Set module as transmitter
  //Setup for GPS
  Serial.begin(9600); // connect serial
  ss.begin(GPSBaud); // connect gps sensor
}

void loop()
{
  // GPS_program();
  String gps_string=GPS_program(); //retreiving the GPS String
  //String mpu_string=MPU_Program(); //retreiving the MPU String
  String total_string=String(gps_string);//Combining both the Strings for sending it through nrf24L01
  const int len=total_string.length();
  char mystring[len];
  total_string.toCharArray(mystring,len);
  Serial.println(mystring);
  Serial.println(total_string);
  nrf24L01_program(mystring); //Transmitting the String to receiver
  delay(1000);
}
