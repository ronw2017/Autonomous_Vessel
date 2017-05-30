#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <Servo.h>
#include <math.h>

//TESTING------------------------------------------------------------------------------------------------RW 
// SIGN  METHOD        DETAIL    DATE
//(RW)  (LAND TESTED) (WALKING) (5/22/16)

//GLOBAL INITIALIZATION----------------------------------------------------------------------------------RW
int pos=0; //used heavily in Haversine Formula
int d=3;   //Left=0, Right=1, Center=3
int ts=0;  //turn state 1=right, 2=left, 3=center
int ms=0;  //motion state 1=forward, 2=reverse, 3=stop
int LL=-1; //location index "-1" to receive first update
bool LOCK=true; //destination reached Note: "true" to load first location
double lat2=1.0;
double lon2=1.0;

//LOCATION LIST------------------------------------------------------------------------------------------RW
double LAT[] = {28.1372,28.137246,28.150127,24.555059,38.966143}; //array starts at 0
double LON[] = {-81.6231,-81.620012,-81.850741,-81.779987,-76.895797};//UPSTREET,BACK,SCHOOL,KEYWEST,MD

//DUAL H BRIDGE INITIALIZATION---------------------------------------------------------------------------RW
int Ain1 = 11;
int Ain2 = 10;
int Bin3 = 6;
int Bin4 = 5;

SoftwareSerial mySerial(3, 2);

Adafruit_GPS GPS(&mySerial);

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences. 
#define GPSECHO  false  //ORIGINAL TRUE APPROVED KEEP FALSE!!

// this keeps track of whether we're using the interrupt
// off by default!
boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy

void setup()  
{   
  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  // also spit it out
  Serial.begin(115200);
  Serial.println("Adafruit GPS library basic test!");

  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
  
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);//ORIGINAL ON
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY); //NO SATELLITE# NO ALTITUDE NO QUALITY
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time
  
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate ORIGINAL ON
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz

  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA); //ORIGINAL ON

  // the nice thing about this code is you can have a timer0 interrupt go off
  // every 1 millisecond, and read data from the GPS for you. that makes the
  // loop code a heck of a lot easier!
  useInterrupt(true);

  delay(1000);
  // Ask for firmware version
  mySerial.println(PMTK_Q_RELEASE);
}


// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c; //THIS IS THE LINE WITH ALL DATA_RonW inserted(Serial.print("");) 
    // writing direct to UDR0 is much much faster than Serial.print 
    // but only one character can be written at a time. 
#endif
}

void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}

uint32_t timer = millis();
void loop()                     // run over and over again
{
  // in case you are not using the interrupt above, you'll
  // need to 'hand query' the GPS, not suggested :(
  if (! usingInterrupt) {
    // read data from the GPS in the 'main loop'
    char c = GPS.read();
    // if you want to debug, this is a good time to do it!
    if (GPSECHO)
      if (c) Serial.print(c);
  }
  
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }
  if (timer > millis())  timer = millis();

  if (millis() - timer > 2000) { 
    timer = millis(); // reset the timer
    
    //Serial.print("\nTime: ");
    //Serial.print(GPS.hour, DEC); Serial.print(':');
    //Serial.print(GPS.minute, DEC); Serial.print(':');
    //Serial.print(GPS.seconds, DEC); Serial.print('.');
    //Serial.println(GPS.milliseconds);
    //Serial.print("Date: ");
    //Serial.print(GPS.day, DEC); Serial.print('/');
    //Serial.print(GPS.month, DEC); Serial.print("/20");
    //Serial.println(GPS.year, DEC);
    //Serial.print("Fix: "); Serial.print((int)GPS.fix);
    //Serial.print(" quality: "); Serial.println((int)GPS.fixquality); 
    if (GPS.fix) {// THE CODE WILL ONLY ACTIVATE MOTION IF A SATELLITE FIX IS PRESENT--------------------RW
      //Serial.print("Location: ");
      //Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
      //Serial.print(", "); 
      //Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
      //Serial.print("Location (in degrees, works with Google Maps): ");
      Serial.print(GPS.latitudeDegrees, 4);
      Serial.print(", "); 
      Serial.println(GPS.longitudeDegrees, 4);
      
      //Serial.print("Speed (knots): "); Serial.println(GPS.speed);
      Serial.print("Heading Angle: "); Serial.println(GPS.angle);
      //Serial.print("Altitude (cm): "); Serial.println(GPS.altitude);
      //Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
      
      // NEW GEO LOCATION--------------------------------------------------------------------------------RW
      
      if (LOCK == true){ //NEXT LOCATION IS LOADED, WHEN DESTINATION IS REACHED
        LL=LL+1;
        lat2=LAT[LL]; //
        lon2=LON[LL]; //
        LOCK=false;   
      }
      
      Serial.print(lat2,4);
      Serial.print(", ");
      Serial.println(lon2,4);
      
      // HAVERSINE FORMULA BEARING CALCULATION-----------------------------------------------------------RW       
      double lat1=GPS.latitudeDegrees;
      double lon1=GPS.longitudeDegrees;
      
      double d=cos(lat1)*sin(lat2)-sin(lat1)*cos(lat2)*cos(lon2-lon1);
      double f=sin(lon2-lon1)*cos(lat2);
      double thetaDF=atan2(f,d);
      double thetaDFA=thetaDF*(180/PI);
      double thetaDFB= (thetaDFA+180);
      double thetaDFC = fmod(thetaDFB, 360);
      double Heading=thetaDFC;
      
          Serial.print("New Heading ");
          Serial.println(Heading); 
      
      
      // COURSE CORRECTION-------------------------------------------------------------------------------RW
      int CH = GPS.angle;
      
      int DH = Heading;
      
      if (abs(CH-DH)>180){
        if ((CH-DH)>0){
          pos=360-(CH-DH);
          //fprintf('right: %d\n',pos);
          d=1;//right
        }else if ((CH-DH)<0){
          pos=abs(360+(CH-DH));
          //fprintf('left: %d\n',pos);
          d=0;//left
        }
      }
    if (abs(CH-DH)<180){
      if (CH-DH>0){
        pos=CH-DH;
        //fprintf('left: %d\n',pos);
        d=0;//Left
      }else if (CH-DH<0){
        pos=DH-CH;
        //fprintf('right: %d\n',pos);
        d=1;//Right
      }
    }
    // DISTANCE TO DESTINATION---------------------------------------------------------------------------RW
    double dist_c1 = pow((sin(((lat2-lat1)*(PI/180))/2)),2)+(cos(lat1*(PI/180))*cos(lat2*(PI/180))*pow((sin(((lon2-lon1)*(PI/180))/2)),2));
    double dist_c2 = 2*(atan2( (sqrt(dist_c1)) , (sqrt(1-dist_c1)) ));
    double dist = 6371000 * dist_c2;
    double distA=dist*.00062137;

    // DESTINATION EXECUTED CODE GOES HERE (INSIDE IF STATEMENT)-----------------------------------------RW
    if (distA<.1){ //DESTINATION IS REACHED WITHIN .1MI (528 FT) OF TARGET
      LOCK = true; //FOR SMALLER AREA DECREASE distA!!!!
      //stopmove();//UNCOMMENT TO STOP BOAT FOR DESTINATION CODE EXECUTION
      
    }

    // DATA COLLECTION AND STORAGE AREA (NO DATALOGGING HARDWARE INSTALLED CURRENTLY)--------------------
    
    
    // PROGRESS VARIABLE CHECKER-------------------------------------------------------------------------RW
    Serial.print("degree: ");
    Serial.print(pos);
    Serial.print("\n");
    Serial.print("direction(l/r): ");
    Serial.print(d);
    Serial.print("\n");
    Serial.print("Distance(mi): ");
    Serial.print(distA);
    Serial.print("\n");
      
// DIRECTION AND DEGREE CORRECTION WITH 60 DEGREE THRESHOLD---------------------------------------------RW
      if(d==1){
        if(pos>30){ //THRESHOLD VALUE 30 DEG OFF COURSE
          forward(40); //speed 40/255
          right();
          //pos; (accurate offset degrees) DOES NOT USE DEGREES FOR TURNING BOAT MOTOR TURN TOO MILD
          //Serial.print("trigger: d=1\n");//RIGHT TURN ACTIVATION STATUS CHECKPOINT
          
          //ALGORITHM FOR SERVO CONVERSION (ASSUME 90 DEG CENTER)---------------------------------------
          //MOTORDEG=90+pos;
          //
        }
      }else if(d==0){
        if(pos>30){ //THRESHOLD VALUE 30 DEG OFF COURSE
         forward(40); //speed 40/255
         left();
          //pos; (accurate offset degrees) DOES NOT USE DEGREES FOR TURNING BOAT MOTOR TURN TOO MILD
          //Serial.print("trigger: d=0\n");//LEFT TURN ACTIVATION STATUS CHECKPOINT
          
          //ALGORITHM FOR SERVO CONVERSION (ASSUME 90 DEG CENTER)---------------------------------------
          //MOTORDEG=90-pos;
        }
      }
      
      // SPACING FOR SERIAL BEAUTIFICATION
      Serial.print("\n\n");
    }
    }
  }
//MOTION FUNCTION----------------------------------------------------------------------------------------RW
void backward(int spd){
  analogWrite(Ain1, 0);
  analogWrite(Ain2, spd);
  delay(1000);
  ms=2;
  motionstate();
}
void forward(int spd){
  analogWrite(Ain1, spd);
  analogWrite(Ain2, 0);
  delay(1000);
  ms=1;
  motionstate();
}
//TURNING FUNCTION --------------------------------------------------------------------------------------RW
void left(){//int spd){
  int spd=160;
  analogWrite(Bin3, 0);
  analogWrite(Bin4, spd);
  delay(1000);
  ts=2;
  turnstate();
  stopturn();
  
}
void right(){//int spd){
  int spd=160;
  analogWrite(Bin3, spd);
  analogWrite(Bin4, 0);
  delay(1000);
  ts=1;
  turnstate();
  stopturn();
}
//RESTING FUNCTIONS--------------------------------------------------------------------------------------RW
void stopturn(){
  analogWrite(Bin3, 0);
  analogWrite(Bin4, 0);
  delay(100);
  ts=3;
  turnstate();
  
}
void stopmove(){
  analogWrite(Ain1, 0);
  analogWrite(Ain2, 0);
  delay(20);
  ms=3;
  motionstate();
}
//STATUS FUNCTIONS FOR TESTING---------------------------------------------------------------------------RW
void motionstate(){
  Serial.print("motion state: ");
  if(ms==1){
    Serial.print("forward");
  }else if(ms==2){
    Serial.print("reverse");
  }else if(ms==3){
    Serial.print("stop");
  }
  Serial.println("\n");
}
void turnstate(){
  Serial.print("turn state: ");
  if(ts==1){
    Serial.print("right");
  }else if(ts==2){
    Serial.print("left");
  }else if(ts==3){
    Serial.print("center");
  }
  Serial.println("\n");
}  
