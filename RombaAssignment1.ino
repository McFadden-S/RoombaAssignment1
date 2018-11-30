#include<SoftwareSerial.h>

// pin assignments for arduino to roomba 7 mini-din port

int rxPin = 12;        // from arduino(10) to roomba pin(4) white wire
int txPin = 13;       // to arduino(11) from roomba pin(3) red wire
int ddPin = 5;        // device detect used to wake up roomba

//new software port to emulate hardware port
//use software because roomba's tx doesnt always carry enough
//voltage to meet arduino's requiremnts
  SoftwareSerial mySerial(rxPin, txPin);  

int WO = 16; //wheel offset for drive wheels  

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); //starts serial port for the serial monitor
    delay(1500);
  Serial.println("Hello World"); //prints message to the monitor
    delay(100); //allows for sucessful output to the serial monitor
  pinMode (ddPin, OUTPUT);  //set ddPin(5) to output from arduino

  mySerial.begin(19200); //start software serial for communication to roomba
    delay(1000);

  if (mySerial.available()){ //listens to roomba
      Serial.println(mySerial.read()); //writes to usb input from software serial if connected to laptop
  }//end of if

  //wake uo roomba with series of pulses to the dd pin on the roomba
  wakeUp();
  Serial.println("I'm awake.");

  //starts the IO for other commands
  mySerial.write(128);
  delay(1000);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  //put into safe mode
    mySerial.write(131);
    Serial.println("I'm in safe mode");
    delay(1000);

  //These commented out functions are the functions for the assignment
  //square (250, 2000, 1500);
  //circle(700, 250);
  //triangle(250, 1000);
  //stumble();
  
  //mySerial.write(173); //stops loop
}

/***********************************************
 * direct drive wheel motors
 * 4 bytes are
 * right velocity high byte
 * right velocity low byte
 * left velocity high byte
 * left velocitn low byte
 * 
 * examples of both bitwise shift right and highByte, lowByte
 * to parse the argument into 2 bytes
 * 
 * constrain is used to ensure that the parameter values are within 
 * range for driving roomba's wheels
 */
void driveWheels(int right, int left)
{
  constrain(right, -500, 500);
  constrain(left, -500, 500);

  Serial.println("Driving");
  
  mySerial.write(145);

  mySerial.write(highByte(right));
  mySerial.write(lowByte(right));
  mySerial.write(highByte(left));
  mySerial.write(lowByte(left));
 } // end drive wheels

 /*******************************************
 * stop roomba, send drive opcode (137) 
 * with all zero arguments
 ********************************************/
void stopDrive(void){
  mySerial.write(byte(137));
  mySerial.write(byte(0));
  mySerial.write(byte(0));
  mySerial.write(byte(0));
  mySerial.write(byte(0));  
} // end stopdrive()

/*******************************************
 * wake up roomba by pulsing dd pin 5 high low high
 ********************************************/
 void wakeUp (void)
{
  //setWarningLED(ON);
  //Serial.println("wake up, wake up.");
  digitalWrite(ddPin, HIGH);
  delay(100);
  digitalWrite(ddPin, LOW);
  delay(500);
  digitalWrite(ddPin, HIGH);
  delay(2000);
}

/*
 * Makes roomba move in a square
 * speed in mm/s, d1 & d2 in mm
 */
 void square (int sS, int d1, int d2){
  Serial.println("Square");
  
  int sL = sS; //speed for left
  int sR = sS + WO; //speed for right
  
  int angle = 90; //angle of turn
  
  int s1Time = d1/sS * 1000; //caculates turning time one
  int s2Time = d2/sS * 1000; //calculates turning time two
  
  int st = 0; //speed for turning

  //straight
  driveWheels(sR, sL);
    delay(s1Time);
    stopDrive();
  //turn
  driveWheels(st, sL);
    AngleDetect(angle);
  //straight
  driveWheels(sR, sL);
    delay(s2Time);
    stopDrive();
  //turn
  driveWheels(st, sL);
    AngleDetect(angle);
  //straight
  driveWheels(sR, sL);
    delay(s1Time);
    stopDrive();
  //turn
  driveWheels(st, sL);
    AngleDetect(angle);
  //straight
  driveWheels(sR, sL);
    delay(s2Time);
    stopDrive();
  //turn
  driveWheels(st, sL);
    AngleDetect(angle);

  stopDrive(); //stops movement
 }//end of square

/*
 * Makes roomba move in a circle
 * r = radius in mm
 * v = velocity in mm/s
 */
 void circle (int r, int v){
  Serial.println("Circle");
  
  int pi = 3.14159265358979;
  int t = (2*pi*r)/v*1000; //calculates time of movement

   mySerial.write(137);
   mySerial.write(highByte(v));
   mySerial.write(lowByte(v));
   mySerial.write(highByte(r));
   mySerial.write(lowByte(r));
   delay(t);

   stopDrive(); //stops movement
 }//end of circle

 /*
  * makes roomba move in a triangle
  * v = velocity in mm/s
  * d = distance in mm
  */
  void triangle (int v, int d){
    Serial.println("Triangle");
  
    int vt = 0; //speed for turn
    int vR = v + WO; //speed for right
    int vL = v; //speed for left
    
    int angle = 130; //angle of turn
    
    int t = d/v * 1000; //time of straight movement

    //straight
    driveWheels(vR,vL);
      delay(t);
      stopDrive();
    //turn
    driveWheels(vt, v);
      AngleDetect(angle);
    //straight
    driveWheels(vR,vL);
      delay(t);
      stopDrive();
    //turn
    driveWheels(vt, v);
      AngleDetect(angle);
    //straight
    driveWheels(vR,vL);
      delay(t);
      stopDrive();
    //turn
    driveWheels(vt, v);
      AngleDetect(angle);

    stopDrive(); //stops movement
    
  }//end of triangle

  /*
   * uses bump sensor to roughly navigate to move around
   */
   void stumble (){
    int v = 400; //speed
    int b = false; //boolean for if there was a bump

    //drives straight
    driveWheels(v, v);
      while (!b){ //continues until there is a bump
        b = bump();
      }//end of while
   }//end of stumble
  
  /*
  * used to calculate the angle of turn and used to replace time
  * delays. 
  * a = angle
  */
  void AngleDetect (int a){
    Serial.println("angle detect");
    
    double angle = 0; //done clockwise rn
    int data[2]; //holds data of the angle

    //clears counter
    mySerial.write(142);
    mySerial.write(20);

    bufferClear(); 
    
    while (angle >= -a){
     
        delay(30);
      //requests angle
      mySerial.write(142);
      mySerial.write(20);
        delay(30);

      //reads angle
      data[0] = mySerial.read();
      data[1] = mySerial.read();

      //formats bytes into a int
      angle += ((int) word (data[0],data[1]));
      }//end of while loop
  }//end of AngleDetect

  /*
   * Checkes to see if the roomba bumps into something and if 
   * true then reverse roomba for two seconds
   */
   boolean bump(){
      Serial.println("bump");
      
      int data = 0; //holds data from sensor
      int v = -200; //speed
      int vT = 200; //speed for term
      int t = 2000; //delay for straight
      int angle = 90; //angle 

      boolean bump = false;

    bufferClear();
      //requests bump sensor data
    mySerial.write(142);
    mySerial.write(7);
      delay(50);
    data = mySerial.read(); //reads data
    
    if(data>0){ //if there is a bump
      //straight backwards
      driveWheels(v, v);
        delay(t);
        stopDrive();
      //turns
      driveWheels(0, vT);
        AngleDetect(angle);
        stopDrive();

        bump = true;
    }//end of if 0

    return bump;
   }//end of bump

   /*
    * clears input buffer
    */
    void bufferClear(){
      while (mySerial.available()>0){ //contiunes while data in buffer
        int t = mySerial.read(); //reads in extra data
      }//end of while loop
    }//end of bufferClear

