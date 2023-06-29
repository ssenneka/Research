#include <RGBmatrixPanel.h>

#define CLK  8   // USE THIS ON ADAFRUIT METRO M0, etc.
//#define CLK A4 // USE THIS ON METRO M4 (not M0)
//#define CLK 11 // USE THIS ON ARDUINO MEGA
#define OE   9
#define LAT 10
#define A   A0
#define B   A1
#define C   A2
#define D   A3
int incomingbyte;
int radius = 2;
RGBmatrixPanel matrix(A, B, C, D, CLK, LAT, OE, false);


int incomingByte;
int counter = 0;
int SolePin = 12;
#define LEDPIN 13
#define SENSORPIN A5
int VALID = 0;
int dispt = 25;
unsigned long CurrentTime = 0;
unsigned long TimeDrink = 0;
int SensorState = 0;

void setup() {
  // put your setup code here, to run once:
  matrix.begin();
  Serial.begin(9600);
  Serial.begin(9600);
  pinMode(SolePin, OUTPUT);
  pinMode(LEDPIN, OUTPUT);
  pinMode(SENSORPIN, INPUT);
  digitalWrite(SENSORPIN, HIGH);
  //digitalWrite(SolePin, LOW);


}

void loop() {
  // put your main code here, to run repeatedly:
  incomingbyte = Serial.read();
  SensorState = analogRead(SENSORPIN);
  //Serial.println(SensorState);
  if (incomingbyte == 'A')
  {
    matrix.fillCircle(5,5,radius, matrix.Color333(1,1,1));
  }
  if (incomingbyte == 'B')
  {
    matrix.fillCircle(5,15,radius, matrix.Color333(1,1,1));
  }
  if (incomingbyte == 'C')
  {
    matrix.fillCircle(5,25,radius, matrix.Color333(1,1,1));
  }
  if (incomingbyte == 'D')
  {
    matrix.fillCircle(15,5,radius, matrix.Color333(1,1,1));
  }
  if (incomingbyte == 'E')
  {
    matrix.fillCircle(15,15,radius, matrix.Color333(1,1,1));
  }
  if (incomingbyte == 'F')
  {
    matrix.fillCircle(15,25,radius, matrix.Color333(1,1,1));
  }
  if (incomingbyte == 'G')
  {
    matrix.fillCircle(25,5,radius, matrix.Color333(1,1,1));
  }
  if (incomingbyte == 'H')
  {
    matrix.fillCircle(25,15,radius, matrix.Color333(1,1,1));
  }
  if (incomingbyte == 'I')
  {
    matrix.fillCircle(25,25,radius, matrix.Color333(1,1,1));
  }
  if (incomingbyte == 'O')
  {
    matrix.fillRect(0,0,32,32, matrix.Color333(0,0,0));
  }
  if (incomingbyte == 'R')
  {
    digitalWrite(LEDPIN, HIGH);
    VALID = 1;
  }
 
  if((SensorState > 20) and (VALID == 1))
  {
    digitalWrite(SolePin, HIGH);
    digitalWrite(LEDPIN, LOW);
    delay(dispt);
    digitalWrite(SolePin, LOW);
    // Serial.println("HIGH");
    counter = counter + 1;
    VALID = 0;
    Serial.print('P');
  }
  if (incomingbyte == 'L') 
  {
    digitalWrite(SolePin, LOW);
    digitalWrite(LEDPIN, LOW);
    VALID = 0;
    Serial.print('L');
  }
  if (incomingbyte == 'T')
  {
    digitalWrite(SolePin, HIGH);
    delay(dispt);
    digitalWrite(SolePin, LOW);
  }
  if (incomingbyte == 'K')
  {
    digitalWrite(SolePin, HIGH);
  }
  
}
