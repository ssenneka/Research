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
int radius = 3;
RGBmatrixPanel matrix(A, B, C, D, CLK, LAT, OE, false);



void setup() {
  matrix.begin();
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  incomingbyte = Serial.read();
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
  if (incomingbyte == 'L')
  {
    matrix.fillRect(0,0,32,32,matrix.Color333(1,1,1));
  }
}
