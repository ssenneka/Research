int SpeakerPin = 8;
int incomingByte;
int counter = 0;
int SolePin = 6;
#define LEDPIN 13
#define SENSORPIN A0
int VALID = 0;
int dispt = 25;
unsigned long CurrentTime = 0;
unsigned long TimeDrink = 0;
int SensorState = 0;

void setup() {
  // put your setup code here, to run once:
Serial.begin(9600);
pinMode(SpeakerPin,OUTPUT);
pinMode(SolePin, OUTPUT);
pinMode(LEDPIN, OUTPUT);
pinMode(SENSORPIN, INPUT);
digitalWrite(SENSORPIN, HIGH);
digitalWrite(SolePin, LOW);



}

void loop() {
  // put your main code here, to run repeatedly:
  CurrentTime = millis();

  SensorState = analogRead(SENSORPIN);
  //Serial.println(SensorState);

    incomingByte = Serial.read();
    if (incomingByte == 'H') 
    {
      digitalWrite(LEDPIN, HIGH);
      VALID = 1;
    }
    if((SensorState > 250) and (VALID == 1)){
      digitalWrite(SolePin, HIGH);
      digitalWrite(LEDPIN, LOW);
      delay(dispt);
      digitalWrite(SolePin, LOW);
     // Serial.println("HIGH");
      counter = counter + 1;
      VALID = 0;
      Serial.print('P');
    }
    if (incomingByte == 'C')
    {
      digitalWrite(SolePin, HIGH);
    }
    if (incomingByte == 'L') 
    {
      digitalWrite(SolePin, LOW);
      digitalWrite(LEDPIN, LOW);
      VALID = 0;
      Serial.print('D');
    }
    if (incomingByte == 'N')
    {
      Serial.println(counter);
      counter = 0;
    }
    if (incomingByte == 'T')
    {
      digitalWrite(SolePin, HIGH);
      delay(dispt);
      digitalWrite(SolePin, LOW);
    }
 // }
}
