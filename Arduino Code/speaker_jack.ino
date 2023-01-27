int SpeakerPin = 8;
int incomingByte;
int counter = 0;
int SolePin = 6;
#define LEDPIN 13
#define SENSORPIN A0
int VALID = 0;
int RewardDispense = 10;
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
 // Serial.println(SensorState);
  //Serial.println(VALID);
  SensorState = analogRead(SENSORPIN);
  //Serial.println(SensorState);
  //Serial.println(CurrentTime);
  //if(Serial.available() > 0) 
 // {
    incomingByte = Serial.read();
    if (incomingByte == 'H') 
    {
      tone(SpeakerPin,3000,500);
      VALID = 1;
    }
    if((SensorState > 70) and (VALID == 1)){
      digitalWrite(SolePin, HIGH);
      digitalWrite(LEDPIN, HIGH);
     // Serial.println("HIGH");
      TimeDrink = CurrentTime;
      VALID = 0;
    }
    digitalWrite(LEDPIN, LOW);
    if(CurrentTime - TimeDrink > RewardDispense){
        digitalWrite(SolePin, LOW);
     }
    if (incomingByte == 'L') 
    {
      digitalWrite(SolePin, LOW);
      VALID = 0;
    }
 // }
}
