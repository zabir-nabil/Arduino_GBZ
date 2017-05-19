// ---------------------------------------------------------------------------
// Author: Zabir Al Nazi
// ---------------------------------------------------------------------------

#include <NewPing.h>

#define TRIGGER_PIN  12  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     13  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 100 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

//right ->motor 1
//left -> motor 2
#define Md1 10  // Enable Pin for motor 1 , Motor driver
#define Md2 9 // Enable Pin for motor 2 , Motor driver
//M1 Left L2 Right
#define M11 3  // Control pin 1 for motor 1
#define M12 2 // Control pin 2 for motor 1
#define M21 4  // Control pin 1 for motor 2
#define M22 5  // Control pin 2 for motor 2


#define cLED 11 //tooClose
#define mLED 7 //midRange
#define aLED 6 //away

int counter;
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.

void setup() {
  counter = 0;
  Serial.begin(9600); // Open serial monitor at 115200 baud to see ping results.


  pinMode(Md1, OUTPUT);
  pinMode(Md2, OUTPUT);

  pinMode(M11, OUTPUT);
  pinMode(M12, OUTPUT);
  pinMode(M21, OUTPUT);
  pinMode(M22, OUTPUT);

  //LED DEBUGGING
  pinMode(cLED, OUTPUT);
  pinMode(mLED, OUTPUT);
  pinMode(aLED, OUTPUT);


}
void forward()
{
  digitalWrite(M11, HIGH);
  digitalWrite(M12, LOW);
  digitalWrite(M21, HIGH);
  digitalWrite(M22, LOW);
  analogWrite(Md2, 4);
  delay(20);
  analogWrite(Md2, 4);
  delay(20);
}
void stop_motion()
{
  digitalWrite(M11, HIGH);
  digitalWrite(M12, LOW);
  digitalWrite(M21, HIGH);
  digitalWrite(M22, LOW);
  analogWrite(Md2, 4);
  delay(20);
  analogWrite(Md2, 4);
  delay(20);
}
void right_move()
{
while(1){
  digitalWrite(M11, LOW);
  digitalWrite(M12, HIGH);

  digitalWrite(M21, HIGH);
  digitalWrite(M22, LOW);

  analogWrite(Md1, 0);
  analogWrite(Md2, 0);

  delay(100);

}
}
void left_move()
{

  digitalWrite(M11, HIGH);
  digitalWrite(M12, LOW);

  digitalWrite(M21, LOW);
  digitalWrite(M22, HIGH);

  analogWrite(Md1, 200);
  analogWrite(Md2, 200);

  delay(100);

  
}
void right_forward()
{
  digitalWrite(M11, HIGH);
  digitalWrite(M12, LOW);

  digitalWrite(M21, HIGH);
  digitalWrite(M22, LOW);

  analogWrite(Md1, 4);
  analogWrite(Md2, 10);

  delay(20);

}
void left_forward()
{
  digitalWrite(M11, LOW);
  digitalWrite(M12, HIGH);

  digitalWrite(M21, LOW);
  digitalWrite(M22, HIGH);

  analogWrite(Md1, 20);
  analogWrite(Md2, 10);
  delay(10);


}
void sonar_move()
{

  delay(50);                     // Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.
  int dis_cm = sonar.ping_cm();
 // Serial.print(dis_cm);
 // Serial.println();
  if(dis_cm <=0 )
  {
    digitalWrite(cLED, LOW);
    digitalWrite(mLED, LOW);
    digitalWrite(aLED, HIGH);
    forward();
    
  }
  else if(dis_cm <= 10)
  {

    digitalWrite(cLED, HIGH);
    digitalWrite(mLED, HIGH);
    digitalWrite(aLED, HIGH);
    stop_motion();
    
  }
  else if(dis_cm <= 25)
  {
    digitalWrite(cLED, HIGH);
    digitalWrite(mLED, LOW);
    digitalWrite(aLED, LOW);
    if(counter%2)
    right_move();
    else
    left_move();
  }
  else if (dis_cm <= 35)
  {
    digitalWrite(cLED, LOW);
    digitalWrite(mLED, HIGH);
    digitalWrite(aLED, LOW);
    if(counter%2)
    right_move();
    else
    left_move;
  }
  else
  {
    digitalWrite(cLED, LOW);
    digitalWrite(mLED, LOW);
    digitalWrite(aLED, HIGH);
   forward();
  }

}

void loop() {
  
 sonar_move();
 //forward();
 counter++;
  
}
