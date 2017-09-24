//Author : Zabir Al Nazi

//Core Master P(ID)
//Format of PID function :-> PID(setPoint,curPoint) -> motor_drive()

//Before Competition -> Tuning
#define M_S 135 //max PID speed max possible 255
#define KP 95
#define KD 120
#define SFS 120 //SMALL FORWARD SPEED
#define TS 135 // Turn speed
//Switch :

#define ON 1
#define OFF 0

//Mood :

#define COMP_MOOD ON

#define BLACK_ON_WHITE ON

//Sensors

#define NUM_SENSORS 6 // number of sensors used

//right ->motor 1
//left -> motor 2
#define Md1 3  // Enable Pin for motor 1 , Motor driver  right motor
#define Md2 8 // Enable Pin for motor 2 , Motor driver left motor
//M1 Left L2 Right
#define M11 4  // Control pin 1 for motor 1
#define M12 5 // Control pin 2 for motor 1
#define M21 7  // Control pin 1 for motor 2
#define M22 6  // Control pin 2 for motor 2

#define rLED 32
#define mLED 33
#define lLED 34
#define endLED 35

#define s1 42
#define s2 43
#define s3 44
#define s4 45

#define SONAR_MODE OFF
int LSpins[] = {34, 32, 30, 28, 26, 24}; //left to right

#define left_turn_pin 36
#define right_turn_pin 22


int max_speed;

//end point condition

bool isEndPoint;

bool isStart;

bool fullWhiteSurface;

double Light_S_input;

double L_S_setPoint;

double PID_output;
int right_speed;
int left_speed;


double proportional;      // Replace set_point by your set point
double integral;
double derivative;
double last_proportional;

double Kp_Light_S;
double Kd;
double Ki;

unsigned int sensorValues[NUM_SENSORS];

void setup()
{
  delay(500);
  Serial.begin(9600); // set the data rate in bits per second for serial data transmission
  delay(1000);


  pinMode(Md1, OUTPUT);
  pinMode(Md2, OUTPUT);

  pinMode(M11, OUTPUT);
  pinMode(M12, OUTPUT);
  pinMode(M21, OUTPUT);
  pinMode(M22, OUTPUT);

  //LED DEBUGGING
  pinMode(rLED, OUTPUT);
  pinMode(mLED, OUTPUT);
  pinMode(lLED, OUTPUT);
  pinMode(endLED, OUTPUT);

  for (int i = 0; i < NUM_SENSORS; i++)
    pinMode(LSpins[i], INPUT);

  //END POINT
  pinMode(s1, INPUT);
  pinMode(s2, INPUT);
  pinMode(s3, INPUT);
  pinMode(s4, INPUT);


  integral = 0;
  L_S_setPoint = 3.5 ; //-100 0 100


  if (COMP_MOOD == ON)
  {
    max_speed = M_S;
  }
  else
  {
    max_speed  = 140;
  }

  isStart = true;

  Kp_Light_S = KP; //130NOT    160

  Kd = KD;
  Ki = 0.000;
}

bool endFunction()
{
  int sum_sensor = digitalRead(s1) + digitalRead(s2) + digitalRead(s3) + digitalRead(s4);
  if (sum_sensor >= 3)
    return true;
  return false;
}
void startMotion()
{
  digitalWrite(endLED, LOW);
  digitalWrite(rLED, LOW);
  digitalWrite(mLED, LOW);
  digitalWrite(lLED, HIGH);
  delay(800);
  digitalWrite(endLED, LOW);
  digitalWrite(rLED, LOW);
  digitalWrite(mLED, HIGH);
  digitalWrite(lLED, LOW);
  delay(800);
  digitalWrite(endLED, LOW);
  digitalWrite(rLED, HIGH);
  digitalWrite(mLED, LOW);
  digitalWrite(lLED, LOW);
  delay(800);
}
void L_sensors_read()
{
  Light_S_input = 0 ;

  fullWhiteSurface = false ;

  double tt = 0.00 ;
  double temp = 0 ;
  //Serial.print(digitalRead(36));
  //Serial.print(' ');
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    sensorValues[i] = digitalRead(LSpins[i]);
    //Serial.print(sensorValues[i]);
    //Serial.print(" ");
  }

  if (BLACK_ON_WHITE)
  {

    for (int i = 0; i < NUM_SENSORS; i++)
    {
      sensorValues[i] = !sensorValues[i];
      //  Serial.print(sensorValues[i]);
      // Serial.print(" ");
    }

  }


  for (int i = 0; i < NUM_SENSORS; i++)
  {

    //  Serial.print(sensorValues[i]);
    // Serial.print(" ");
  }

  // Serial.print(digitalRead(22));
  // Serial.print(' ');
  // Serial.println();


  for (int i = 0; i <= 5; i++)
  {

    temp += (sensorValues[i]) * (i + 1) * 100.0;

    tt += (sensorValues[i]) * 100.0;

    //Conditions for end point



  }
  if (tt > 0.01) {
    Light_S_input = temp / tt;
  }
  else
  {
    fullWhiteSurface = true;
    Light_S_input = L_S_setPoint;
  }



  //Serial.println(Light_S_input);

  //delay(50);




}




void calc_turn()   //Restricting the PID_output between max_speed.
{
  if (PID_output < -max_speed)
  {
    PID_output = -max_speed;
  }
  else if (PID_output > max_speed)
  {
    PID_output = max_speed;
  }

  if (PID_output > 0)   //Too much left need to go right
  {
    right_speed = max_speed - PID_output ;
    left_speed = max_speed ;
  }
  else
  {
    //Need to go left
    right_speed = max_speed;
    left_speed = max_speed + PID_output;


  }

  //Serial.print("Right : ");Serial.print(right_speed);
  //Serial.print(" Left : ");Serial.println(left_speed);
  //delay(200);

}

void master_PID(double setPoint, double curPoint, double Kp)
{

  proportional  = curPoint - setPoint;
  integral += proportional;
  derivative = proportional - last_proportional;
  last_proportional = proportional;

  PID_output = (double)(proportional * Kp + integral * Ki + derivative * Kd);
  // Serial.print("Error  pid output : ");
  // Serial.println(PID_output);

}


///////////-------------------------------------------------------------------------------------------------------------

///////////-------------------------------------------------------------------------------------------------------------

void motor_drive(int right_speed, int left_speed )
{
  // Drive motors according to the calculated values for a turn


  digitalWrite(M11, HIGH);
  digitalWrite(M12, LOW);

  digitalWrite(M21, HIGH);
  digitalWrite(M22, LOW);

  analogWrite(Md1, right_speed);
  analogWrite(Md2, left_speed);

  delay(4);



  /*
    Serial.print("Right : ");
    Serial.println(right_speed);
    Serial.print("Left : ");
    Serial.println(left_speed);

    delay(500);           // Optional
  */
}
void right_forward() //not used, just for testing
{
  //digitalWrite(M11, HIGH);
  // digitalWrite(M12, LOW);

  digitalWrite(M11, HIGH);
  digitalWrite(M12, LOW);

  //analogWrite(Md1, 20);


  analogWrite(Md1, 0);
  delay(1000);
  analogWrite(Md1, 255);
  delay(1000);


  analogWrite(Md1, 100);
  delay(1000);

  analogWrite(Md1, 200);
  delay(1000);


  digitalWrite(M21, HIGH);
  digitalWrite(M22, LOW);

  //analogWrite(Md1, 20);


  analogWrite(Md2, 0);
  delay(1000);
  analogWrite(Md2, 255);
  delay(1000);


  analogWrite(Md2, 100);
  delay(1000);

  analogWrite(Md2, 200);
  delay(1000);

}
bool left_on()
{
  if (BLACK_ON_WHITE && !digitalRead(left_turn_pin))

    return 1;

  if (!BLACK_ON_WHITE && digitalRead(left_turn_pin))
  {
    return 1;
  }
  return 0;
}
bool right_on()
{
  if (BLACK_ON_WHITE && !digitalRead(right_turn_pin))
  {
    return 1;
  }
  if (!BLACK_ON_WHITE && digitalRead(right_turn_pin))
  {
    return 1;
  }
  return 0;
}
void small_forward() //risky because of using delay
{
  digitalWrite(M11, HIGH);
  digitalWrite(M12, LOW);
  digitalWrite(M21, HIGH);
  digitalWrite(M22, LOW);
  analogWrite(Md1, SFS);
  analogWrite(Md2, SFS);
  delay(8);

}
void turn_left()
{

  //  while (true) {
  //    L_sensors_read();
  //    if (!sensorValues[0])
  //      break;
  //    small_forward();
  //  }
  while (true)
  {
    L_sensors_read();

    if (fullWhiteSurface)
      break;
    small_forward();
    delay(5);
  }

  while (true)
  {
    L_sensors_read();
   
   
      if (sensorValues[3] || sensorValues[2])
        return;
    digitalWrite(M11, HIGH);
    digitalWrite(M12, LOW);

    digitalWrite(M21, LOW);
    digitalWrite(M22, HIGH);

    analogWrite(Md1, TS);
    analogWrite(Md2, TS);

    delay(5);
  }
}
void turn_right()
{
  //  while (true) {
  //    L_sensors_read();
  //    if (!sensorValues[5])
  //      break;
  //    small_forward();
  //  }
  while (true)
  {
    L_sensors_read();

    if (fullWhiteSurface)
      break;
    small_forward();
    delay(4);
  }

  while (true)
  {
    L_sensors_read();
      if (sensorValues[2] || sensorValues[3])
        return;
    digitalWrite(M11, LOW);
    digitalWrite(M12, HIGH);

    digitalWrite(M21, HIGH);
    digitalWrite(M22, LOW);

    analogWrite(Md1, TS);
    analogWrite(Md2, TS);

    delay(4);
  }
}
void loop()
{

  if (isStart)
  {
    // startMotion();
    isStart = false;
  }

  // read raw sensor values

  L_sensors_read();

  if (right_on() )//&& !left_on())
  {
    //small_forward();

    L_sensors_read();
    if (right_on() )//&& !left_on())
    {
      //        digitalWrite(endLED, LOW);
      //        digitalWrite(rLED, HIGH);
      //        digitalWrite(mLED, LOW);
      //        digitalWrite(lLED, LOW);
      //motor_drive(0, 0);
      //delay(2000);
      turn_right();
      Serial.println("RT");
    }
  }

  else if (left_on()) //&& !right_on())
  {
    //small_forward();

    L_sensors_read();
    if (left_on())// && !right_on())
    {
      //        digitalWrite(endLED, LOW);
      //        digitalWrite(rLED, LOW);
      //        digitalWrite(mLED, LOW);
      //        digitalWrite(lLED, HIGH);
      turn_left();
      //motor_drive(0, 0);
      //delay(2000);
      Serial.println("LT");
    }
  }

  if (endFunction() && false)
  {
    digitalWrite(endLED, HIGH);
    digitalWrite(rLED, LOW);
    digitalWrite(mLED, LOW);
    digitalWrite(lLED, LOW);

    motor_drive(5, 5);
    delay(1000);

  }
  //  digitalWrite(endLED, LOW);
  //  digitalWrite(rLED, LOW);
  //  digitalWrite(mLED, HIGH);
  //  digitalWrite(lLED, LOW);

  L_sensors_read();
  master_PID( L_S_setPoint , Light_S_input , Kp_Light_S );
  calc_turn();
  motor_drive(right_speed, left_speed);
  Serial.print(left_speed);
  Serial.print(' ');
  Serial.println(right_speed);


}
