/* 
 * rosserial Subscriber Example
 * Blinks an LED on callback
 */
const byte motorpins[6] = {11,13,12,8,7,10}; //Motor pins for motor for motor driver
const byte RMotorFpin = motorpins[1];        //INT 1
const byte RMotorBpin = motorpins[2];        //INT 2
const byte LMotorFpin = motorpins[3];        //INT 3
const byte LMotorBpin = motorpins[4];        //INT 4
const byte RPWMpin = motorpins[0];           //ENA
const byte LPWMpin = motorpins[5];           //ENB
const byte encoderpinsB[2] = {4,5};          // Encoder B pins
const byte RencoderA = 2;                    // Interrupt pin Right encoder
const byte RencoderB = encoderpinsB[0];      // Right B pin
const byte LencoderA = 3;                    // Interrupt pin Left encoder
const byte LencoderB = encoderpinsB[1];      // Left B pin
const byte diameter = 136;                   // wheel diameter in mm
const int pulses_per_revolution = 663;       // encoder pulses per shaft revolution
int pulses[2];
byte last;
byte PWM = 255;
long Rdistancemm;
long Ldistancemm;
long Avgdistancemm;

#include <ros.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float32.h>

ros::NodeHandle  nh;
std_msgs::Float32 dist_msg;
ros::Publisher distance("distance", &dist_msg);
void changeDir( const std_msgs::UInt8& cmd_msg){

 switch (cmd_msg.data) {
    case 0:
      STOP();
      last = 0;
      break;
    case 1:
      FOWARD();
      if(last != 1){
        reset_counter();
        }
       last = 1;
       break;
    case 2:
       BACK();
       if(last != 2){
        reset_counter();
        }
       last = 2;
       break;
    case 3:
        RIGHT();
        if(last != 3){
        reset_counter();
        }
       last = 3;
        break;
    case 4:
        LEFT();
        if(last != 4){
        reset_counter();
        }
       last = 4;
        break;
    default:
        STOP();
        last = 0;
        reset_counter();
        break;
    }
    noInterrupts();
     Rdistancemm = (pulses[0]/pulses_per_revolution)*diameter*M_PI;
     Ldistancemm = (pulses[1]/pulses_per_revolution)*diameter*M_PI;
     interrupts();
     if(last==1){
     Avgdistancemm = (Rdistancemm + Ldistancemm)/2;
     dist_msg.data = Avgdistancemm;
     }
     else if (last==2){
      Avgdistancemm = -(Rdistancemm + Ldistancemm)/2;
     dist_msg.data = Avgdistancemm;
     }
     else if(last == 3){
      dist_msg.data = Ldistancemm;
     }
     else if (last == 4){
      dist_msg.data = Ldistancemm;
     }
      distance.publish( &dist_msg );
      dist_msg.data = PWM;
      distance.publish( &dist_msg );
 }
void changePWM( const std_msgs::UInt8& pwm_msg){
  PWM = pwm_msg.data;
  dist_msg.data = PWM;
  distance.publish( &dist_msg );
}

ros::Subscriber<std_msgs::UInt8> dir("dir", &changeDir );
ros::Subscriber<std_msgs::UInt8> pwm("PWM", &changePWM );

void setup()
{ 
   REncoderInit();
  LEncoderInit();
  MotorInit();
  nh.initNode();
  nh.subscribe(dir);
  nh.subscribe(pwm);
  nh.advertise(distance);
}

void loop()
{  
  nh.spinOnce();
  delay(1);
}
void MotorInit(){
  for(byte i=0; i<6;i++)
  pinMode(motorpins[i],OUTPUT);
}
void RMotorF(byte PWM){
  digitalWrite(RMotorFpin,HIGH);
  digitalWrite(RMotorBpin,LOW);
  analogWrite(RPWMpin, PWM);
}
void LMotorF(byte PWM){
  digitalWrite(LMotorFpin,HIGH);
  digitalWrite(LMotorBpin,LOW);
  analogWrite(LPWMpin, PWM);
}
void RMotorB(byte PWM){
  digitalWrite(RMotorBpin,HIGH);
  digitalWrite(RMotorFpin,LOW);
  analogWrite(RPWMpin, PWM);
}
void LMotorB(byte PWM){
  digitalWrite(LMotorBpin,HIGH);
  digitalWrite(LMotorFpin,LOW);
  analogWrite(LPWMpin, PWM);
}
void RMotorStop(){
  digitalWrite(RMotorFpin,LOW);
  digitalWrite(RMotorBpin,LOW);
  analogWrite(RPWMpin, 0);
}
void LMotorStop(){
  digitalWrite(LMotorFpin,LOW);
  digitalWrite(LMotorBpin,LOW);
  analogWrite(LPWMpin, 0);
}
void FOWARD(){
  int L = 0;
  int R = 0;
  RMotorF(PWM-R);
  LMotorF(PWM-L);
  if (pulses[0] > pulses [1]) R++;
  else if(pulses [1] > pulses[0]) L++;
}
void BACK(){
  int L = 0;
  int R = 0;
  RMotorB(PWM-R);
  LMotorB(PWM-L);
  if (pulses[0] < pulses [1]) R++;
  else if(pulses [1] < pulses[0]) L++;
}
void STOP(){
  RMotorStop();
  LMotorStop();
}
void LEFT(){
  int L = 0;
  int R = 0;
  RMotorB(PWM-R);
  LMotorF(PWM-L);
  if (-pulses[0] > pulses [1]) R++;
  else if(pulses [1] > -pulses[0]) L++;
  }
void RIGHT(){
  int L = 0;
  int R = 0;
    RMotorF(PWM-R);
    LMotorB(PWM-L);
    if (pulses[0] > -pulses [1]) R++;
    else if(-pulses [1] > pulses[0]) L++;
}
void reset_counter()
{
  for(byte i=0; i<2;i++){
    pulses[i] = 0;
  }
}
void REncoderInit(){
  pinMode(RencoderB,INPUT);
  attachInterrupt(digitalPinToInterrupt(RencoderA), Rpulsecounter, RISING);
}
void LEncoderInit(){
  pinMode(LencoderB,INPUT);
  attachInterrupt(digitalPinToInterrupt(LencoderA), Lpulsecounter, RISING);
}
void Rpulsecounter()
{
    int val = digitalRead(encoderpinsB[0]);
    if(val == LOW)pulses[0]--;
    else if(val == HIGH) pulses[0]++;
}
void Lpulsecounter()
{
    int val = digitalRead(encoderpinsB[1]);
    if(val == LOW)pulses[1]--;
    else if(val == HIGH) pulses[1]++;
}
