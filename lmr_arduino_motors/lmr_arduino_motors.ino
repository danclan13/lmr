// lmr project arduino motors - old bootlogger

#define EI_ARDUINO_INTERRUPTED_PIN

#include <FastPID.h>
#include <EnableInterrupt.h>
#include <Wire.h>

int motor[3];
volatile boolean rFlag = false;
volatile byte i2cdata[256];
uint8_t writeBufferIndex;
uint8_t readBufferIndex;

unsigned long i2ctimeout = 300000;
unsigned long i2ct = 0;
int dir=0;
int spd=0;
int dir_old=0;
int spd_old=0;

//Define Variables we'll be connecting to
int Input1, Output1, Input2, Output2, Input3, Output3;
int Setpoint1 = 0;
int Setpoint2 = 0;
int Setpoint3 = 0;
int sign1 = 1;
int sign2 = 1;
int sign3 = 1;



//Specify the links and initial tuning parameters
float Kp=10.8, Ki=2.6, Kd=0.56, Hz = 100;
FastPID PID1(Kp, Ki, Kd, Hz, 9, true);
FastPID PID2(Kp, Ki, Kd, Hz, 9, true);
FastPID PID3(Kp, Ki, Kd, Hz, 9, true);

#define INTEG_MAX    250
#define INTEG_MIN    -250

//Sensor 
  const int hall_11 = 2;
  const int hall_12 = 8;
  const int hall_21 = 4;
  const int hall_22 = 12;
  const int hall_31 = 7;
  const int hall_32 = 13;
  unsigned long t1, t1_updated, t2, t2_updated, t3, t3_updated; 
  unsigned long dt1, dt2, dt3;
  unsigned long braketime = 300000;
  unsigned rpm1, rpm2, rpm3; 

//Driver
  const int pwm_11 = 10; 
  const int pwm_12 = 11; 
  const int pwm_21 = 6; 
  const int pwm_22 = 9; 
  const int pwm_31 = 3; 
  const int pwm_32 = 5; 
  int in_11; 
  int in_12; 
  int in_21; 
  int in_22; 
  int in_31; 
  int in_32;  



void setup()
{
  Wire.begin(83);                // join i2c bus with address #8
  delay(100); 
  Wire.onReceive(receiveEvent); 

if (PID1.err()) {
    Serial.println("There is a configuration error!");
    for (;;) {}
  }
if (PID2.err()) {
    Serial.println("There is a configuration error!");
    for (;;) {}
  }
if (PID3.err()) {
    Serial.println("There is a configuration error!");
    for (;;) {}
  }

PID1.setOutputRange(-255, 255);
PID2.setOutputRange(-255, 255);
PID3.setOutputRange(-255, 255);

  Serial.begin(115200);

pinMode(pwm_11, OUTPUT);
pinMode(pwm_12, OUTPUT);
pinMode(pwm_21, OUTPUT);
pinMode(pwm_22, OUTPUT);
pinMode(pwm_31, OUTPUT);
pinMode(pwm_32, OUTPUT);

TCCR0B = TCCR0B & B11111000 | B00000011;
TCCR1B = TCCR1B & B11111000 | B00000011;
TCCR2B = TCCR2B & B11111000 | B00000011;


pinMode(hall_12, INPUT);
pinMode(hall_22, INPUT);
pinMode(hall_32, INPUT);
pinMode(hall_11, INPUT);
pinMode(hall_21, INPUT);
pinMode(hall_31, INPUT);

  enableInterrupt(hall_11, interruptFunction1,RISING);
  enableInterrupt(hall_21, interruptFunction2,RISING);  
  enableInterrupt(hall_31, interruptFunction3,RISING);

writeBufferIndex = 0;
readBufferIndex = 0;
}

void loop()
{ 
/****************************************************************
 * 
 *   begin I2C read and timeout funcitons
 * 
 ****************************************************************/
  
  if (rFlag == true)  // when data is availabe -> read it 
  {
    LoadBytes();
    
    rFlag = false;
    i2ct = micros();
    Serial.print(motor[0]);
    Serial.print(",");
    Serial.print(motor[1]);
    Serial.print(",");
    Serial.println(motor[2]);
  }

  if ((micros()-i2ct) > i2ctimeout)   // timeout for i2c signal
  {
    Serial.println("i2c communication failed");
    for (int i=0; i<3; i++){
      motor[i] = 0;}
    i2ct = micros();
    }

/****************************************************************
 * 
 *   end I2C read and timeout funcitons
 * 
 ****************************************************************/


/****************************************************************
 * 
 *   begin changing motor setpoints
 * 
 ****************************************************************/
// if speed or direction change requested change motor setpoints

  if(spd!=spd_old || dir!=dir_old){
    motorspd_Deg(spd,dir);
    spd_old=spd;
    dir_old=dir;
  }
  
  Setpoint1 = motor[0];
  Setpoint2 = motor[1];
  Setpoint3 = motor[2];

  
/****************************************************************
 * 
 *   end changing motor setpoints
 * 
 ****************************************************************/



/****************************************************************
 * 
 *   begin RPM and PID calculations
 * 
 ****************************************************************/
// calculate revolutions per minute

  dt1 = t1_updated - t1; 
  if (dt1 > braketime || (micros()-t1_updated) > braketime) 
    {rpm1 = 0;} 
  else 
    {rpm1 = (60000000.0 / (dt1 * 823.1))*sign1;}

  dt2 = t2_updated - t2; 
  if (dt2 > braketime || (micros()-t2_updated) > braketime) 
    {rpm2 = 0;} 
  else 
    {rpm2 = (60000000.0 / (dt2 * 823.1))*sign2;}

  dt3 = t3_updated - t3; 
  if (dt3 > braketime || (micros()-t3_updated) > braketime) 
    {rpm3 = 0;} 
  else 
    {rpm3 = (60000000.0 / (dt3 * 823.1))*sign3;}

  Input1 = rpm1;
  Input2 = rpm2;
  Input3 = rpm3;

// get output valuues form PID loop
  
  int16_t Output1 = PID1.step(Setpoint1, Input1);
  int16_t Output2 = PID2.step(Setpoint2, Input2);
  int16_t Output3 = PID3.step(Setpoint3, Input3);

  motor_write(Output1,Output2,Output3);
/****************************************************************
 * 
 *   begin RPM and PID calculations and write to motors
 * 
 ****************************************************************/
 


}


void motorspd_Deg(int spd, int angle){
  // create intermidiate angles
  int angle_1 = PI/3+angle*PI;
  int angle_2 = PI/3-angle*PI;
  int angle_3 = angle*PI;
  // calculate motor speed settings
  motor[1] = spd*cos(angle_1);
  motor[2] = spd*cos(angle_2);
  motor[3] = -spd*cos(angle_3);
}

void motor_write(int16_t Output1,int16_t Output2,int16_t Output3){
  // reorder output pins depending on PID loop output

  if (Output1 < 0) 
    {Output1 = -Output1;
    in_11 = pwm_12;
    in_12 = pwm_11;}
   else 
    {in_11 = pwm_11;
    in_12 = pwm_12;}

      if (Output2 < 0) 
    {Output2 = -Output2;
    in_21 = pwm_22;
    in_22 = pwm_21;}
   else 
    {in_21 = pwm_21;
    in_22 = pwm_22;}

      if (Output3 < 0) 
    {Output3 = -Output3;
    in_31 = pwm_32;
    in_32 = pwm_31;}
   else 
    {in_31 = pwm_31;
    in_32 = pwm_32;}

// write to motors

  analogWrite(in_11, Output1);
  analogWrite(in_12, 0);


  analogWrite(in_21, Output2);
  analogWrite(in_22, 0);
  

  analogWrite(in_31, Output3);
  analogWrite(in_32, 0);
}

/****************************************************************
 * 
 *   begin interrupts
 * 
 ****************************************************************/

  char lastbyte;
  char lastbyte2;
void LoadBytes(void){
  while(writeBufferIndex!=readBufferIndex) 
    {
      if (i2cdata[readBufferIndex] == 251)
      {
        readBufferIndex++;
        motor[0] = i2cdata[readBufferIndex]-80;
      }
      else if (i2cdata[readBufferIndex] == 252)
      {
        readBufferIndex++;
        motor[1] = i2cdata[readBufferIndex]-80;
      }
      else if (i2cdata[readBufferIndex] == 253)
      {
        readBufferIndex++;
        motor[2] = i2cdata[readBufferIndex]-80;
      }
      else if(i2cdata[readBufferIndex] == 241){
        readBufferIndex++;
        spd = i2cdata[readBufferIndex];   // write speed value
      }
      else if(i2cdata[readBufferIndex] == 242){
        readBufferIndex++;
        dir = i2cdata[readBufferIndex];   // write angle value
        
      }
       readBufferIndex++; 
      
    }
}

void receiveEvent(int howMany) {
//  for (int i=0; i<howMany && i<8; i++)
//  {
//    i2cdata[i] = Wire.read(); // receive byte as a character
//  }
//  rFlag = true;
//
  while (howMany>0)
  {
    howMany--;
    i2cdata[writeBufferIndex] = Wire.read(); // receive byte as a character
    lastbyte2=lastbyte;
    lastbyte = i2cdata[writeBufferIndex];
    writeBufferIndex++;
   
    if(lastbyte2 == '\n' && lastbyte == '\r' ){
      rFlag = true;
      break;
    }
  }
}

void interruptFunction1() {
   t1 = t1_updated; 
   t1_updated = micros();
   sign1 = -1 + (digitalRead(hall_12))*2;
   }

 void interruptFunction2() {
   t2 = t2_updated; 
   t2_updated = micros();
   sign2 = -1 + (digitalRead(hall_22))*2;
 }

 void interruptFunction3() {
   t3 = t3_updated; 
   t3_updated = micros();
   sign3 = -1 + (digitalRead(hall_32))*2;
 }
 
/****************************************************************
 * 
 *   end interrupts
 * 
 ****************************************************************/
