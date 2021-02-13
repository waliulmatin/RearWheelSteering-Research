/***********************************
 * 
 * Name: Waliul Matin
 * Semester: FA19
 * Project Name: Rear Wheel Steering for FSAE car
 * Version: 2.1b
 * Update log: Final project code implemented in the system. This code was configured to control the right actuator specifically. 
 */

 /////////////////////////////// ----------- Variables
const int pinPwmA = 6, pinPwmB = 10;            
double desPos=0, accPos=0, pot=0;            //Variables for control
const double sF = 1.5;                       //Scale factor
double dE=0, iE=0;                           //Derivative error and Integral error
double error=0, lasterror=0;                 //errors
const double kp = 10, ki = 0, kd = 0;        //PID gain constants 
int A_SIG=0, B_SIG=1;                        //Encoder signal variables 

 ///////////////////////////////

 void setup(){
  
  //TCCR0B = TCCR0B & 0B11111000|0X01;        //Changes the PWM frequency for the designated pins
  
  attachInterrupt(0, A_RISE, RISING);         //Connect A output to interrupt pin 1 (D2)
  attachInterrupt(1, B_RISE, RISING);         //Connect B output to interrupt pin 2 (D3)

  pinMode(pinPwmA, OUTPUT);                   //Initialize both PWM pins as digital outputs.
  pinMode(pinPwmB, OUTPUT);
  
  Serial.begin(9600);                         //Initializes Serial comm for potential debugging
  //Serial.println("Start");                  //Prints Start to indicate that device has initialized

  Initialize();                               //Calls upon the Initialize function
 }
 ///////////////////////////////
 void loop(){                                 //Main loop containing all the run time code
 //Serial.println((PID(desPos,accPos)));
 //Serial.println(accPos);
  pot = (1024 - (analogRead(A0)));
  
  if((pot >= 220) && (pot <= 505)){           //Sets up condition so device will only control the motor when the steering wheel is in the desired range
    desPos = ((1024 - (analogRead(A0)))+(offset(analogRead(A1))))*sF; //Converts the pot value to match the scale of the encoder. Uses the offset function and multiplies with the scale Factor
    Control((PID(desPos,accPos)));            //Calls upon the Control function with the return of the PID function as the input
  }
  else{
    Control(0);                               //Sets the pwm signal to 0 when steering wheel is not in desire range
  }
 }

 //////////////////////////////

 void Initialize(){                           //Function used to perfrom a set of calibration calculations to find absolute position of the actuators
  Serial.println("Initializing...");
  Control(255);                               //Commands the actuator to go full throttle one way
  delay(5000);                                //delays for 2 seconds to allow actuator to move
  accPos = 1158;                              //sets the encoder value to zero for reference
  Serial.println("Initialized");              //Prints out Initialized to indicate that device has finished initializing
  
 }
 /////////////////////////////
 int PID(int dP, int aP){                     //Function is used to return the PID terms for motor control with desired position and actual position as inputs
  error = dP - aP;                            //Find the error between the desired and actual positions
  
  dE = error - lasterror;                     //Change in error - Differential Component
  iE += error;                                //Culmination of error - Integral Component
  //iE = constrain(iE, -50, 50);                //constrains the steady state error value 
  
  int pid = (kp * error) + (ki * iE) + (kd * dE); //Pid term is calculated by the given equation
  pid = constrain(pid, -255, 255);            //Constrains the pid value to accomodate 8 bit PWM pins of Arduino
  
  lasterror = error;                          //Saves the last error to calculate the differential component
  return pid;                                 //returns the pid value as the output of the function

 }
 /////////////////////////////
 void Control(int pid){                       //Function for motor control with pid as input
  if (pid > 0) {                              //if the pid term is positive
    analogWrite(pinPwmA, pid);                //outputs the PWM value through pinA
    analogWrite(pinPwmB, LOW);                //sets PWM through pinB to zero
  } 
 else if(pid < 0) {
    analogWrite(pinPwmA, LOW);                //sets PWM through pinA to zero
    analogWrite(pinPwmB, -pid);               //outputs the inverse of PWM value through pinB since a PWM value cannot be negative
  }
  else{
    analogWrite(pinPwmA, LOW);                //sets PWM through pinA to zero
    analogWrite(pinPwmB, LOW);    
  }

 }
 //////////////////////////////
 int offset(int off){                         //function to create the proper offset component for pot
  double result = (off-512)/4;                //This marks the center of the 10 bit pot as zero and the magnitude is decreased by a factor of 4
  return result;                              //returns the result value as output of function
 }
 
////////////////////////////////////////////// Interrrupt function - Increments or Decrements accPos count based on encoder signal reading: https://www.youtube.com/watch?v=v4BbSzJ-hz4
void A_RISE(){
 detachInterrupt(0);
 A_SIG=1;
 
 if(B_SIG==0)
 accPos++;//moving forward
 if(B_SIG==1)
 accPos--;//moving reverse

 attachInterrupt(0, A_FALL, FALLING);
}

void A_FALL(){
  detachInterrupt(0);
 A_SIG=0;
 
 if(B_SIG==1)
 accPos++;//moving forward
 if(B_SIG==0)
 accPos--;//moving reverse

 
 attachInterrupt(0, A_RISE, RISING);  
}

void B_RISE(){
 detachInterrupt(1);
 B_SIG=1;
 
 if(A_SIG==1)
 accPos++;//moving forward
 if(A_SIG==0)
 accPos--;//moving reverse


 attachInterrupt(1, B_FALL, FALLING);
}

void B_FALL(){
 detachInterrupt(1);
 B_SIG=0;
 
 if(A_SIG==0)
 accPos++;//moving forward
 if(A_SIG==1)
 accPos--;//moving reverse


 attachInterrupt(1, B_RISE, RISING);
}

////////////////////////////////////////////////////////////
