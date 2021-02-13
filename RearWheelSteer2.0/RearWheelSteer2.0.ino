/***********************************
 * 
 * Name: Waliul Matin
 * Semester: FA19
 * Project Name: Rear Wheel Steering for FSAE car
 * Version: 2.0
 * Update log: Further improvements to the code from version 1 to make it stable
 */

 /////////////////////////////// ----------- Variables
const int pinPwmA = 6, pinPwmB = 5;
int desPos, accPos;                           //Variables for control 
int sF;                                       //Scale factor
int dE, iE;                                   //Derivative error and Integral error
int error, lasterror;                         //errors
const int kp = 1, ki = 0, kd = 0;             //PID gain constants 
int A_SIG=0, B_SIG=1;                         //Encoder signal variables 

 ///////////////////////////////

 void setup(){
  
  //TCCR0B = TCCR0B & 0B11111000|0X01;        //Changes the PWM frequency for the designated pins
  
  attachInterrupt(0, A_RISE, RISING);         //Connect A output to interrupt pin 1 (D2)
  attachInterrupt(1, B_RISE, RISING);         //Connect B output to interrupt pin 2 (D3)

  pinMode(pinPwmA, OUTPUT);                   //Initialize both PWM pins as digital outputs.
  pinMode(pinPwmB, OUTPUT);
  
  Serial.begin(9600);                         //Initializes Serial comm for potential debugging
  //Serial.println("Start");                    //Prints Start to indicate that device has initialized

  Initialize();                               //Calls upon the Initialize function
 }

 void loop(){                                 //Main loop containing all the run time code
  
  //Serial.println(sF);
  if((desPos >= 573) && (desPos <= 696)){     //Sets up condition so device will only control the motor when the steering wheel is in the desired range
    desPos = (analogRead(A0)+(offset(analogRead(A1))))*sF; //Converts the pot value to match the scale of the encoder. Uses the offset function and multiplies with the scale Factor
    Control((PID(desPos,accPos)));            //Calls upon the Control function with the return of the PID function as the input
  }
  else{
    Control(0);                               //Sets the pwm signal to 0 when steering wheel is not in desire range
  }
 }

 //////////////////////////////

 void Initialize(){                           //Function used to perfrom a set of calibration calculations to find absolute position of the actuators
  int potcent = 660;                          //Value when steering is at center position
  Control(255);                               //Commands the actuator to go full throttle one way
  delay(5000);                                //delays for 2 seconds to allow actuator to move
  accPos = 0;                                 //sets the encoder value to zero for reference
  Control(-255);                              //Commands the actuator to go full throttle the other way
  delay(5000);                                //delays for 2 seconds to allow actuator to move
  sF = (abs(accPos))/1024;                    //scale factor to convert pot value to be on the same scale as the encoder
  Serial.print("accPos:");
  Serial.println(accPos);
  Serial.print("sF:");
  Serial.println(sF);  
  int desPosint = potcent*sF;                 //sets the intial desPos value to center the wheels
  //Serial.println(sF);
  while(abs(error) >= 20){                    //run this loop while error is greater than 20
    Control((PID(desPosint,accPos)));         //stays in the while loop until error is less than 20 counts. This is necessary as the Control function needs to run multiple time to control the motor as intended
  }
  //Serial.println("Initialized");            //Prints out Initialized to indicate that device has finished initializing
 }

 int PID(int dP, int aP){                     //Function is used to return the PID terms for motor control with desired position and actual position as inputs
  error = dP - aP;                            //Find the error between the desired and actual positions
  
  dE = error - lasterror;                     //Change in error - Differential Component
  iE += error;                                //Culmination of error - Integral Component
  
  int pid = (kp * error) + (ki * iE) + (kd * dE); //Pid term is calculated by the given equation
  pid = constrain(pid, -255, 255);            //Constrains the pid value to accomodate 8 bit PWM pins of Arduino
  
  lasterror = error;                          //Saves the last error to calculate the differential component
  return pid;                                 //returns the pid value as the output of the function

 }

 void Control(int pid){                       //Function for motor control with pid as input
  if (pid > 0) {                              //if the pid term is positive
    analogWrite(pinPwmA, pid);                //outputs the PWM value through pinA
    analogWrite(pinPwmB, LOW);                //sets PWM through pinB to zero
  } 
 else if(pid < 0) {
    analogWrite(pinPwmA, LOW);                //sets PWM through pinA to zero
    analogWrite(pinPwmB, -pid);               //outputs the inverse of PWM value through pinB since a PWM value cannot be negative
  }

 }

 int offset(int off){                         //function to create the proper offset component for pot
  int result;                                 //creates the result variable
  result = (off-512)/4;                       //This marks the center of the 10 bit pot as zero and the magnitude is decreased by a factor of 4
  return result;                              //returns the result value as output of function
 }
 
//////////////////////////////////////////////// Interrrupt function - Increments or Decrements accPos count based on encoder signal reading: https://www.youtube.com/watch?v=v4BbSzJ-hz4
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
