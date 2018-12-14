enum direction {CENTER, RIGHT, LEFT};               //defining by names for ease of use

#define Pin_1st_Motor 2                             //motor drive pin
#define Pin_2nd_Motor 5
#define Pin_Dir_1st_Motor 4                         //motor direction pin
#define Pin_Dir_2nd_Motor 6
#define Pin_speed_channel 20                        //channel interrupt pin
#define Pin_steer_channel 21

float steer_response = 0.06;       //three different functions can be used for control, change func number to be used at line 166  ( Motor_diff = func*() ) and upload 
                                   //       func1  range 0.01  ==> 0.9
                                   //       func2  range 0.01  ==> 0.1
                                   //       func3  range 0.1   ==> 5
                                   //steer_response value can be changed through serial screen ( no need to reupload)
                                   //steer_response value represents precentage of speed that should be braked ;
                                   //for values within range, bigger value means more aggressive steering (braking)

int Speed_1st_Motor = 0;
int Speed_2nd_Motor = 0;
int Motorspeed = 0;
int Motor_steer = 0;
int Motor_diff = 0;
int Motor_Dir = 1;                           //inital direction of rotation for motor 
int Motor_Dir_last = 1;                      //prev direction of rotation          
int speed_rx_pulse_width = 0;
int steer_rx_pulse_width = 0;
int rx_last_pwm_error = 0;                       //used for debugging in case remote pwm is out of bounds
unsigned long speed_reviever_pwm_pulse_width = 0;
unsigned long steer_reviever_pwm_pulse_width = 0;
unsigned long speed_reviever_pwm_pulse_width_last = 0;
unsigned long steer_reviever_pwm_pulse_width_last = 0;
int sp_reciever_ranges[4] = {1038,1430,1470,1850};    //speed channel Radio constant: 0-1 CCW speedm, 1-2 deadband, 2-3 CW speed
int st_reciever_ranges[4] = {1075,1450,1550,1950};

//first motor pins are configured as output ,
//then motordirection pins are configured ,
//then RC speed & steer channels are configured ,
//and interrupt is attached to their pins.

void setup() {
    pinMode(Pin_1st_Motor, OUTPUT);
    pinMode(Pin_2nd_Motor, OUTPUT);
    pinMode(Pin_Dir_1st_Motor, OUTPUT);
    pinMode(Pin_Dir_2nd_Motor, OUTPUT);   
    pinMode(Pin_speed_channel, INPUT);
    pinMode(Pin_steer_channel, INPUT);
    attachInterrupt(digitalPinToInterrupt(Pin_speed_channel),     // we rely on interrupts to change speed and direction values as querying pins on main loop is inefficient 
		    speed_ch_Interrupt,
		    CHANGE);
    attachInterrupt(digitalPinToInterrupt(Pin_steer_channel),
		    steer_ch_Interrupt,
		    CHANGE);
    digitalWrite(Pin_Dir_1st_Motor, LOW);                              //initial direction of rotation cw
    digitalWrite(Pin_Dir_2nd_Motor, HIGH);                             //motors have opposite intial direction of roatation so they are always reversed in code to mitigate this problem
    Serial.begin(9600);
}

void loop() {
  if(Motor_Dir != Motor_Dir_last){                    //stop motor then reverse so nothing is fried or broken if direction changes
    Serial.println("#### DIRECTION CHANGE ####");         
    Motor_Dir_last = Motor_Dir;
    analogWrite(Pin_1st_Motor, 0);
    analogWrite(Pin_2nd_Motor, 0);
    delay(150);
  }

  analogWrite(Pin_1st_Motor, Speed_1st_Motor);                 //change speed
  analogWrite(Pin_2nd_Motor, Speed_2nd_Motor);
  
                                                   
                                                  
  digitalWrite(Pin_Dir_1st_Motor, (HIGH ^ Motor_Dir)); //1st_motor direction of rotation was opposite to 2nd_motor
  digitalWrite(Pin_Dir_2nd_Motor, Motor_Dir);          //so to make them spin in the same dir i had to reverse one of them
  delay(50);
}

void speed_ch_Interrupt()
{
   speed_rx_pulse_width = micros() - speed_reviever_pwm_pulse_width_last;
   
   if(speed_rx_pulse_width>sp_reciever_ranges[0] and
      speed_rx_pulse_width<sp_reciever_ranges[3]){
     
       speed_reviever_pwm_pulse_width = speed_rx_pulse_width;
     
        if(speed_reviever_pwm_pulse_width>sp_reciever_ranges[0] and
	   speed_reviever_pwm_pulse_width<sp_reciever_ranges[1]){
	  
	    Motor_Dir = LOW;
            Motorspeed = map(speed_reviever_pwm_pulse_width,
			     sp_reciever_ranges[0],sp_reciever_ranges[1],
			     255,0);
	}
        if(speed_reviever_pwm_pulse_width>sp_reciever_ranges[2] and
	   speed_reviever_pwm_pulse_width<sp_reciever_ranges[3]){
	  
	    Motor_Dir = HIGH;
            Motorspeed = map(speed_reviever_pwm_pulse_width,
			     sp_reciever_ranges[2],sp_reciever_ranges[3],
			     0,255);
        }
        if(speed_reviever_pwm_pulse_width>sp_reciever_ranges[1] and
	   speed_reviever_pwm_pulse_width<sp_reciever_ranges[2]){
	  
            Motorspeed = 1;
        }
    }
   else{
        rx_last_pwm_error = speed_rx_pulse_width;
       }
   speed_reviever_pwm_pulse_width_last = micros();
    
   Motor_param_calc();
}


void steer_ch_Interrupt()
{
   steer_rx_pulse_width = micros() - steer_reviever_pwm_pulse_width_last;
  
   if(steer_rx_pulse_width>st_reciever_ranges[0] and
      steer_rx_pulse_width<st_reciever_ranges[3] ){
      
       steer_reviever_pwm_pulse_width = steer_rx_pulse_width;
      
        if(speed_reviever_pwm_pulse_width>st_reciever_ranges[0] and
           steer_reviever_pwm_pulse_width<st_reciever_ranges[1] ){
	
	    Motor_steer = LEFT;
	    Motor_diff = map(steer_reviever_pwm_pulse_width,
			     st_reciever_ranges[0],st_reciever_ranges[1],
			     0,255);
	}
        if(steer_reviever_pwm_pulse_width>st_reciever_ranges[2] and
	   steer_reviever_pwm_pulse_width<st_reciever_ranges[3] ){
	  
	    Motor_steer = RIGHT;
	    Motor_diff = map(steer_reviever_pwm_pulse_width,
			     st_reciever_ranges[2],st_reciever_ranges[3],
			     255,0);
        }
        if(steer_reviever_pwm_pulse_width>st_reciever_ranges[1] and
	   steer_reviever_pwm_pulse_width<st_reciever_ranges[2] ){
	  
            Motor_steer = CENTER;
	    Motor_diff = 0;
        }
    }
   else{
        rx_last_pwm_error = steer_rx_pulse_width;
       }
   steer_reviever_pwm_pulse_width_last = micros();
     
   Motor_param_calc();
}

/* motor_param_calc is called within the interrupt function so as to avoid a race condition as some values might get changed by a different interrupt amidst its execution ,
 this is not the right way to do it , but this is not production code so....... , it should be in the main function and interrupts should be disabled before it then
 reenabled after and it should be optimized a bit */ 


void Motor_param_calc()
{
  if (Serial.available() > 0) {
    steer_response = Serial.read();}                    //change steer_response from serial screen

  Motor_diff = func1();
  
  if( Motor_steer == LEFT){
    Speed_1st_Motor = Motorspeed - Motor_diff;
    Speed_2nd_Motor = Motorspeed; }
  
  else if( Motor_steer == RIGHT){
    Speed_1st_Motor = Motorspeed;
    Speed_2nd_Motor = Motorspeed - Motor_diff; }
  
  else if( Motor_steer == CENTER){
    Speed_1st_Motor = Motorspeed;
    Speed_2nd_Motor = Motorspeed; }
}


	     
//three different functions to try and calculate the braking force applied to the wheel closer to the turning curve's center

//a linear function
inline int func1()
{
  return Motor_diff * steer_response;
}

//a sigmoid squashing function spin-off to make a non-linear braking ratio
//subtracting from 255 is to align the function and get it in the first quadrant(result of exponential increases with motor_diff) 
inline int func2()
{
  return Motorspeed * steer_response / (1 + exp((255-Motor_diff * steer_response)/100));
}

//map function with in_min & out_min equal to 0
inline int func3()
{
  return Motor_diff * steer_response * 10 / 51 ;
}
