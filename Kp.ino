/*

Black line follower code


*/

float Kp=0,Ki=0,Kd=0;
float error=0, P=0, I=0, D=0, PID_value=0;
float previous_error=0, previous_I=0;
int sensor[5]={0, 0, 0, 0, 0};
int initial_motor_speed=100;

void read_sensor_values(void);
void calculate_pid(void);
void motor_control(void);

/*sensor
int Left_Sensor;
int Right_Sensor;
int Center_Sensor;
*/

//motor
        //right
int motor_r_forward = 7;
int motor_r_reverse = 6;
int motor_r_enable  =10;
        //left
int motor_l_forward = 9;
int motor_l_reverse = 8;
int motor_l_enable  =11;


void motor_run(int leftMotorSpeed,int rightMotorSpeed)  //motor speed b/w   -250  to 250
{
  if(leftMotorSpeed >= 0)

    {

        analogWrite(motor_l_enable, leftMotorSpeed);
        digitalWrite(motor_l_forward, HIGH);
        digitalWrite(motor_l_reverse, LOW);

    }

    else

    {   int j=250+rightMotorSpeed;
         Serial.println("backward");
         analogWrite(motor_l_enable, j);
        digitalWrite(motor_l_reverse, HIGH);
        digitalWrite(motor_l_forward ,LOW);
    }

    if(rightMotorSpeed >= 0)

    { 
        analogWrite(motor_r_enable,rightMotorSpeed);
        digitalWrite(motor_r_forward, HIGH);
        digitalWrite(motor_r_reverse, LOW);

    }

    else

    {  int n=250+rightMotorSpeed;

        analogWrite(motor_r_enable,n);
        digitalWrite(motor_r_forward, LOW);
        digitalWrite(motor_r_reverse, HIGH);

    }


}


void setup()
{
  
  
  Serial.begin(9600);
  pinMode(A0 , INPUT);
  pinMode(A1 , INPUT);
  pinMode(A2 , INPUT);
  pinMode(A3 , INPUT);
  pinMode(A4 , INPUT);
  
  pinMode(motor_r_forward, OUTPUT);
  pinMode(motor_r_reverse, OUTPUT);
  pinMode(motor_l_forward, OUTPUT);
  pinMode(motor_l_reverse, OUTPUT);
}

void loop()
{
  sensor[1]=analogRead(A0);
  sensor[3]=analogRead(A2);
  sensor[2]=analogRead(A1);
  sensor[0]=analogRead(A3);
  sensor[5]=analogRead(A4);



  }


}
  
