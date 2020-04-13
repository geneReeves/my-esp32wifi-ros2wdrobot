
#include <ros.h>
#include <std_msgs/String.h>
#include <WiFi.h>
#include <std_msgs/Int16.h>
#include <PID_v1.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Twist.h>
#include <ros/time.h>
#include <L298N.h>

                                                                            // WIFI CONFIGURATION
const char SSID[] = "ros-master-pi4D1C";
const char PASSWORD[] = "robotseverywhere";
IPAddress server(10,42,0,1); // e.g.: IPAddress server(192, 168, 1, 3);
const uint16_t serverPort = 11411;

WiFiClient client;

class WiFiHardware {
  public:
  WiFiHardware() {};

  void init() {
    client.connect(server, serverPort);
  }

  int read() {
    return client.read();
  }

  void write(uint8_t* data, int length) {
    for(int i=0; i<length; i++)
      client.write(data[i]);
  }

  unsigned long time() {
     return millis();
  }
};



                                                                            // ROBOT SPECIFIC PARAMETERS 

const double radius = 0.04;                   //Wheel radius, in m
const double wheelbase = 0.187;               //Wheelbase, in m
const double encoder_cpr = 990                //Encoder ticks or counts per rotation
const double speed_to_pwm_ratio = 0.00235     //Ratio to convert speed (in m/s) to PWM value. It was obtained by plotting the wheel speed in relation to the PWM motor command (the value is the slope of the linear function).
const double min_speed_cmd = 0.0882           //(min_speed_cmd/speed_to_pwm_ratio) is the minimum command value needed for the motor to start moving. This value was obtained by plotting the wheel speed in relation to the PWM motor command (the value is the constant of the linear function).

double speed_req = 0;                         //Desired linear speed for the robot, in m/s
double angular_speed_req = 0;                 //Desired angular speed for the robot, in rad/s

double speed_req_left = 0;                    //Desired speed for left wheel in m/s
double speed_act_left = 0;                    //Actual speed for left wheel in m/s
double speed_cmd_left = 0;                    //Command speed for left wheel in m/s 

double speed_req_right = 0;                   //Desired speed for right wheel in m/s
double speed_act_right = 0;                   //Actual speed for right wheel in m/s
double speed_cmd_right = 0;                   //Command speed for right wheel in m/s 
                        
const double max_speed = 0.4;                 //Max speed in m/s

int PWM_leftMotor = 0;                        //PWM command for left motor
int PWM_rightMotor = 0;                       //PWM command for right motor 

//initializing all the variables
#define LOOPTIME                      100     //Looptime in millisecond
const byte noCommLoopMax = 10;                //number of main loops the robot will execute without communication before stopping
unsigned int noCommLoops = 0;                 //main loop without communication counter

double speed_cmd_left2 = 0;      
unsigned long lastMilli = 0;



                                                                            // ROS DECLARATION
ros::NodeHandle_<WiFiHardware> nh;

std_msgs::String str_msg;
std_msgs::Int16 int_msg;
geometry_msgs::Vector3Stamped speed_msg; //create a "speed_msg" ROS message


                                                                            //ROS PUBLISHER
ros::Publisher chatter("chatter", &str_msg);
char hello[13] = "hello world!";

//create a publisher to ROS topic "speed" using the "speed_msg" type
ros::Publisher speed_pub("speed", &speed_msg); 



                                                                           // MOTOR PARAMETERS


//pin definition
// Motor A - left look from back
#define enable1Pin 13
#define motor1Pin1 27
#define motor1Pin2 26

// Motor B - right look from back
#define motor2Pin3 33;
#define motor2Pin4 32;
#define enable2Pin 12;



                                                                            // PID CONTROLLER PARAMETERS 

//PID constants

const double PID_left_param[] = { 2, 5, 1 }; //Respectively Kp, Ki and Kd for left motor PID
const double PID_right_param[] = { 2, 5, 1 }; //Respectively Kp, Ki and Kd for right motor PID

volatile float pos_left = 0;       //Left motor encoder position
volatile float pos_right = 0;      //Right motor encoder position

PID PID_leftMotor(&speed_act_left, &speed_cmd_left, &speed_req_left, PID_left_param[0], PID_left_param[1], PID_left_param[2], DIRECT);          //Setting up the PID for left motor
PID PID_rightMotor(&speed_act_right, &speed_cmd_right, &speed_req_right, PID_right_param[0], PID_right_param[1], PID_right_param[2], DIRECT);   //Setting up the PID for right motor


int PIN_ENCOD_A_MOTOR_LEFT = 34;  //Left motor encoder pin 34
int PIN_ENCOD_A_MOTOR_RIGHT = 35; //Right motor encoder pin 35



                                                                            // FUNCTION DEFINITION
void connectWiFi(){
  Serial.begin(115200);
  WiFi.begin(SSID,PASSWORD);
    Serial.print("WiFi connecting");

  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(100);
  }

  Serial.println(" connected");
  //nh.initNode();
  //nh.advertise(chatter);
  delay(10);  
}

void setupMotor(){

  L298N leftMotor(enable1Pin,motor1Pin1,motor1Pin2);
  L298N rightMotor(enable2Pin,motor2Pin3,motor2Pin4);

    //setting motor speeds to zero
  leftMotor.setSpeed(0);
  leftMotor.stop();
  rightMotor.setSpeed(0);
  rightMotor.stop();
  
  Serial.begin(115200);

  // testing
  Serial.print("Testing DC Motor...");
}

void setupPID(){
  
   //setting PID parameters
  PID_leftMotor.SetSampleTime(95);
  PID_rightMotor.SetSampleTime(95);
  PID_leftMotor.SetOutputLimits(-max_speed, max_speed);
  PID_rightMotor.SetOutputLimits(-max_speed, max_speed);
  PID_leftMotor.SetMode(AUTOMATIC);
  PID_rightMotor.SetMode(AUTOMATIC);
  
}

void setEncoder(){

  // Define the rotary encoder for left motor
  pinMode(PIN_ENCOD_A_MOTOR_LEFT, INPUT); 
  //pinMode(PIN_ENCOD_B_MOTOR_LEFT, INPUT); 
  digitalWrite(PIN_ENCOD_A_MOTOR_LEFT, HIGH);                // turn on pullup resistor
  //digitalWrite(PIN_ENCOD_B_MOTOR_LEFT, HIGH);
  attachInterrupt(0, encoderLeftMotor, RISING);

  // Define the rotary encoder for right motor
  pinMode(PIN_ENCOD_A_MOTOR_RIGHT, INPUT); 
  //pinMode(PIN_ENCOD_B_MOTOR_RIGHT, INPUT); 
  digitalWrite(PIN_ENCOD_A_MOTOR_RIGHT, HIGH);                // turn on pullup resistor
  //digitalWrite(PIN_ENCOD_B_MOTOR_RIGHT, HIGH);
  attachInterrupt(1, encoderRightMotor, RISING);
  
}

//function that will be called when receiving command from host
void handle_cmd (const geometry_msgs::Twist& cmd_vel) {
  noCommLoops = 0;                                                  //Reset the counter for number of main loops without communication
  
  speed_req = cmd_vel.linear.x;                                     //Extract the commanded linear speed from the message

  angular_speed_req = cmd_vel.angular.z;                            //Extract the commanded angular speed from the message
  
  speed_req_left = speed_req - angular_speed_req*(wheelbase/2);     //Calculate the required speed for the left motor to comply with commanded linear and angular speeds
  speed_req_right = speed_req + angular_speed_req*(wheelbase/2);    //Calculate the required speed for the right motor to comply with commanded linear and angular speeds
}


                                                                                    // ROS SUBSCRIBER

ros::Subscriber<geometry_msgs::Twist> cmd_vel("cmd_vel", handle_cmd);   //create a subscriber to ROS topic for velocity commands (will execute "handle_cmd" function when receiving data)



                                                                                   // NODE HANDLER SETUP

void initNodeHandler(){

  nh.initNode();
  nh.advertise(chatter);

  nh.subscribe(cmd_vel);    //suscribe to ROS topic for velocity commands
  nh.advertise(speed_pub);  //prepare to publish speed in ROS topic
  
}
                      

                                                                                   // SETUP ALL

void setup() {

  connectWiFi();
  initNodeHandler();
  setupMotor();

  setupPID();
  setEncoder();
}
                                                                                   //LOOP MAIN
void loop() {

  nh.spinOnce();

  if((millis()-lastMilli) >= LOOPTIME)   
  {                                                                           // enter timed loop
    lastMilli = millis();
  
    
    
    if (abs(pos_left) < 5){                                                   //Avoid taking in account small disturbances
      speed_act_left = 0;
    }
    else {
      speed_act_left=((pos_left/encoder_cpr)*2*PI)*(1000/LOOPTIME)*radius;           // calculate speed of left wheel
    }
    
    if (abs(pos_right) < 5){                                                  //Avoid taking in account small disturbances
      speed_act_right = 0;
    }
    else {
    speed_act_right=((pos_right/encoder_cpr)*2*PI)*(1000/LOOPTIME)*radius;          // calculate speed of right wheel
    }
    
    pos_left = 0;
    pos_right = 0;

    speed_cmd_left = constrain(speed_cmd_left, -max_speed, max_speed);
    PID_leftMotor.Compute();                                                 
    // compute PWM value for left motor. Check constant definition comments for more information.
    PWM_leftMotor = constrain(((speed_req_left+sgn(speed_req_left)*min_speed_cmd)/speed_to_pwm_ratio) + (speed_cmd_left/speed_to_pwm_ratio), -255, 255); //
    
    if (noCommLoops >= noCommLoopMax) {                   //Stopping if too much time without command
      leftMotor.setSpeed(0);
      leftMotor.stop();
    }
    else if (speed_req_left == 0){                        //Stopping
      leftMotor.setSpeed(0);
      leftMotor.stop();
    }
    else if (PWM_leftMotor > 0){                          //Going forward
      leftMotor.setSpeed(abs(PWM_leftMotor));
      leftMotor.backward();
    }
    else {                                               //Going backward
      leftMotor.setSpeed(abs(PWM_leftMotor));
      leftMotor.forward();
    }
    
    speed_cmd_right = constrain(speed_cmd_right, -max_speed, max_speed);    
    PID_rightMotor.Compute();                                                 
    // compute PWM value for right motor. Check constant definition comments for more information.
    PWM_rightMotor = constrain(((speed_req_right+sgn(speed_req_right)*min_speed_cmd)/speed_to_pwm_ratio) + (speed_cmd_right/speed_to_pwm_ratio), -255, 255); // 

    if (noCommLoops >= noCommLoopMax) {                   //Stopping if too much time without command
      rightMotor.setSpeed(0);
      rightMotor.stop();
    }
    else if (speed_req_right == 0){                       //Stopping
      rightMotor.setSpeed(0);
      rightMotor.stop();
    }
    else if (PWM_rightMotor > 0){                         //Going forward
      rightMotor.setSpeed(abs(PWM_rightMotor));
      rightMotor.forward();
    }
    else {                                                //Going backward
      rightMotor.setSpeed(abs(PWM_rightMotor));
      rightMotor->backward();
    }

    if((millis()-lastMilli) >= LOOPTIME){         //write an error if execution time of the loop in longer than the specified looptime
      Serial.println(" TOO LONG ");
    }

    noCommLoops++;
    if (noCommLoops == 65535){
      noCommLoops = noCommLoopMax;
    }
    
    publishSpeed(LOOPTIME);   //Publish odometry on ROS topic
  }
 }
 
                                                                    // FUNCTIONS 
                                                
//Publish function for odometry, uses a vector type message to send the data (message type is not meant for that but that's easier than creating a specific message type)
void publishSpeed(double time) {
  speed_msg.header.stamp = nh.now();      //timestamp for odometry data
  speed_msg.vector.x = speed_act_left;    //left wheel speed (in m/s)
  speed_msg.vector.y = speed_act_right;   //right wheel speed (in m/s)
  speed_msg.vector.z = time/1000;         //looptime, should be the same as specified in LOOPTIME (in s)
  speed_pub.publish(&speed_msg);
  nh.spinOnce();
  nh.loginfo("Publishing odometry");
}
                                                
//Left motor encoder counter
void encoderLeftMotor() {
  if (digitalRead(PIN_ENCOD_A_MOTOR_LEFT) == digitalRead(PIN_ENCOD_B_MOTOR_LEFT)) pos_left++;
  else pos_left--;
}

//Right motor encoder counter
void encoderRightMotor() {
  if (digitalRead(PIN_ENCOD_A_MOTOR_RIGHT) == digitalRead(PIN_ENCOD_B_MOTOR_RIGHT)) pos_right--;
  else pos_right++;
}

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
