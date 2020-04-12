
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
// Motor A
int motor1Pin1 = 27;
int motor1Pin2 = 26;
int enable1Pin = 13;

// Motor B
int motor2Pin3 = 33;
int motor2Pin4 = 32;
int enable2Pin = 12;

// Setting PWM properties
const int freq = 30000;
const int pwmChannel = 0;
const int resolution = 8;
int dutyCycle = 200;

bool mfwd; // flag to move fwd
bool mbwd;  // flag to move bwd 
bool tleft;  // flag to turn left
bool tright; // flag to turn right
int len=0; //length of motor running

L298N leftMotor();
L298N rightMotor();


                                                                            // PID CONTROLLER PARAMETERS 

//PID constants

const double PID_left_param[] = { 2, 5, 1 }; //Respectively Kp, Ki and Kd for left motor PID
const double PID_right_param[] = { 2, 5, 1 }; //Respectively Kp, Ki and Kd for right motor PID

volatile float pos_left = 0;       //Left motor encoder position
volatile float pos_right = 0;      //Right motor encoder position

PID PID_leftMotor(&speed_act_left, &speed_cmd_left, &speed_req_left, PID_left_param[0], PID_left_param[1], PID_left_param[2], DIRECT);          //Setting up the PID for left motor
PID PID_rightMotor(&speed_act_right, &speed_cmd_right, &speed_req_right, PID_right_param[0], PID_right_param[1], PID_right_param[2], DIRECT);   //Setting up the PID for right motor


int encoderLeftPin = 34;  //Left motor encoder pin 34
int encoderRightPin = 35; //Right motor encoder pin 35



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
  // sets the pins as outputs:
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(enable1Pin, OUTPUT);
  pinMode(motor2Pin3, OUTPUT);
  pinMode(motor2Pin4, OUTPUT);
  pinMode(enable2Pin, OUTPUT);

  // configure LED PWM functionalities
  ledcSetup(pwmChannel, freq, resolution);

  // attach the channel to the GPIO to be controlled
  ledcAttachPin(enable1Pin, pwmChannel);
  ledcAttachPin(enable2Pin, pwmChannel);

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

void stopMove(){
  // Stop the DC motor
  Serial.println("Motor stopped");
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin3, LOW);
  digitalWrite(motor2Pin4, LOW);
  
}

void moveFwd(int ln){
  // Move the DC motor forward at maximum speed
  Serial.println("Moving Forward");
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, HIGH);
  digitalWrite(motor2Pin3, LOW);
  digitalWrite(motor2Pin4, HIGH);

  //Serial.print(len);
  //Serial.print(mfwd);  

  len=ln-1; // use local variable to decrement global var value
  
    if (len<=1)
    {
      mfwd=false;
      //stop();
    }
    else
      mfwd=true;
}

void moveBwd(int ln){
  // Move DC motor backwards at maximum speed
  Serial.println("Moving Backwards");
  digitalWrite(motor1Pin1, HIGH);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin3, HIGH);
  digitalWrite(motor2Pin4, LOW);

  len=ln-1; // use local variable to decrement global var value
  
    if (len<=1)
    {
      mbwd=false;
      //stop();
    }
    else
      mbwd=true;
}

void turnRight(int ln){
  // Turn the DC motor right at maximum speed
  Serial.println("Turn Right");
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, HIGH);
  digitalWrite(motor2Pin3, LOW);
  digitalWrite(motor2Pin4, LOW);

  len=ln-1; // use local variable to decrement global var value
  
    if (len<=1)
    {
      tright=false;
      //stop();
    }
    else
      tright=true;
 }

void turnLeft(int ln){
  // Turn the DC motor left at maximum speed
  Serial.println("Turn Left");
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin3, LOW);
  digitalWrite(motor2Pin4, HIGH);

  len=ln-1; // use local variable to decrement global var value
  
    if (len<=1)
    {
      tleft=false;
      //stop();
    }
    else
      tleft=true;  
 }

void moveFwdInc(){
  // Move DC motor forward with increasing speed
  digitalWrite(motor1Pin1, HIGH);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin3, LOW);
  digitalWrite(motor2Pin4, HIGH);

  while (dutyCycle <= 255){
    ledcWrite(pwmChannel, dutyCycle);
    Serial.print("Forward with duty cycle: ");
    Serial.println(dutyCycle);
    dutyCycle = dutyCycle + 5;
    delay(500);
    }
  dutyCycle = 200;

 }
                                                                           
                                                                            
                                                                            // ROS CALLBACK FUNCTIONS

void leftCallback(const std_msgs::Int16& msg) { //  All subscriber messages callbacks here
    len = abs(msg.data);  
    turnLeft(len);
}

void rightCallback(const std_msgs::Int16& msg) {
    len = abs(msg.data);
    turnRight(len);
}

void forwardCallback(const std_msgs::Int16& msg) {
    len = abs(msg.data);  
    moveFwd(len);        //set length off motor running from data in /car/forward
}

void backwardCallback(const std_msgs::Int16& msg) {
    len = abs(msg.data);
    moveBwd(len);
}

void stopCallback(const std_msgs::Int16& msg) {
   stopMove();
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

ros::Subscriber<std_msgs::Int16> sub_f("/car/forward", &forwardCallback);
ros::Subscriber<std_msgs::Int16> sub_b("/car/backward", &backwardCallback);
ros::Subscriber<std_msgs::Int16> sub_l("/car/left", &leftCallback);
ros::Subscriber<std_msgs::Int16> sub_r("/car/right", &rightCallback);
ros::Subscriber<std_msgs::Int16> sub_s("/car/stop", &stopCallback);

ros::Subscriber<geometry_msgs::Twist> cmd_vel("cmd_vel", handle_cmd);   //create a subscriber to ROS topic for velocity commands (will execute "handle_cmd" function when receiving data)



                                                                                   // NODE HANDLER SETUP

void initNodeHandler(){

  nh.initNode();
  nh.advertise(chatter);
  //nh.getHardware()->setConnection(server,serverPort);
  nh.subscribe(sub_f);
  nh.subscribe(sub_b);
  nh.subscribe(sub_l);
  nh.subscribe(sub_r);
  nh.subscribe(sub_s);  
}

                      

                                                                                   // SETUP ALL

void setup() {

  connectWiFi();
  initNodeHandler();
  setupMotor();

  //initialize the variables we're linked to
  leftInput = analogRead(encoderLeftPin);
  rightInput = analogRead(encoderRightPin);
  Setpoint = 100;

  //turn the PID on
  leftPID.SetMode(AUTOMATIC);
  rightPID.SetMode(AUTOMATIC);

}
                                                                                   //LOOP MAIN
void loop() {

  nh.spinOnce();

  if (mfwd==true) {
    ledcWrite(pwmChannel, dutyCycle); // without this in loop the motor wont move
    moveFwd(len); // give decremented global var value into local var argument
  }
  else if (mbwd==true) {
    ledcWrite(pwmChannel, dutyCycle); // without this in loop the motor wont move
    moveBwd(len); // give decremented global var value into local var argument
  }
  else if (tleft==true) {
    ledcWrite(pwmChannel, dutyCycle); // without this in loop the motor wont move
    turnLeft(len); // give decremented global var value into local var argument
  } 
  else if (tright==true) {
    ledcWrite(pwmChannel, dutyCycle); // without this in loop the motor wont move
    turnRight(len); // give decremented global var value into local var argument
  } 
  else
    stopMove();


/*
  leftInput = analogRead(encoderLeftPin);
  rightInput = analogRead(encoderRightPin);
  leftPID.Compute();
  rightPID.Compute();
  
  //analogWrite(PIN_OUTPUT, Output);
*/
 
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
}
