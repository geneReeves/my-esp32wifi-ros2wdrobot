#include <ros.h>
#include <std_msgs/String.h>
#include <WiFi.h>
#include <std_msgs/Int16.h>

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

                                                                            // ROS DECLARATION
ros::NodeHandle_<WiFiHardware> nh;

std_msgs::String str_msg;
std_msgs::Int16 int_msg;


                                                                            //ROS PUBLISHER
ros::Publisher chatter("chatter", &str_msg);

char hello[13] = "hello world!";

                                                                            // MOTOR CONFIGURATION
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

                                                                            //FUNCTION DEFINITION
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


                                                                                    //ROS SUBSCRIBER

ros::Subscriber<std_msgs::Int16> sub_f("/car/forward", &forwardCallback);
ros::Subscriber<std_msgs::Int16> sub_b("/car/backward", &backwardCallback);
ros::Subscriber<std_msgs::Int16> sub_l("/car/left", &leftCallback);
ros::Subscriber<std_msgs::Int16> sub_r("/car/right", &rightCallback);
ros::Subscriber<std_msgs::Int16> sub_s("/car/stop", &stopCallback);


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

                                                                                   //SETUP

void setup() {

  connectWiFi();
  initNodeHandler();
  setupMotor();

}
                                                                                   //LOOP
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
}
