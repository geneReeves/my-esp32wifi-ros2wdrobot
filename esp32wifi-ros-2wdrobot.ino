#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <WiFi.h>

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
  nh.initNode();
  nh.advertise(chatter);
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

void stop(){
  // Stop the DC motor
  Serial.println("Motor stopped");
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin3, LOW);
  digitalWrite(motor2Pin4, LOW);
  }

void moveFwd(len){
  // Move the DC motor forward at maximum speed
  Serial.println("Moving Forward");
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, HIGH);
  digitalWrite(motor2Pin3, LOW);
  digitalWrite(motor2Pin4, HIGH);
  
  delay(len);
  }

void moveBwd(){
  // Move DC motor backwards at maximum speed
  Serial.println("Moving Backwards");
  digitalWrite(motor1Pin1, HIGH);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin3, HIGH);
  digitalWrite(motor2Pin4, LOW);
  
  delay(len);
}

void turnRight(){
  // Turn the DC motor right at maximum speed
  Serial.println("Turn Right");
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, HIGH);
  digitalWrite(motor2Pin3, LOW);
  digitalWrite(motor2Pin4, LOW);
  
  delay(len);
  }

void turnLeft(){
  // Turn the DC motor left at maximum speed
  Serial.println("Turn Left");
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin3, LOW);
  digitalWrite(motor2Pin4, HIGH);
  
  delay(len);
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
    delay(len);
    }
  dutyCycle = 200;

  }


                                                                            // ROS CONFIGURATION
                                                                            
                                                                            
// Ros CallBack Functions
void leftCallback(const std_msgs::Int16& msg) { //  All subscriber messages callbacks here
    len = abs(msg.data);	
	turnLeft(len);
}

void rightCallback(const std_msgs::Int16& msg) {
    len = abs(msg.data);
	turnRight();
}

void forwardCallback(const std_msgs::Int16& msg) {
    len = abs(msg.data);	
	moveFwd();
}

void backwardCallback(const std_msgs::Int16& msg) {
    len = abs(msg.data);
    moveBwd();
}

void stopCallback(const std_msgs::Int16& msg) {
	stop();
}


// Ros Objects
ros::NodeHandle_<WiFiHardware> nh;

std_msgs::Int16 int_msg;
std_msgs::String str_msg;

ros::Publisher chatter("chatter", &str_msg);
char hello[13] = "hello world!";

ros::Subscriber<std_msgs::Int16> sub_f("/car/forward", &forwardCallback);
ros::Subscriber<std_msgs::Int16> sub_b("/car/backward", &backwardCallback);
ros::Subscriber<std_msgs::Int16> sub_l("/car/left", &leftCallback);
ros::Subscriber<std_msgs::Int16> sub_r("/car/right", &rightCallback);
ros::Subscriber<std_msgs::Int16> sub_s("/car/stop", &stopCallback);




                                                                            // MOTOR CONFIGURATION
// Motor A
int motor1Pin1 = 27;
int motor1Pin2 = 26;
int enable1Pin = 14;

// Motor B
int motor2Pin3 = 33;
int motor2Pin4 = 32;
int enable2Pin = 12;

// Setting PWM properties
const int freq = 30000;
const int pwmChannel = 0;
const int resolution = 8;
int dutyCycle = 200;



                                                                                   //SETUP

void setup() {

  setupMotor();
  connectWiFi();

}
                                                                                   //LOOP
void loop() {

nh.spinOnce();
delay(500);

}