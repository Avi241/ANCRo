#define UP D3
#define DOWN D1
#define LEFT D2
#define RIGHT D4
#define A D5
#define B D7
#define C D0
#define D D6

#include <ESP8266WiFi.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>

const char* ssid     = "Arvind";
const char* password = "12345678";
// Set the rosserial socket server IP address
IPAddress server(192,168,43,233);
// Set the rosserial socket server port
const uint16_t serverPort = 11411;

float x=0.5,z=1.0;


ros::NodeHandle nh;
geometry_msgs::Twist twist;
ros::Publisher vel( "/cmd_vel", &twist);

void setup() 
{
  pinMode(UP,INPUT_PULLUP);
  pinMode(DOWN,INPUT_PULLUP);
  pinMode(LEFT,INPUT_PULLUP);
  pinMode(RIGHT,INPUT_PULLUP);
  pinMode(A,INPUT_PULLUP);
  pinMode(B,INPUT_PULLUP);
  pinMode(C,INPUT_PULLUP);
  pinMode(D,INPUT_PULLUP);
  // Use ESP8266 serial to monitor the process
  Serial.begin(115200);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  // Connect the ESP8266 the the wifi AP
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  // Set the connection to rosserial socket server
  nh.getHardware()->setConnection(server, serverPort);
  nh.initNode();

  // Another way to get IP
  Serial.print("IP = ");
  Serial.println(nh.getHardware()->getLocalIP());
  nh.advertise(vel);
  // put your setup code here, to run once:

}

void loop() 
{
  
  if(!digitalRead(UP))
  twist.linear.x = x;
  else if(!digitalRead(DOWN))
  twist.linear.x = -x;
  else if(!digitalRead(B))
  twist.angular.z = z;
  else if(!digitalRead(D))
  twist.angular.z = -z;

  else if(!digitalRead(LEFT))
  {x=x-0.01; z=z-0.01;}
  else if(!digitalRead(RIGHT))
  {x=x-0.01; z=z+0.01;}
  else if(!digitalRead(A))
  {
  twist.linear.x = x; 
  twist.angular.z = z;
  }
  else if(!digitalRead(C))
  {
  twist.linear.x = x; 
  twist.angular.z = -z;
  }
//  else if(!digitalRead(UP) && !digitalRead(A))
//  {
//  twist.linear.x = x; 
//  twist.angular.z = z;
//  }
//  else if(!digitalRead(DOWN) && !digitalRead(A))
//  {
//  twist.linear.x = x; 
//  twist.angular.z = z;
//  }
//  else if(!digitalRead(UP) && !digitalRead(C))
//  {
//  twist.linear.x = x; 
//  twist.angular.z = -z;
//  }
//  else if(!digitalRead(DOWN) && !digitalRead(C))
//  {
//  twist.linear.x = x; 
//  twist.angular.z = -z;
//  }
  
  else
  {
    twist.linear.x = 0;
    twist.angular.z = 0;
  }
  vel.publish(&twist);
  nh.spinOnce();
delay(10);
}
