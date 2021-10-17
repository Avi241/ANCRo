
#define TCP 1 //  0 if you are using UART for ROSserial or  1 if You are using ROSserial over Tcp(Wifi)

#if TCP
#include "ArduinoTcpHardware.h"
#else
#include "ArduinoHardware.h"
#endif

#include "fun.h"
#include "ros.h"
#include "geometry_msgs/Twist.h"
#include <ros/time.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>

#if TCP

#include <WiFi.h>
const char *ssid = "Arvind";
const char *password = "12345678";
// Set the rosserial socket server IP address
IPAddress server(192, 168, 43, 233); // ros-master ip address
// Set the rosserial socket server port
const uint16_t serverPort = 11411;

#endif

// Object initialization
ros::NodeHandle nh; 
geometry_msgs::TransformStamped t;
nav_msgs::Odometry odom_msg;
ros::Publisher odom_pub("odom", &odom_msg);
tf::TransformBroadcaster broadcaster;

unsigned long currentMillis;
long previousMillis = 0; // set up timers
float loopTime = 10;

long previous_EncoderR = 0, previous_EncoderL = 0, current_EncoderR = 0, current_EncoderL = 0;

// tf variables to be broadcast
double x = 0;
double y = 0;
double theta = 0;

char base_link[] = "/base_link";
char odom[] = "/odom";

// position and velocity variables read from the ODrive
long posR;
long posL;

// variables to work out the different on each cycle

long posR_old;
long posL_old;
long posR_diff;
long posL_diff;
float posR_mm_diff;
float posL_mm_diff;
float pos_average_mm_diff;
float pos_total_mm;

float kpR = 2, kdR = 0.1, kiR = 0.3, phi = 0;
float kpL = 2, kdL = 0.1, kiL = 0.3;
int pwmR, pwmL;
double set_velocityR = 0, current_velocityR = 0, error_velocityR = 0, error_velocity_sumR = 0, error_velocity_previousR = 0;
double set_velocityL = 0, current_velocityL = 0, error_velocityL = 0, error_velocity_sumL = 0, error_velocity_previousL = 0;

hw_timer_t *timer = NULL; // timer for Esp32

void IRAM_ATTR onTimer() // Timer ISR of 10ms
{
    //  portENTER_CRITICAL_ISR(&timerMux);
    //  portEXIT_CRITICAL_ISR(&timerMux);

    current_EncoderR = myEncR.read();
    current_velocityR = 6000.0 * ((current_EncoderR - previous_EncoderR) / 1462.0);
    previous_EncoderR = current_EncoderR;

    current_EncoderL = myEncL.read();
    current_velocityL = 6000.0 * ((current_EncoderL - previous_EncoderL) / 1462.0);
    previous_EncoderL = current_EncoderL;

    error_velocityR = set_velocityR - current_velocityR;
    error_velocityL = set_velocityL - current_velocityL;

    pwmR = error_velocityR * kpR + error_velocity_sumR * (kiR) + (error_velocityR - error_velocity_previousR) * kdR;
    pwmL = error_velocityL * kpL + error_velocity_sumL * (kiL) + (error_velocityL - error_velocity_previousL) * kdL;

    error_velocity_previousR = error_velocityR;
    error_velocity_previousL = error_velocityL;

    error_velocity_sumR = error_velocity_sumR + error_velocityR;
    error_velocity_sumL = error_velocity_sumL + error_velocityL;

    if (error_velocity_sumR > 300)
        error_velocity_sumR = 300;
    if (error_velocity_sumR < -300)
        error_velocity_sumR = -300;

    if (error_velocity_sumL > 300)
        error_velocity_sumL = 300;
    if (error_velocity_sumL < -300)
        error_velocity_sumL = -300;

    // Logic for Right Motor

    if (pwmR <= 255 && pwmR > 0)
        mr_clk(pwmR);

    else if (pwmR > 255)
        mr_clk(255);

    else if (pwmR < 0 && pwmR >= (-255))
    {
        mr_aclk(abs(pwmR));
    }

    else if (pwmR < -255)
        mr_aclk(255);
    else
        mr_clk(0);

    // Logic for Left Motor
    if (pwmL <= 255 && pwmL > 0)
        ml_clk(pwmL);

    else if (pwmL > 255)
        ml_clk(255);

    else if (pwmL < 0 && pwmL >= (-255))
        ml_aclk(abs(pwmL));

    else if (pwmL < -255)
        ml_aclk(255);
    else
        mr_clk(0);
}

void velCallback(const geometry_msgs::Twist &vel)
{
    set_velocityR = 187.2 * vel.linear.x + 32.14 * vel.angular.z;
    set_velocityL = -186.2 * vel.linear.x + 32.14 * vel.angular.z;
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", velCallback);

void setup()
{

#if TCP

    Serial.begin(115200);
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);

    // Connect the ESP to the wifi AP
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());

    // Set the connection to rosserial socket server
    nh.getHardware()->setConnection(server, serverPort);

    // Another way to get IP
    Serial.print("IP = ");
    Serial.println(nh.getHardware()->getLocalIP());

#endif

#if !TCP
    nh.getHardware()->setBaud(115200);
#endif

    nh.initNode(); // init ROS
    nh.subscribe(sub);
    nh.advertise(odom_pub); //setup publisher
    broadcaster.init(nh); // set up broadcaster

    pinSetup(); // set the pinmodes

    //Timers Setup of ESP32 for 10ms Interrupt timer
    timer = timerBegin(0, 80, true);
    timerAttachInterrupt(timer, &onTimer, true);
    timerAlarmWrite(timer, 10000, true);
    timerAlarmEnable(timer);
}

void loop()
{

    nh.spinOnce();

    currentMillis = millis();
    if (currentMillis - previousMillis >= loopTime)
    { // run a loop every 10ms
        previousMillis = currentMillis;
        /// ROS Logics
        posR = current_EncoderR;
        posL = current_EncoderL * -1;

        // work out the difference on each loop, and bookmark the old value
        posR_diff = posR - posR_old;
        posL_diff = posL - posL_old;
        posR_old = posR;
        posL_old = posL;

        // calc mm from encoder counts
        posR_mm_diff = posR_diff / 4.561;
        posL_mm_diff = posL_diff / 4.561;

        // calc distance travelled based on average of both wheels
        pos_average_mm_diff = (posR_mm_diff + posL_mm_diff) / 2; // difference in each cycle
        pos_total_mm += pos_average_mm_diff;                     // calc total running total distance

        // calc angle or rotation to broadcast with tf
        phi = ((posR_mm_diff - posL_mm_diff) / 318);

        theta += phi;

        if (theta >= TWO_PI)
        {
            theta -= TWO_PI;
        }
        if (theta <= (-TWO_PI))
        {
            theta += TWO_PI;
        }

        // calc x and y to broadcast with tf

        y += pos_average_mm_diff * sin(theta + phi/2);
        x += pos_average_mm_diff * cos(theta + phi/2);

        // *** broadcast odom->base_link transform with tf ***

        geometry_msgs::TransformStamped t;

        t.header.frame_id = odom;
        t.child_frame_id = base_link;

        t.transform.translation.x = x / 1000; // convert to metres
        t.transform.translation.y = y / 1000;
        t.transform.translation.z = 0;

        t.transform.rotation = tf::createQuaternionFromYaw(theta);
        t.header.stamp = nh.now();

        broadcaster.sendTransform(t);

        // *** broadcast odom message ***

        nav_msgs::Odometry odom_msg;
        odom_msg.header.stamp = nh.now();
        odom_msg.header.frame_id = odom;
        odom_msg.pose.pose.position.x = x / 1000;
        odom_msg.pose.pose.position.y = y / 1000;
        odom_msg.pose.pose.position.z = 0.0;
        odom_msg.pose.pose.orientation = tf::createQuaternionFromYaw(theta);

        odom_msg.child_frame_id = base_link;
        odom_msg.twist.twist.linear.x = ((posR_mm_diff + posL_mm_diff) / 2) / 10;     // forward linear velovity
        odom_msg.twist.twist.linear.y = 0.0;                                          // robot does not move sideways
        odom_msg.twist.twist.angular.z = ((posR_mm_diff - posL_mm_diff) / 318) * 100; // anglular velocity

        odom_pub.publish(&odom_msg);
    }
}
