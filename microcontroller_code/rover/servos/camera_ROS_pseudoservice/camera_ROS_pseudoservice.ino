/* Written by Mastafa Awal for the York University Rover Team
    Dynamixel library written by Akira --> https://github.com/akira215/DynamixelArduino
    SoftHalfDuplex library written by  --> https://github.com/akira215/HalfDuplexSerial-for-Arduino
    This code is open source and released under the GNU Lesser General Public License 2.1.
    This code is using a pseudo-service instead of an actual one because of reliability issues.
*/

//Import required files
#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Empty.h>
#include <SoftHalfDuplexSerial.h>
#include <DynamixelAx.h>
#include <messages/servo_position.h>

#define error_fatal "F" //Short and sweet error message to save those precious memory bytes since ROS can't use the flash command
#define error_med "M"
#define error_low "L"

#define SERVO_ID 1
#define SERVO_DATA_PIN 8
#define MAX_SERVO_ANGLE 150
#define MIN_SERVO_ANGLE -150
#define MOVING_SPEED 1023
#define MAX_TORQUE 512

messages::servo_position servo_state;
std_msgs::Empty empty_msg;

softHalfDuplexSerial port(SERVO_DATA_PIN);
dxlAx servo(&port);

bool mode_absolute = false;
unsigned short error = DXL_ERR_SUCCESS;

//Lower our memory footprint
ros::NodeHandle_<ArduinoHardware, 6, 6, 150, 150>  nh;

//Setup camera position publisher

ros::Publisher position_pub("camera/position", &servo_state);

/* Any data that is [-150,150] is considered position data and makes the servo go to that position. 
For continous rotation, 1000 is the new zero. A value [-1100, 1100] determines how fast it will move and in what direction (positive --> CW)
Value of 300 used because these servos can only move to positions in 300 degrees in goal mode. Declared as a constant in case we upgrade.
*/

void pan_cam(const std_msgs::Int16 &pan) {
  reset_error_state();
  if (pan.data >= MIN_SERVO_ANGLE && pan.data <= MAX_SERVO_ANGLE) {
    setModeAbsolute();
    servo.setGoalPosition(SERVO_ID, (MAX_SERVO_ANGLE - pan.data) * 3.41);
  } else if (pan.data >= 900 && pan.data <= 1000) {
    setModeCont();
    servo.setMovingSpeed(SERVO_ID, (-(pan.data - 1000) * 10.23));
  } else if (pan.data > 1000 && pan.data <= 1100 ) {
    setModeCont();
    servo.setMovingSpeed(SERVO_ID, (((pan.data - 1000) * 10.23) + 1024));
  } else {
    nh.loginfo(error_low);
  }

  waitForServoReady();
  //servo.isMoving(SERVO_ID);
  waitForServoResponse();
  if (servo.readDxlError() != DXL_ERR_SUCCESS) {
    nh.logfatal(error_fatal);
    position_request(empty_msg); // call the publisher method to transmit error message
  }

}


void position_request(const std_msgs::Empty &e) {
  reset_error_state();
  waitForServoReady();
  servo.readPresentPosition(SERVO_ID);
  waitForServoResponse();
  error = servo.readDxlError();

  if (error != DXL_ERR_SUCCESS) {
    servo_state.error = true;
    servo_state.position = 2000 + error; //See google drive "Software Information" doc for error code details
    nh.logwarn(error_med);
  } else {
    servo_state.position = - ((servo.readDxlResult()/3.41) - MAX_SERVO_ANGLE); \

    //Determine if the servo is currently moving or not
    waitForServoReady();
    servo.isMoving(SERVO_ID);
    waitForServoResponse();

    error = servo.readDxlError();
    
    if (error != DXL_ERR_SUCCESS) {
      servo_state.error = true;
      servo_state.position = 2000 + error;
      nh.logwarn(error_med);
      
    } else {
      
      if (!servo.readDxlResult() && mode_absolute) {
        servo_state.error = false;
      } else {
        servo_state.error = true;
        if (!mode_absolute) {
          servo_state.position = 2100; //Error code for servo moving due to continous command sent
        } else {
          servo_state.position = 2200; //Error code for servo not finishing moving into requested position
        }
      }
    }
  }

  position_pub.publish(& servo_state);

}

ros::Subscriber<std_msgs::Int16> sub_cam_pan("camera/pan", &pan_cam);
ros::Subscriber<std_msgs::Empty> sub_pos_req("camera/position_request", &position_request);

void setup()
{

  servo.begin(19200); //begin communication with the servo
  servo.reboot(SERVO_ID); //reboot the servo
  delay(500);
  servo.setLedEnable(SERVO_ID, 1);  //Blink the LED for debugging purposes
  //Initialize the servo value;
  setModeAbsolute();
  servo.setMovingSpeed(SERVO_ID, MOVING_SPEED);
  servo.setGoalPosition(SERVO_ID, 512); //Center the servo
  servo.setTorqueEnable(SERVO_ID, 1); //Enable torque so that servo is forced to stay in position
  servo.setMaxTorque(SERVO_ID, MAX_TORQUE);
  servo.setStatusReturnLevel(SERVO_ID, 1); //Only return when read commands are sent
  //Initiate the node & subscribers
  nh.initNode();
  nh.subscribe(sub_cam_pan);
  nh.subscribe(sub_pos_req);
  nh.advertise(position_pub);
  servo.setLedEnable(SERVO_ID, 0); //Turn off the LED
}


void loop()
{
  nh.spinOnce();
}


void setModeAbsolute() {
  if (!mode_absolute) {
    servo.setCWAngleLimit(SERVO_ID, 1);
    servo.setCCWAngleLimit(SERVO_ID, 1023);
    mode_absolute = true;
  }
}

void setModeCont() {
  if (mode_absolute) {
    servo.setCWAngleLimit(SERVO_ID, 0);
    servo.setCCWAngleLimit(SERVO_ID, 0);
    mode_absolute = false;
  }
}

void waitForServoResponse() {
  while (!servo.dxlDataReady()) {
    //Wait until the servo answers
  }
}

void waitForServoReady() {
  while (servo.isBusy()) {
    //Delay till servo is ready
  }
}

void reset_error_state() {
   error = DXL_ERR_SUCCESS;
}
