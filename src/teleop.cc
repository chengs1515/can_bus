#include <stdio.h>
#include <termios.h>
#include <iostream>
#include <memory>
#include <thread>
#include <ros/ros.h>
#include <autoware_msgs/ControlCommandStamped.h>

#include "../include/can_bus/CANBus.hpp"

using  std::cout;
using  std::endl;
#define throttle_inc_delta 0.1
#define brake_inc_delta 0.1
#define  steer_inc_delta 0.0524

const uint32_t KEYCODE_O = 0x4F;  // '0'

const uint32_t KEYCODE_UP1 = 0x57;  // 'w'
const uint32_t KEYCODE_UP2 = 0x77;  // 'w'
const uint32_t KEYCODE_DN1 = 0x53;  // 'S'
const uint32_t KEYCODE_DN2 = 0x73;  // 's'
const uint32_t KEYCODE_LF1 = 0x41;  // 'A'
const uint32_t KEYCODE_LF2 = 0x61;  // 'a'
const uint32_t KEYCODE_RT1 = 0x44;  // 'D'
const uint32_t KEYCODE_RT2 = 0x64;  // 'd'

const uint32_t KEYCODE_PKBK = 0x50;  // hand brake or parking brake

// set throttle, gear, and brake
const uint32_t KEYCODE_SETT1 = 0x54;  // 'T'
const uint32_t KEYCODE_SETT2 = 0x74;  // 't'
const uint32_t KEYCODE_SETG1 = 0x47;  // 'G'
const uint32_t KEYCODE_SETG2 = 0x67;  // 'g'
const uint32_t KEYCODE_SETB1 = 0x42;  // 'B'
const uint32_t KEYCODE_SETB2 = 0x62;  // 'b'
const uint32_t KEYCODE_ZERO = 0x30;   // '0'

const uint32_t KEYCODE_SETQ1 = 0x51;  // 'Q'
const uint32_t KEYCODE_SETQ2 = 0x71;  // 'q'

// change action
const uint32_t KEYCODE_MODE = 0x6D;  // 'm'

// emergency stop
const uint32_t KEYCODE_ESTOP = 0x45;  // 'E'

// help
const uint32_t KEYCODE_HELP = 0x68;   // 'h'
const uint32_t KEYCODE_HELP2 = 0x48;  // 'H'

class Teleop {
 public:
  Teleop(ros::NodeHandle& nh):nh_(nh){
      ctr_pub_ = nh_.advertise<autoware_msgs::ControlCommand>("/ctrl_cmd",1) ;
      
      PrintKeycode();
      keyboard_thread_.reset(new std::thread([this] { KeyboardLoopThreadFunc(); }));
  };
  
    void KeyboardLoopThreadFunc() {
    char c = 0;
    int32_t level = 0;
    double brake = 0;
    double throttle = 0;
    double steering = 0;
    struct termios cooked_;
    struct termios raw_;
    int32_t kfd_ = 0;
    bool parking_brake = false;

    // get the console in raw mode
    tcgetattr(kfd_, &cooked_);
    std::memcpy(&raw_, &cooked_, sizeof(struct termios));
    raw_.c_lflag &= ~(ICANON | ECHO);
    // Setting a new line, then end of file
    raw_.c_cc[VEOL] = 1;
    raw_.c_cc[VEOF] = 2;
    tcsetattr(kfd_, TCSANOW, &raw_);
    puts("Teleop:\nReading from keyboard now.");
    puts("---------------------------");
    puts("Use arrow keys to drive the car.");

    while (ros::ok()) {
      // get the next event from the keyboard
      if (read(kfd_, &c, 1) < 0) {
        perror("read():");
        exit(-1);
      }

      switch (c) {
        case KEYCODE_UP1:  // accelerate
        case KEYCODE_UP2:
          cmd_.linear_acceleration += throttle_inc_delta;
          cmd_.linear_acceleration = cmd_.linear_acceleration>1?1:cmd_.linear_acceleration;
          cout<<"target acceleration command = "<<cmd_.linear_acceleration<<endl;
          break;
        
        case KEYCODE_DN1:  // decelerate
        case KEYCODE_DN2:
           cmd_.linear_acceleration -= brake_inc_delta;
          cmd_.linear_acceleration = cmd_.linear_acceleration>-1?cmd_.linear_acceleration:-1;
            cout<<"target acceleration command ="<<cmd_.linear_acceleration<<endl;
          break;

        case KEYCODE_LF1:  // left
        case KEYCODE_LF2:
          cmd_.steering_angle -= steer_inc_delta;
          cmd_.steering_angle = cmd_.steering_angle>-0.5?cmd_.steering_angle:-0.5;
          cout << "Steering Target command = " << cmd_.steering_angle<<endl;
          break;
        
        case KEYCODE_RT1:  // right
        case KEYCODE_RT2:
          cmd_.steering_angle += steer_inc_delta;
          cmd_.steering_angle = cmd_.steering_angle>0.5?0.5:cmd_.steering_angle;
          cout << "Steering Target command = " << cmd_.steering_angle<<endl;
          break;
        
        case KEYCODE_PKBK:  // hand brake
          cout << "invalid key pressed"<<endl;
          break;
        
        case KEYCODE_ESTOP:
          cmd_.linear_acceleration = -0.5;
          cout << "Estop Brake : " << cmd_.linear_acceleration<<endl;
          break;

        case KEYCODE_SETT1:  // set throttle
        case KEYCODE_SETT2:
          
          cmd_.linear_velocity +=  1;
         cout<<"Target llinearvelocity is: "<<cmd_.linear_velocity<<endl;
          break;

        case KEYCODE_SETG1:
        case KEYCODE_SETG2:
          cout << "invalid key pressed"<<endl;
          break;

        case KEYCODE_SETB1:
        case KEYCODE_SETB2:
        cmd_.linear_velocity -=  1;
          cout<<"Target llinearvelocity is: "<<cmd_.linear_velocity<<endl;
          break;
        
        case KEYCODE_SETQ1:
        case KEYCODE_SETQ2:
            cout<<"invalid key pressed"<<endl;
          break;
        case KEYCODE_MODE:
          // read keyboard again
          if (read(kfd_, &c, 1) < 0) {
            exit(-1);
          }
          resetkeyboard();

          break;
        
        case KEYCODE_HELP:
        case KEYCODE_HELP2:
          PrintKeycode();
          break;
        default:
          // printf("%X\n", c);
          break;
      }
    }  // keyboard_loop big while
    tcsetattr(kfd_, TCSANOW, &cooked_);
    cout << "keyboard_loop thread quited.";
    return;
  }  // end of keyboard loop thread

    void timerCallback(const ros::TimerEvent& event)
    {
        ctr_pub_.publish(cmd_);
    }
private:
    ros::NodeHandle nh_;
    ros::Publisher ctr_pub_;
    ros::Timer timer_;
    autoware_msgs::ControlCommand cmd_;

    std::unique_ptr<std::thread> keyboard_thread_;

    static void PrintKeycode() {
    system("clear");
    printf("=====================    KEYBOARD MAP   ===================\n");
    printf("HELP:               [%c]     |\n", KEYCODE_HELP);
    printf("Set Action      :   [%c]+Num\n", KEYCODE_MODE);
    printf("                     0 RESET ACTION\n");
    printf("                     1 START ACTION\n");
    printf("\n-----------------------------------------------------------\n");
    printf("Set Gear:           [%c]+Num\n", KEYCODE_SETG1);
    printf("                     0 GEAR_NEUTRAL\n");
    printf("                     1 GEAR_DRIVE\n");
    printf("                     2 GEAR_REVERSE\n");
    printf("                     3 GEAR_PARKING\n");
    printf("                     4 GEAR_LOW\n");
    printf("                     5 GEAR_INVALID\n");
    printf("                     6 GEAR_NONE\n");
    printf("\n-----------------------------------------------------------\n");
    printf("Throttle/Speed up:  [%c]     |  Set Throttle:       [%c]+Num\n",
           KEYCODE_UP1, KEYCODE_SETT1);
    printf("Brake/Speed down:   [%c]     |  Set Brake:          [%c]+Num\n",
           KEYCODE_DN1, KEYCODE_SETB1);
    printf("Steer LEFT:         [%c]     |  Steer RIGHT:        [%c]\n",
           KEYCODE_LF1, KEYCODE_RT1);
    printf("Parkinig Brake:     [%c]     |  Emergency Stop      [%c]\n",
           KEYCODE_PKBK, KEYCODE_ESTOP);
    printf("\n-----------------------------------------------------------\n");
    printf("Exit: Ctrl + C, then press enter to normal terminal\n");
    printf("===========================================================\n");
  }

    void resetkeyboard()
    {
        cmd_.linear_acceleration = 0.0;
        cmd_.steering_angle = 0.0;
        cmd_.linear_velocity = 0.0;
    }
};

int main(int argc,char** argv)
{
    ros::init(argc,argv,"teleop_control");
    ros::NodeHandle nh("~");
    Teleop teleop(nh);
    ros::Timer timer_ = nh.createTimer(ros::Duration(0.01),&Teleop::timerCallback,&teleop);

    ros::spin();
    return 0;
}