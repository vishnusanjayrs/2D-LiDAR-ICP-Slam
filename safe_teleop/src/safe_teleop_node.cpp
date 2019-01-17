/**
 * @file safe_teleop_node.cpp
 * @brief ROS Node using the safe teleop library
 * Created by rakesh on 28/09/18.
 */

#include <ncurses.h>
#include <cstring>

#include <ros/ros.h>
#include <safe_teleop/safe_teleop.h>

int main(int argc, char **argv)
{
  // initialize ROS
  ros::init(argc, argv, "safe_teleop_node");

  // call ros spin in a different thread without blocking other things
  ros::AsyncSpinner async_spinner(2);
  async_spinner.start();

  // initialize curses library for keypress
  initscr();
  noecho(); // doesn't output the key pressed to the terminal
  timeout(3000); // return -1 if no keypress happens for 3000 ms

  safe_teleop::SafeTeleop safe_teleop_handler;

  printw((
    std::string("Moving your robot \n\r") +
    std::string("\t i \n\r") +
    std::string("j \t k \t l \n\r") +
    std::string("\t , \n\n\r") +
    std::string("w/x: increase/decrease linear speed \n\r") +
    std::string("e/c: increase/decrease angular speed \n\r")
  ).c_str());

  while (ros::ok())
  {
    auto keypress = getch();

    if (keypress != -1)
    {
      switch (std::tolower((char)keypress))
      {
        case 'i':
          // forward
          safe_teleop_handler.moveForward();
          break;
        case ',':
          // backward
          safe_teleop_handler.moveBackward();
          break;
        case 'j':
          // counter-clockwise
          safe_teleop_handler.rotateCounterClockwise();
          break;
        case 'l':
          // clockwise
          safe_teleop_handler.rotateClockwise();
          break;
        case 'k':
          // stop
          safe_teleop_handler.stop();
          break;
        case 'w':
          // increase linear speed
          safe_teleop_handler.increaseLinearSpeed();
          break;
        case 'x':
          // decrease linear speed
          safe_teleop_handler.decreaseLinearSpeed();
          break;
        case 'e':
          // increase angular speed
          safe_teleop_handler.increaseAngularSpeed();
          break;
        case 'c':
          // decrease angular speed
          safe_teleop_handler.decreaseAngularSpeed();
          break;
        default:
          ROS_INFO("Unrecognized keypress\r");
          break;
      }
    }
  }

  // close curses window
  endwin();

  return 0;
}