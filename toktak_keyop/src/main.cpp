/*****************************************************************************
** Includes
*****************************************************************************/

#include "../include/keyop_core/keyop_core.hpp"

/*****************************************************************************
** Using
*****************************************************************************/

using keyop_core::KeyOpCore;

/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char **argv)
{
  ros::init(argc, argv, "toktak_keyop");
  KeyOpCore keyop;
  if (keyop.init())
  {
    keyop.spin();
  }
  else
  {
    ROS_ERROR_STREAM("Couldn't initialise KeyOpCore!");
  }

  ROS_INFO_STREAM("Program exiting");
  return 0;
}
