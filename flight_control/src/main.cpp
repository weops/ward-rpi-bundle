/*
 * dummy wrapper
 */

#include <flight_control/flight_control.h>

int main( int argc, char **argv)
{
  ros::init( argc, argv, "flight_control" );
  
  ros::NodeHandle n("~");

  return 0;
}
