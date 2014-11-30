/*
 * flight algo that calculate flight path from reading sensor data and determining distance and direction of its target
 * move the drone by calling service from flight_cmd_wrapper
 *
 */

#include <flight_algo/flight_algo.h>
bool flight_interactive( FlightControl *flightControl );
bool flight_auto( FlightControl *flightControl );

enum DroneCMD
{
   LED,
   FLAT_TRIM,
   LAUNCH,
   LAND,
   ADVANCE,
   STRIDE,
   UP,
   TURN,
   HOVER,
   EXIT
};


int main( int argc, char **argv)
{
  bool  programExit = false;
  bool  controlMode = false; // false: interactive CMD line, true: auto

  ros::init( argc, argv, "flight_control" );
  
  ros::NodeHandle n("~");

  ros::Rate loop_rate( 10 );

  n.getParam( "controlMode", controlMode );
  ROS_INFO( "control mode is %d", controlMode );

  //FlightControl flightControl(TTL);
  FlightControl *flightControl = new FlightControl(WIFI);

  while (!ros::ok());
  //system( "rosrun ardrone_autonomy ardrone_driver" );
  ROS_INFO( "waiting for drone..." );
  // wait till drone is ready
  while (!flightControl->check_drone_ready());
  flightControl->print_connection_mode();
  ROS_INFO( "flight_control start" );

  // TODO: start with push button

  // TODO: in automode
  // count down 10 sec and launch

  while (ros::ok() && !programExit)
  {
    if (controlMode)
    {
      programExit = flight_auto( flightControl );
    }
    else
    {

      programExit = flight_interactive( flightControl );
    }
  }

  ros::spinOnce();
  loop_rate.sleep();

  return 0;
}

// return true if flight is done
bool flight_interactive( FlightControl *flightControl )
{
  DroneCMD  cmd;
  int       inputCMD;
  bool      programExit = false;

  cout  << LED        << ": led\n"
        << FLAT_TRIM  << ": flat_trim\n"
        << LAUNCH     << ": launch\n"
        << LAND       << ": land\n"
        << ADVANCE    << ": advance\n"
        << STRIDE     << ": stride\n"
        << UP         << ": up\n"
        << TURN       << ": turn\n"
        << HOVER      << ": hover\n"
        << EXIT       << ": exit\n";
  cin >> inputCMD;
  cout << "I received:" << inputCMD << endl;
  cmd = (DroneCMD)inputCMD;
  switch (cmd)
  {
    case LED:
      flightControl->led_animation();
      break;
    case FLAT_TRIM:
      flightControl->flat_trim();
      break;
    case LAUNCH:
      flightControl->flight_launch();
      break;
    case LAND:
      flightControl->flight_land();
      break;
    case ADVANCE:
      flightControl->flight_advance( 0.1 );
      break;
    case STRIDE:
      flightControl->flight_stride( 0.1 );
      break;
    case UP:
      flightControl->flight_up( 0.1 );
      break;
    case TURN:
      flightControl->flight_turn( -0.1 );
      break;
    case HOVER:
      flightControl->flight_hover();
      break;
    case EXIT:
      ROS_INFO( "Exiting programming" );
      programExit = true;
      break;
    default:
      ROS_INFO( "Wrong CMD" );
      break;
  }

  return programExit;
}

// return true if flight is done
bool flight_auto( FlightControl *flightControl )
{
  bool   programExit = false;

  // wait for 10 sec then launch
  // wait for 10 sec then land

  flightControl->led_animation(); //TODO: launching sequence should use different LED animation
  ros::Duration(10).sleep();
  flightControl->flight_launch();
  ROS_INFO (" launch");
  ros::Duration(10).sleep();
  flightControl->flight_land();
  ROS_INFO ("land");

  programExit = true;

  return programExit;
}

