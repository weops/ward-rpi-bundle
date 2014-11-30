#ifndef _FLIGHT_CONTROL_H_
#define _FLIGHT_CONTROL_H_
#include "std_msgs/Empty.h"
#include "std_srvs/Empty.h"
#include <ardrone_autonomy/LedAnim.h>
#include "geometry_msgs/Twist.h"

enum ConnectMode
{
  WIFI,
  TTL
};


class FlightControl
{
  public:
  private:
    ConnectMode         connectMode;
    ros::NodeHandle     n;
    ros::ServiceClient  srvLedClient;
    ros::ServiceClient  srvFlatTrimClient;
    ros::Publisher      msgLandPublisher;
    ros::Publisher      msgLaunchPublisher;
    ros::Publisher      msgMovePublisher;

  public:
    FlightControl( ConnectMode mode ); // hide default constructor
    void led_animation();
    void flat_trim();
    void flight_launch();
    void flight_land();
    void print_connection_mode();
    void flight_advance( double magnitude );
    void flight_stride( double magnitude );
    void flight_up( double magnitude );
    void flight_turn( double magnitude );
    void flight_hover();

    bool check_drone_ready();

  private:
};

#endif
