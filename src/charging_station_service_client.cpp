// 可以通过该命令来启动服务：rosservice call /go_to_charging_station
#include <ros/ros.h>
#include <charging_station/GoToChargingStation.h>
int main(int argc, char **argv)
{
    ros::init(argc, argv, "charging_station_service_client");
    ros::NodeHandle nh;

    ros::ServiceClient client = nh.serviceClient<charging_station::GoToChargingStation>("go_to_charging_station");

    charging_station::GoToChargingStation srv;

    // srv.request.some_parameter = some_value;

    if (client.call(srv))
    {
        ROS_INFO("Service call succeeded. Success: %s", srv.response.success ? "true" : "false");
    }
    else
    {
        ROS_ERROR("Failed to call service go_to_charging_station");
        return 1;
    }

    return 0;
}
