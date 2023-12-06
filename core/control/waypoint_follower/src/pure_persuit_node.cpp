#include "pure_persuit.h"
#include <ros/console.h>


int main(int argc, char **argv) {
    ros::init(argc, argv, "pure_pursuit_node");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
    waypoint_follower::PurePursuitNode app;
    app.run();
    return 0;
}
