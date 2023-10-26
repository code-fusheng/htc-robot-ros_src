#include "ndt_localization/ndt_localization.h"

using namespace NDTLocalization;

int main(int argc, char **argv) {
    ros::init(argc, argv, "ndt_localization_node");

    ros::NodeHandle nh_, pnh_("~");

    init(nh_, pnh_);

    ros::spin();
    return 0;
}
