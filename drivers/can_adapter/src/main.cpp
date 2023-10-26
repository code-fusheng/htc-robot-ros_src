#include <can_adapter.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "can_adapter_node");
    can_adapter::CanAdapterNode node;
    ros::spin();
    return 0;
}