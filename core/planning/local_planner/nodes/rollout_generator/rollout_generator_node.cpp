#include "rollout_generator/rollout_generator.h"

using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rollout_generator_node");
    RolloutGeneratorNS::RolloutGenerator app;
    app.run();
    return 0;
}
