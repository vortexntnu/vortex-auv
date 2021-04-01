
#include "dp_reference_model/reference_model.h"

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "dp_reference_mode");
    ros::NodeHandle nh;
    ReferenceModel referenceModel(nh);
    ros::spin();

    return 0;
}
