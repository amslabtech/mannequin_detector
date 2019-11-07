#include "mannequin_detector/mannequin_detector.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mannequin_detector");
    MannequinDetector mannequin_detector;
    mannequin_detector.process();
    return 0;
}