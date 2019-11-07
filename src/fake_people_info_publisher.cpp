#include <ros/ros.h>
#include <sensor_msgs/TimeReference.h>
#include <sensor_msgs/Image.h>

class FakePeopleInfoPublisher
{
public:
    FakePeopleInfoPublisher();
    void callback(const sensor_msgs::ImageConstPtr);
    void process();

private:
    ros::NodeHandle nh;

    ros::Subscriber image_sub;
    ros::Publisher fake_people_info_pub;
};

FakePeopleInfoPublisher::FakePeopleInfoPublisher()
{
    image_sub = nh.subscribe("/usb_cam/image_raw", 1, &FakePeopleInfoPublisher::callback, this);
    fake_people_info_pub = nh.advertise<sensor_msgs::TimeReference>("people_info", 1);
}

void FakePeopleInfoPublisher::callback(const sensor_msgs::ImageConstPtr image)
{
    sensor_msgs::TimeReference fake_people_info;
    fake_people_info.header = image->header;
    fake_people_info.source = "{\"objects\":[]}";
    fake_people_info_pub.publish(fake_people_info);
}

void FakePeopleInfoPublisher::process()
{
    ros::spin();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "fake_people_info_publisher");
    FakePeopleInfoPublisher faker;
    faker.process();
    return 0;
}