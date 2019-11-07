#ifndef __MANNEQUIN_DETECTOR_H
#define __MANNEQUIN_DETECTOR_H

#include <omp.h>

#include <ros/ros.h>

#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/TimeReference.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include <opencv2/opencv.hpp>
#include <opencv2/bgsegm.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/foreach.hpp>
#include <boost/optional.hpp>

struct BBox
{
    double x_min = 0;
    double x_max = 0;
    double y_min = 0;
    double y_max = 0;
    double distance = 0;
};

class MannequinDetector
{

public:
    typedef pcl::PointXYZRGB PointType ;
    typedef pcl::PointCloud<PointType> PointCloudType;
    typedef PointCloudType::Ptr PointCloudTypePtr;

    MannequinDetector(void);
    void set_threshold_bibs();
    void set_threshold_face();
    void set_threshold_hats();

    void camera_info_callback(const sensor_msgs::CameraInfoConstPtr&);
    void sensor_callback(const sensor_msgs::ImageConstPtr&, const sensor_msgs::PointCloud2ConstPtr&, const sensor_msgs::TimeReferenceConstPtr&);
    void get_color_from_distance(double, int&, int&, int&);
    void sensor_fusion(const sensor_msgs::Image&, const sensor_msgs::PointCloud2&, const sensor_msgs::TimeReference&);
    void load_bboxes_of_json(const sensor_msgs::TimeReference&, const sensor_msgs::Image&);
    geometry_msgs::PoseStamped get_distance_from_person(PointCloudTypePtr& pc);
    bool is_mannequin(const BBox&, const cv::Mat&, cv::Mat&);
    void make_binarized_image(const cv::Mat&, const std::vector<int>&, const std::vector<int>&, cv::Mat&);
    void sigma_cut(const std::vector<int>&, double, std::vector<int>&);
    void get_euclidean_cluster(PointCloudTypePtr& pc, PointCloudType& output_pc);
    void process(void);

private:
    double MAX_DISTANCE;
	double TOLERANCE;
	double BIBS_RATIO;
	double HAT_RATIO;
	double HEAD_RATIO;
	int MIN_CLUSTER_SIZE;
	int MAX_CLUSTER_SIZE;
    sensor_msgs::CameraInfo camera_info;
    std::vector<BBox> bboxes;

    std::string MAN_BIBS_COLOR;
    std::string WOMAN_BIBS_COLOR;
    std::string CHILD_BIBS_COLOR;
    std::vector<int> bibs_indexes;
    std::vector<std::string> bibs_colors = {"red", "blue", "yellow", "green", "pink", "white"};
    std::string bibses_name[6][2] = {{"LOWER_RED",    "UPPER_RED"   },
                                     {"LOWER_BLUE",   "UPPER_BLUE"  },
                                     {"LOWER_YELLOW", "UPPER_YELLOW"},
                                     {"LOWER_GREEN",  "UPPER_GREEN" },
                                     {"LOWER_PINK",   "UPPER_PINK"  },
                                     {"LOWER_WHITE",  "UPPER_WHITE" } };
    std::string input_bibses_hsv[6][2];
    int BIBSES_HSV[6][2][3]; // [red, blue, yellow, green, pink, white][lower, upper][h, s, v]

    std::string LOWER_FACE;
    std::string UPPER_FACE;
    int FACE_HSV[2][3];

    std::string MAN_HAT_COLOR;
    std::string WOMAN_HAT_COLOR;
    std::string CHILD_HAT_COLOR;
    std::vector<int> hats_indexes;
    std::vector<std::string> hats_colors = {"green", "forest", "red", "orange"};
    std::string hats_name[4][2] = {{"LOWER_HGREEN",  "UPPER_HGREEN"},
                                   {"LOWER_FOREST", "UPPER_FOREST"},
                                   {"LOWER_HRED",    "UPPER_HRED"},
                                   {"LOWER_ORANGE", "UPPER_ORANGE"}};
    std::string input_hats_hsv[4][2];
    int HATS_HSV[4][2][3];

    ros::NodeHandle nh;
    ros::NodeHandle local_nh;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2, sensor_msgs::TimeReference> sensor_fusion_sync_subs;
    message_filters::Subscriber<sensor_msgs::Image> image_sub;
    message_filters::Subscriber<sensor_msgs::PointCloud2> pc_sub;
    message_filters::Subscriber<sensor_msgs::TimeReference> bboxes_sub; // TODO msgの型を自作する
    message_filters::Synchronizer<sensor_fusion_sync_subs> sensor_fusion_sync;

    ros::Subscriber camera_info_sub;
    ros::Publisher image_pub;
    ros::Publisher pc_pub;
    ros::Publisher semantic_cloud_pub;
    ros::Publisher mannequin_pose_pub;

    tf::TransformListener listener;
    Eigen::Affine3d transform;
    typedef std::tuple<int, int, int> ColorTuple;
    std::map<ColorTuple, std::string> color_with_class;
};

#endif // __MANNEQUIN_DETECTOR_H