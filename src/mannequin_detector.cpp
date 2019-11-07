#include "mannequin_detector/mannequin_detector.h"

MannequinDetector::MannequinDetector() : local_nh("~"), image_sub(nh, "/usb_cam/image_raw", 30), pc_sub(nh, "/velodyne_points", 30), bboxes_sub(nh, "/people_info", 30), sensor_fusion_sync(sensor_fusion_sync_subs(30), image_sub, pc_sub, bboxes_sub)
{
    camera_info_sub = nh.subscribe("/camera_info", 1, &MannequinDetector::camera_info_callback, this);
    mannequin_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/mannequin_position", 1);
    image_pub = nh.advertise<sensor_msgs::Image>("/projection/raw", 1);
    pc_pub = nh.advertise<sensor_msgs::PointCloud2>("/cloud/colored/raw", 1);
    semantic_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/cloud/semantic/raw", 1);
    sensor_fusion_sync.registerCallback(boost::bind(&MannequinDetector::sensor_callback, this, _1, _2, _3));
    
    local_nh.param("MAX_DISTANCE", MAX_DISTANCE, {20.0});
	local_nh.param("TOLERANCE", TOLERANCE, 0.20);
	local_nh.param("BIBS_RATIO", BIBS_RATIO, 0.20);
	local_nh.param("HAT_RATIO", HAT_RATIO, 0.02);
	local_nh.param("HEAD_RATIO", HEAD_RATIO, 0.05);
	local_nh.param("MIN_CLUSTER_SIZE", MIN_CLUSTER_SIZE, 20);
	local_nh.param("MAX_CLUSTER_SIZE", MAX_CLUSTER_SIZE, 900);
    local_nh.param("MAN_BIBS_COLOR", MAN_BIBS_COLOR, {"red"});
    local_nh.param("WOMAN_BIBS_COLOR", WOMAN_BIBS_COLOR, {"red"});
    local_nh.param("CHILD_BIBS_COLOR", CHILD_BIBS_COLOR, {"red"});
    local_nh.param("MAN_HAT_COLOR", MAN_HAT_COLOR, {"red"});
    local_nh.param("WOMAN_HAT_COLOR", WOMAN_HAT_COLOR, {"red"});
    local_nh.param("CHILD_HAT_COLOR", CHILD_HAT_COLOR, {"red"});
    for(int i=0; i<6; i++)
    {
        for (int j=0; j<2; j++)
        {
            local_nh.param(bibses_name[i][j], input_bibses_hsv[i][j], {"0,0,0"});
        }
    }
    set_threshold_bibs();
    auto man_bibs_itr = std::find(bibs_colors.begin(), bibs_colors.end(), MAN_BIBS_COLOR);
    bibs_indexes.push_back(std::distance(bibs_colors.begin(), man_bibs_itr));
    auto woman_bibs_itr = std::find(bibs_colors.begin(), bibs_colors.end(), WOMAN_BIBS_COLOR);
    bibs_indexes.push_back(std::distance(bibs_colors.begin(), woman_bibs_itr));
    auto child_bibs_itr = std::find(bibs_colors.begin(), bibs_colors.end(), CHILD_BIBS_COLOR);
    bibs_indexes.push_back(std::distance(bibs_colors.begin(), child_bibs_itr));

    local_nh.param("LOWER_FACE", LOWER_FACE, {"0,0,0"});
    local_nh.param("UPPER_FACE", UPPER_FACE, {"0,0,0"});
    set_threshold_face();

    for(int i=0; i<4; i++)
    {
        for (int j=0; j<2; j++)
        {
            local_nh.param(hats_name[i][j], input_hats_hsv[i][j], {"0,0,0"});
        }
    }
    set_threshold_hats();
    auto man_hat_itr = std::find(hats_colors.begin(), hats_colors.end(), MAN_HAT_COLOR);
    hats_indexes.push_back(std::distance(hats_colors.begin(), man_hat_itr));
    auto woman_hat_itr = std::find(hats_colors.begin(), hats_colors.end(), WOMAN_HAT_COLOR);
    hats_indexes.push_back(std::distance(hats_colors.begin(), woman_hat_itr));
    auto child_hat_itr = std::find(hats_colors.begin(), hats_colors.end(), CHILD_HAT_COLOR);
    hats_indexes.push_back(std::distance(hats_colors.begin(), child_hat_itr));
    

    std::cout << "=== mannequin_detector ===" << std::endl;
    std::cout << "MAX_DISTANCE: " << MAX_DISTANCE << std::endl;
	std::cout << "TOLERANCE: " << TOLERANCE << std::endl;
	std::cout << "BIBS_RATIO: " << BIBS_RATIO << std::endl;
	std::cout << "HAT_RATIO: " << HAT_RATIO << std::endl;
	std::cout << "HEAD_RATIO: " << HEAD_RATIO << std::endl;
	std::cout << "MIN_CLUSTER_SIZE: " << MIN_CLUSTER_SIZE << std::endl;
	std::cout << "MAX_CLUSTER_SIZE: " << MAX_CLUSTER_SIZE << std::endl;
    std::cout << "MAN_BIBS_COLOR: " << MAN_BIBS_COLOR << std::endl;
    std::cout << "WOMAN_BIBS_COLOR: " << WOMAN_BIBS_COLOR << std::endl;
    std::cout << "CHILD_BIBS_COLOR: " << CHILD_BIBS_COLOR << std::endl;
    for(int i=0; i<6; i++)
    {
        for (int j=0; j<2; j++)
        {
            std::cout << bibses_name[i][j] << ": ";
            for (int k=0; k<3; k++)
            {
                std::cout << BIBSES_HSV[i][j][k];
                if(k != 2)
                {
                    std::cout << ",";
                }
                else
                {
                    std::cout << std::endl;
                }
            }
        }
    }
    std::cout << "LOWER_FACE: " << FACE_HSV[0][0] << "," << FACE_HSV[0][1] << "," << FACE_HSV[0][2] << std::endl;
    std::cout << "UPPER_FACE: " << FACE_HSV[1][0] << "," << FACE_HSV[1][1] << "," << FACE_HSV[1][2] << std::endl;
    std::cout << "MAN_HAT_COLOR: " << MAN_HAT_COLOR << std::endl;
    std::cout << "WOMAN_HAT_COLOR: " << WOMAN_HAT_COLOR << std::endl;
    std::cout << "CHILD_HAT_COLOR: " << CHILD_HAT_COLOR << std::endl;
    for(int i=0; i<4; i++)
    {
        for (int j=0; j<2; j++)
        {
            std::cout << hats_name[i][j] << ": ";
            for (int k=0; k<3; k++)
            {
                std::cout << HATS_HSV[i][j][k];
                if(k != 2)
                {
                    std::cout << ",";
                }
                else
                {
                    std::cout << std::endl;
                }
            }
        }
    }
}

void MannequinDetector::set_threshold_bibs()
{
    for (int i=0; i<6; i++)
    {
        for(int j=0; j<2; j++)
        {
            std::vector<std::string> str_int_vec;
            std::stringstream ss{input_bibses_hsv[i][j]};
            std::string buf;
            while(std::getline(ss, buf, ','))
            {
                std::string none_space_buf;
                for (auto c : buf)
                {
                    if(c != ' ')
                    {
                        none_space_buf.push_back(c);
                    }
                }
                str_int_vec.push_back(none_space_buf);
            }

            if (str_int_vec.size() != 3)
            {
                ROS_ASSERT("mannequin_detector launch : ros param error");
            }

            for (int k=0; k<3; k++)
            {
                BIBSES_HSV[i][j][k] = atoi(str_int_vec[k].c_str());
            }
        }
    }
}

void MannequinDetector::set_threshold_face()
{
    std::vector<std::string> lower_str_int_vec;
    std::stringstream lower_ss{LOWER_FACE};
    std::string lower_buf;
    while(std::getline(lower_ss, lower_buf, ','))
    {
        std::string none_space_buf;
        for (auto c : lower_buf)
        {
            if(c != ' ')
            {
                none_space_buf.push_back(c);
            }
        }
        lower_str_int_vec.push_back(none_space_buf);
    }

    if (lower_str_int_vec.size() != 3)
    {
        ROS_ASSERT("mannequin_detector launch : ros param error");
    }

    for (int i=0; i<3; i++)
    {
        FACE_HSV[0][i] = atoi(lower_str_int_vec[i].c_str());
    }

    std::vector<std::string> upper_str_int_vec;
    std::stringstream upper_ss{UPPER_FACE};
    std::string upper_buf;
    while(std::getline(upper_ss, upper_buf, ','))
    {
        std::string none_space_buf;
        for (auto c : upper_buf)
        {
            if(c != ' ')
            {
                none_space_buf.push_back(c);
            }
        }
        upper_str_int_vec.push_back(none_space_buf);
    }

    if (upper_str_int_vec.size() != 3)
    {
        ROS_ASSERT("mannequin_detector launch : ros param error");
    }

    for (int i=0; i<3; i++)
    {
        FACE_HSV[1][i] = atoi(upper_str_int_vec[i].c_str());
    }
}

void MannequinDetector::set_threshold_hats()
{
    for (int i=0; i<4; i++)
    {
        for(int j=0; j<2; j++)
        {
            std::vector<std::string> str_int_vec;
            std::stringstream ss{input_hats_hsv[i][j]};
            std::string buf;
            while(std::getline(ss, buf, ','))
            {
                std::string none_space_buf;
                for (auto c : buf)
                {
                    if(c != ' ')
                    {
                        none_space_buf.push_back(c);
                    }
                }
                str_int_vec.push_back(none_space_buf);
            }

            if (str_int_vec.size() != 3)
            {
                ROS_ASSERT("mannequin_detector launch : ros param error");
            }

            for (int k=0; k<3; k++)
            {
                HATS_HSV[i][j][k] = atoi(str_int_vec[k].c_str());
            }
        }
    }
}

void MannequinDetector::camera_info_callback(const sensor_msgs::CameraInfoConstPtr& msg)
{
    camera_info = *msg;
}

void MannequinDetector::sensor_callback(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::PointCloud2ConstPtr& pc, const sensor_msgs::TimeReferenceConstPtr& bboxes_msg)
{
    try
    {
        tf::StampedTransform transform_;
        listener.waitForTransform("front_camera/optical_frame", pc->header.frame_id, ros::Time(0), ros::Duration(8.0));
        listener.lookupTransform("front_camera/optical_frame", pc->header.frame_id, ros::Time(0), transform_);
        tf::transformTFToEigen(transform_, transform);

        sensor_fusion(*image, *pc, *bboxes_msg);
    }
    catch(tf::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
    }
}

void MannequinDetector::get_color_from_distance(double distance, int& r, int& g, int& b)
{
    r = 255;
    g = 255;
    b = 255;

    distance = std::max(std::min(distance, MAX_DISTANCE), 0.0);

    double v = distance / MAX_DISTANCE * 255;// 0 ~ 255

    if(v < (0.25 * 255))
    {
        r = 0;
        g = 4 * v;
    }
    else if(v < 0.5 * 255)
    {
        r = 0;
        b = 255 + 4 * (0.25 * 255 - v);
    }
    else if(v < 0.75 * 255)
    {
        r = 4 * (v - 0.5 * 255);
        b = 0;
    }
    else
    {
        g = 255 + 4 * (0.75 * 255 - v);
        b = 0;
    }
}

void MannequinDetector::sensor_fusion(const sensor_msgs::Image& image, const sensor_msgs::PointCloud2& pc, const sensor_msgs::TimeReference& bboxes_msg)
{
    double start_time = ros::Time::now().toSec();
    load_bboxes_of_json(bboxes_msg, image);


    pcl::PointCloud<pcl::PointXYZI>::Ptr lidar_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(pc, *lidar_cloud);

    PointCloudTypePtr cloud(new PointCloudType);
    pcl::copyPointCloud(*lidar_cloud, *cloud);

    PointCloudTypePtr trans_cloud(new PointCloudType);
    pcl::transformPointCloud(*cloud, *trans_cloud, transform);

    cv_bridge::CvImageConstPtr cv_img_ptr;
    try
    {
        cv_img_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
    }
    catch(cv_bridge::Exception& ex)
    {
        ROS_ERROR("cv_bridge exception: %s", ex.what());
        return;
    }

    cv::Mat cv_image(cv_img_ptr->image.rows, cv_img_ptr->image.cols, cv_img_ptr->image.type());
    cv_image = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8)->image;

    image_geometry::PinholeCameraModel cam_model;
    cam_model.fromCameraInfo(camera_info);

    
    cv::Mat hsv_image;
    cv::cvtColor(cv_image, hsv_image, CV_BGR2HSV);

    // colored point
    cv::Mat projection_image = cv_image.clone();
    PointCloudTypePtr colored_cloud(new PointCloudType);
    *colored_cloud = *trans_cloud;

    for(auto& pt : colored_cloud->points)
    {
        if(pt.z<0)
        {
            // behind camera
            pt.b = 255;
            pt.g = 255;
            pt.r = 255;
        }
        else
        {
            cv::Point3d pt_cv(pt.x, pt.y, pt.z);
            cv::Point2d uv;
            uv = cam_model.project3dToPixel(pt_cv);

            if(uv.x > 0 && uv. x < cv_image.cols && uv.y > 0 && uv.y < cv_image.rows)
            {
                pt.b = cv_image.at<cv::Vec3b>(uv)[0];
                pt.g = cv_image.at<cv::Vec3b>(uv)[1];
                pt.r = cv_image.at<cv::Vec3b>(uv)[2];

                double distance = sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
                int r, g, b;
                get_color_from_distance(distance, r, g, b);
                cv::circle(projection_image, uv, 1, cv::Scalar(b, g, r), -1);
            }
            else
            {
                pt.b = 255;
                pt.g = 255;
                pt.r = 255;
            }
        }
    }


    PointCloudTypePtr semantic_cloud(new PointCloudType);
    for (auto bbox : bboxes)
    {
        // get point cloud in bbox
        PointCloudTypePtr target_points(new PointCloudType);
        cv::Point2d bbox_center((bbox.x_min + bbox.x_max) / 2, (bbox.y_min + bbox.y_max) / 2);
        for(auto& pt : colored_cloud->points)
        {
            if(pt.z > 0)
            {
                cv::Point3d pt_cv(pt.x, pt.y, pt.z);
                cv::Point2d uv;
                uv = cam_model.project3dToPixel(pt_cv);

                if(bbox.x_min < uv.x && uv.x < bbox.x_max && bbox.y_min < uv.y && uv.y < bbox.y_max)
                {
                    target_points->points.push_back(pt);
                }
            }
        }

        // clustering
        PointCloudTypePtr cluster(new PointCloudType);
        get_euclidean_cluster(target_points, *cluster);
        *semantic_cloud += *cluster;

        cv::Point2d rectangle_xmin_ymin(bbox.x_min, bbox.y_min);
        cv::Point2d rectangle_xmax_ymax(bbox.x_max, bbox.y_max);
        // mannequin
        if(is_mannequin(bbox, hsv_image, projection_image) && !cluster->empty())
        {
            PointCloudTypePtr transformed_cluster(new PointCloudType);
            pcl::transformPointCloud(*cluster, *transformed_cluster, transform.inverse());
            geometry_msgs::PoseStamped mannequin_pose = get_distance_from_person(transformed_cluster);
            double distance_to_mannequin = sqrt(mannequin_pose.pose.position.x * mannequin_pose.pose.position.x + mannequin_pose.pose.position.y * mannequin_pose.pose.position.y);
            if (distance_to_mannequin < MAX_DISTANCE)
            {
                std::cout << "mannequin_pose : " << mannequin_pose << std::endl;
                mannequin_pose.header = pc.header;
                mannequin_pose_pub.publish(mannequin_pose);
                cv::rectangle(projection_image, rectangle_xmin_ymin, rectangle_xmax_ymax, cv::Scalar(0, 0, 255));
            }
            else
            {
                cv::rectangle(projection_image, rectangle_xmin_ymin, rectangle_xmax_ymax, cv::Scalar(255, 0, 0));
            }
            
            
        }
        else
        {
            cv::rectangle(projection_image, rectangle_xmin_ymin, rectangle_xmax_ymax, cv::Scalar(255, 0, 0));
        }
        
    }

    // colored cloud
    PointCloudTypePtr output_cloud(new PointCloudType);
    pcl::transformPointCloud(*colored_cloud, *output_cloud, transform.inverse());

    sensor_msgs::PointCloud2 output_pc;
    pcl::toROSMsg(*output_cloud, output_pc);
    output_pc.header = pc.header;
    pc_pub.publish(output_pc);

    // semantic cloud
    PointCloudTypePtr output_semantic_cloud(new PointCloudType);
    pcl::transformPointCloud(*semantic_cloud, *output_semantic_cloud, transform.inverse());

    sensor_msgs::PointCloud2 output_semantic_pc;
    pcl::toROSMsg(*output_semantic_cloud, output_semantic_pc);
    output_semantic_pc.header = pc.header;
    semantic_cloud_pub.publish(output_semantic_pc);


    // projection/depth image
    sensor_msgs::ImagePtr output_image;
    output_image = cv_bridge::CvImage(std_msgs::Header(), "bgr8", projection_image).toImageMsg();
    output_image->header = image.header;
    image_pub.publish(output_image);

    std::cout << ros::Time::now().toSec() - start_time << "[s]" << std::endl;
}

void MannequinDetector::load_bboxes_of_json(const sensor_msgs::TimeReference& msg, const sensor_msgs::Image& image)
{
    boost::property_tree::ptree pt;
    std::stringstream ss;
    ss << msg.source;
    boost::property_tree::read_json(ss, pt);

    bboxes.clear();
    BOOST_FOREACH (const boost::property_tree::ptree::value_type& child, pt.get_child("objects"))
    {
        const boost::property_tree::ptree& objects = child.second;

        BBox bbox;
        for (auto elem : objects.get_child("bbox"))
        {
            // std::cout << elem.first.data() << " to " << elem.second.data() << std::endl;
            if (bbox.x_min == 0) bbox.x_min = std::atof(elem.second.data().c_str());
            else if (bbox.y_min == 0) bbox.y_min = std::atof(elem.second.data().c_str());
            else if (bbox.x_max == 0) bbox.x_max = std::atof(elem.second.data().c_str());
            else if (bbox.y_max == 0) bbox.y_max = std::atof(elem.second.data().c_str());
        }
        if (bbox.x_min > bbox.x_max)
        {
            double temp = bbox.x_min;
            bbox.x_min = bbox.x_max;
            bbox.x_max = temp;
        }
        if (bbox.y_min > bbox.y_max)
        {
            double temp = bbox.y_min;
            bbox.y_min = bbox.y_max;
            bbox.y_max = temp;
        }
        if (bbox.x_min < 0) bbox.x_min = 0;
        if (bbox.y_min < 0) bbox.y_min = 0;
        if (bbox.x_max > image.width) bbox.x_max = image.width;
        if (bbox.y_max > image.height) bbox.y_max = image.height;
        bboxes.push_back(bbox);

        if (boost::optional<double> score = objects.get_optional<double>("score"))
        {
            // std::cout << score.get() << std::endl;
        }
        else
        {
            // std::cout << "no score" << std::endl;
        }
    }
}

void MannequinDetector::get_euclidean_cluster(PointCloudTypePtr& pc, PointCloudType& output_pc)
{
	//std::cout << "original points size: " << pc.points.size() << std::endl;
	if(pc->points.empty())
    {
		return;
	}

	PointCloudTypePtr cloud_filtered(new PointCloudType);
	pcl::copyPointCloud(*pc, *cloud_filtered);

	typename pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>);
	tree->setInputCloud(cloud_filtered);

	std::vector<pcl::PointIndices> cluster_indices;
	typename pcl::EuclideanClusterExtraction<PointType> ec;
	ec.setClusterTolerance(TOLERANCE);
	ec.setMinClusterSize(MIN_CLUSTER_SIZE);
	ec.setMaxClusterSize(MAX_CLUSTER_SIZE);
	ec.setSearchMethod(tree);
	ec.setInputCloud(cloud_filtered);
	ec.extract(cluster_indices);

	int max_cluster_size = 0;
	// int max_cluster_index = -1;
	int index = 0;
	for(auto indices : cluster_indices){
		PointCloudTypePtr cluster(new PointCloudType);
		for(auto it : indices.indices){
			cluster->points.push_back(cloud_filtered->points.at(it));
		}
		int cluster_size = cluster->points.size();
		//std::cout << "cluster" << index << " size: " << cluster_size << std::endl;
		if(cluster_size > max_cluster_size){
			// max_cluster_index = index;
			max_cluster_size = cluster_size;
			output_pc = *cluster;
		}
		index++;
	}
}

geometry_msgs::PoseStamped MannequinDetector::get_distance_from_person(PointCloudTypePtr& pc)
{
    double sum_x = 0.0;
    double sum_y = 0.0;
    double sum_z = 0.0;
    for (auto& pt : pc->points)
    {
        sum_x += pt.x;
        sum_y += pt.y;
        sum_z += pt.z;
    }

    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.pose.position.x = sum_x / pc->size();
    pose_stamped.pose.position.y = sum_y / pc->size();
    pose_stamped.pose.position.z = sum_z / pc->size();
    return pose_stamped;
}

bool MannequinDetector::is_mannequin(const BBox& bbox, const cv::Mat& image, cv::Mat& projection_image)
{
    double height = bbox.y_max - bbox.y_min;
    cv::Mat bibs_image(image, cv::Rect(cv::Point2i(bbox.x_min, bbox.y_min + height / 4), cv::Point2i(bbox.x_max, bbox.y_max)));
    double min_y = -1;
    int mannequin_bibs_index = -1;
    bool is_mannequin_decided_by_bibs = false;
    for (auto index : bibs_indexes)
    {
        std::cout << bibs_colors[index] << " bibs check" << std::endl;
        std::vector<int> lower(std::begin(BIBSES_HSV[index][0]), std::end(BIBSES_HSV[index][0]));
        std::vector<int> upper(std::begin(BIBSES_HSV[index][1]), std::end(BIBSES_HSV[index][1]));
        cv::Mat binarized_image;
        make_binarized_image(bibs_image, lower, upper, binarized_image);
        // std::cout << binarized_image << std::endl;
        
        int count = 0;
        std::vector<int> mannequin_y_list;
        for (int y=0; y < binarized_image.rows; y++)
        {
            for (int x=0; x < binarized_image.cols; x++)
            {
                if (binarized_image.at<uchar>(y, x) == 255)
                {
                    projection_image.at<cv::Vec3b>(bbox.y_min + y, bbox.x_min + x)[0] = lower[0];
                    projection_image.at<cv::Vec3b>(bbox.y_min + y, bbox.x_min + x)[1] = lower[1];
                    projection_image.at<cv::Vec3b>(bbox.y_min + y, bbox.x_min + x)[2] = lower[2];
                    //std::cout << "x : " << x << ", y : " << y << ", count : " << count << std::endl;
                    mannequin_y_list.push_back(y);
                    count++;
                }
            }
        }
        double sigma = 2.0;
        std::vector<int> sigma_cuted_ys;
        sigma_cut(mannequin_y_list, sigma, sigma_cuted_ys);
        if (!sigma_cuted_ys.empty())
        {
            min_y = *std::min_element(sigma_cuted_ys.begin(), sigma_cuted_ys.end());
        }
        double percentage = (double)count / (double)(binarized_image.rows * binarized_image.cols);
        std::cout << "percentage = " << percentage << std::endl;
        mannequin_bibs_index++;
        if (percentage > BIBS_RATIO)
        {
            is_mannequin_decided_by_bibs = true;
            if(mannequin_bibs_index == 0)
            {
                std::cout << "man" << std::endl;
            }
            else if(mannequin_bibs_index == 1)
            {
                std::cout << "woman" << std::endl;
            }
            else
            {
                std::cout << "child" << std::endl;
            }
            std::cout << "bibs check clear. percentage = " << percentage << std::endl;
            break;
        }
    }
    if(!is_mannequin_decided_by_bibs || min_y == -1)
    {
        return false;
    }

    double width = bbox.x_max - bbox.x_min;
    cv::Rect face_rect(cv::Point2d(bbox.x_min + width / 4, bbox.y_min), cv::Point2d(bbox.x_max - width / 4, bbox.y_min + min_y));
    cv::rectangle(projection_image, face_rect, cv::Scalar(0, 255, 0));
    cv::Mat face_image(image, face_rect);

    std::vector<int> face_lower(std::begin(FACE_HSV[0]), std::end(FACE_HSV[0]));
    std::vector<int> face_upper(std::begin(FACE_HSV[1]), std::end(FACE_HSV[1]));
    cv::Mat face_binarized_image;
    cv::inRange(face_image, face_lower, face_upper, face_binarized_image);
    
    int face_count = 0;
    for (int y=0; y <face_binarized_image.rows; y++)
    {
        for (int x=0; x <face_binarized_image.cols; x++)
        {
            if (face_binarized_image.at<uchar>(y, x) == 255)
            {
                projection_image.at<cv::Vec3b>(bbox.y_min + y, bbox.x_min + x)[0] = face_lower[0];
                projection_image.at<cv::Vec3b>(bbox.y_min + y, bbox.x_min + x)[1] = face_lower[1];
                projection_image.at<cv::Vec3b>(bbox.y_min + y, bbox.x_min + x)[2] = face_lower[2];
                //std::cout << "x : " << x << ", y : " << y << ", count : " << count << std::endl;
                face_count++;
            }
        }
    }
    double face_percentage = (double)face_count / (double)(face_binarized_image.rows * face_binarized_image.cols);
    // std::cout << "counter parsent : " << mm << std::endl; 
    // std::cout << count << ":" << binarized_image.cols << ":" << binarized_image.rows << ":" << mm << std::endl;
    if (face_percentage < HEAD_RATIO)
    {
        return false;
    }
    std::cout << "head check clear. percentage = " << face_percentage << std::endl;

    std::vector<int> hat_lower(std::begin(HATS_HSV[mannequin_bibs_index][0]), std::end(HATS_HSV[mannequin_bibs_index][0]));
    std::vector<int> hat_upper(std::begin(HATS_HSV[mannequin_bibs_index][1]), std::end(HATS_HSV[mannequin_bibs_index][1]));
    cv::Mat hat_binarized_image;
    cv::inRange(face_image, hat_lower, hat_upper, hat_binarized_image);
    // std::cout << binarized_image << std::endl;
    
    int hat_count = 0;
    for (int y=0; y < hat_binarized_image.rows; y++)
    {
        for (int x=0; x < hat_binarized_image.cols; x++)
        {
            if (hat_binarized_image.at<uchar>(y, x) == 255)
            {
                projection_image.at<cv::Vec3b>(bbox.y_min + y, bbox.x_min + x)[0] = hat_lower[0];
                projection_image.at<cv::Vec3b>(bbox.y_min + y, bbox.x_min + x)[1] = hat_lower[1];
                projection_image.at<cv::Vec3b>(bbox.y_min + y, bbox.x_min + x)[2] = hat_lower[2];
                //std::cout << "x : " << x << ", y : " << y << ", count : " << count << std::endl;
                hat_count++;
            }
        }
    }
    double hat_percentage = (double)hat_count / (double)(hat_binarized_image.rows * hat_binarized_image.cols);
    // std::cout << "counter parsent : " << mm << std::endl; 
    // std::cout << count << ":" << binarized_image.cols << ":" << binarized_image.rows << ":" << mm << std::endl;
    if (hat_percentage > HAT_RATIO)
    {
        std::cout << "hat check clear. percentage = " << hat_percentage << std::endl;
        return true;
    }
    return false;
}

void MannequinDetector::make_binarized_image(const cv::Mat& src_image, const std::vector<int>& lower, const std::vector<int>& upper, cv::Mat& binarized_image)
{
    if (lower[0] > 0)
    {
        cv::inRange(src_image, lower, upper, binarized_image);
        return;
    }
    std::vector<int> under_lower{0, lower[1], lower[2]};
    std::vector<int> under_upper{upper[0], upper[1], upper[2]};
    cv::Mat under_binarized_image;
    cv::inRange(src_image, under_lower, under_upper, under_binarized_image);

    std::vector<int> over_lower{180 + lower[0], lower[1], lower[2]};
    std::vector<int> over_upper{179, upper[1],upper[2]};
    cv::Mat over_binarized_image;
    cv::inRange(src_image, over_lower, over_upper, over_binarized_image);

    binarized_image = under_binarized_image | over_binarized_image;
    return;
}

void MannequinDetector::sigma_cut(const std::vector<int>& xs, double sigma, std::vector<int>& output_xs)
{
    double sum = 0;
    for(auto& x: xs)
    {
        sum += x;
    }
    double mean = sum / xs.size();

    double sum_of_squares = 0;
    for(auto& x: xs)
    {
        sum_of_squares += (x - mean) * (x - mean);
    }
    double stdev = sqrt(sum_of_squares / xs.size());

    output_xs.clear();
    for(auto& x : xs)
    {
        if (abs(x - mean) < stdev * sigma)
        {
            output_xs.push_back(x);
        }
    }
}

void MannequinDetector::process(void)
{
    ros::spin();
}
