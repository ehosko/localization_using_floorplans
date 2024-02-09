#include "../../include/localization/simple_localizer.h"
// #include "../../include/localization/cloudGenerator/source_generator.h"
// #include "../../include/localization/cloudGenerator/target_generator.h"


SimpleLocalizer::SimpleLocalizer() : tfListener_(tfBuffer_)
{
    // Constructor
}

SimpleLocalizer::~SimpleLocalizer()
{
    // Destructor
}

void SimpleLocalizer::setupFromParam()
{
    std::string floorplan_path;
    double ray_length;

    std::cout << "Setting up from param..." << std::endl;

    nh_.getParam("floorplan_node/floorplan_path", floorplan_path);
    
    nh_.getParam("floorplan_node/d_l", d_l_);
    nh_.getParam("floorplan_node/theta_l", theta_l_);

    std::string filename;
    nh_.getParam("floorplan_node/log_file", filename);

    sourceGenerator_ = SourceGenerator();
    sourceGenerator_.initSourceGenerator(nh_);
    targetGenerator_ = TargetGenerator(floorplan_path);
    targetGenerator_.initTargetGenerator(nh_);

    // Known inital position and orientation
    position_ = Eigen::Vector3d(0, 0, 0);
    orientation_ = Eigen::Quaterniond(1, 0, 0, 0);
    transformationMatrix_ = Eigen::Matrix4d::Identity();

    std::cout << "Floorplan path: " << floorplan_path << std::endl;

    // Set up subscribers
    // odomSub_ = nh_.subscribe("/rovioli/odom_T_M_I", 1, &SimpleLocalizer::odomCallback, this);
    // cloudSub_ = nh_.subscribe("/isaac/isaac_sensor_model/isaac_sensor_out", 1, &SimpleLocalizer::cloudCallback, this);


    // Create subscribers for the topics you want to synchronize
    message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub(nh_, "/isaac/isaac_sensor_model/isaac_sensor_out", 1);
    message_filters::Subscriber<nav_msgs::Odometry> odom_sub(nh_, "/rovioli/odom_T_M_I", 1);

    // Synchronize the messages using ApproximateTime with a one-second time tolerance
    //typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry> SyncPolicy;
    typedef message_filters::sync_policies::ExactTime<sensor_msgs::PointCloud2, nav_msgs::Odometry> SyncPolicy;
    message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(10), cloud_sub, odom_sub);

    // Register the callback to be called when synchronized messages are received
    sync.registerCallback(boost::bind(&SimpleLocalizer::syncCallback,boost::ref(*this), _1, _2));

    // Set up publishers for transformation
    //transformationPub_ = nh_.advertise<geometry_msgs::TransformStamped>("/floorplan/transformation", 1);

    // Set up logging file
    transformationFile_.open(filename);
    if(!transformationFile_.is_open())
    {
        std::cout << "Error opening file" << std::endl;
    }
    else
    {
        transformationFile_ << "x y z" << std::endl;
    }

    ros::spin();

}

int SimpleLocalizer::computeTransformationGICP(pcl::PointCloud<pcl::PointXYZ> sourceCloud, 
                                                pcl::PointCloud<pcl::PointXYZ> targetCloud,
                                                Eigen::Matrix4d& transformationMatrix)
{
    // Compute transformation matrix using GICP
    std::cout << "Computing transformation matrix using GICP..." << std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr sourceCloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
    *sourceCloudPtr = sourceCloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr targetCloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
    *targetCloudPtr = targetCloud;

    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
    gicp.setInputSource(sourceCloudPtr);
    gicp.setInputTarget(targetCloudPtr);

    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_source =
      boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    //gicp.align(*aligned_source, transformationMatrix.cast<float>());
    gicp.align(*aligned_source);

    transformationMatrix = gicp.getFinalTransformation().cast<double>();

    pcl::PCDWriter wS;
    std::string file_name = "/home/michbaum/Projects/floorplanLocalization/data/SourceCloud.pcd";
    wS.write<pcl::PointXYZ> (file_name, sourceCloud, false);

    pcl::PCDWriter wT;
    file_name = "/home/michbaum/Projects/floorplanLocalization/data/TargetCloud.pcd";
    wT.write<pcl::PointXYZ> (file_name, targetCloud, false);


    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_source(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(sourceCloud, *transformed_source, transformationMatrix.cast<float>());

    pcl::PCDWriter wA;
    file_name = "/home/michbaum/Projects/floorplanLocalization/data/AligendCloud.pcd";
    wA.write<pcl::PointXYZ> (file_name, *transformed_source, false);

    return gicp.hasConverged();
}

void SimpleLocalizer::publishTransformation()
{
    // TODO implement

    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "world";
    transformStamped.child_frame_id = "floorplan";
    
    Eigen::Quaterniond quaternion(transformationMatrix_.block<3, 3>(0, 0));
    
    transformStamped.transform.translation.x = transformationMatrix_(0, 3);
    transformStamped.transform.translation.y = transformationMatrix_(1, 3);
    transformStamped.transform.translation.z = transformationMatrix_(2, 3);
    transformStamped.transform.rotation.x = quaternion.x();
    transformStamped.transform.rotation.y = quaternion.y();
    transformStamped.transform.rotation.z = quaternion.z();
    transformStamped.transform.rotation.w = quaternion.w();

    broadcaster_.sendTransform(transformStamped);

    // transformationFile_ << transformationMatrix_(0, 3) << " " << transformationMatrix_(1, 3) << " " << transformationMatrix_(2, 3) << " " 
    //                     << quaternion.x() << " " << quaternion.y() << " " << quaternion.z() << " " << quaternion.w() << std::endl;
}

void SimpleLocalizer::localize()
{
  
    // while(ros::ok)
    // {
    //     std::cout << "Localizing..." << std::endl;
    // }
}

void SimpleLocalizer::odomCallback(const nav_msgs::Odometry& msg)
{
    // Here check if odometry changed enough to update the position
    Eigen::Vector3d position(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z);
    Eigen::Quaterniond orientation(msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z);

    

    double position_delta = (position - position_).norm();
    double rotation_delta = (orientation.inverse() * orientation_).vec().norm();

    if (position_delta > d_l_|| rotation_delta > theta_l_)
    {
        std::cout << "Odometry changed enough to update the position" << std::endl;
        std::cout << "Position: " << position << std::endl;
        // Here update the position
        position_ = position;
        orientation_ = orientation;
        // Here generate the source cloud
        //sourceGenerator_.projectPosOnFloor(position_, orientation_);

        targetGenerator_.generateTargetCloud(position_, orientation_);
        sourceGenerator_.generateSourceCloud(depthCloud_, orientation_, msg.header.stamp);

        // GICP needs at least 20 points
        if(sourceGenerator_._sourceCloud.size() >= 20 && targetGenerator_._targetCloud.size() >= 20)
        {
            // Here compute the transformation matrix
            int hasConverged = computeTransformationGICP(sourceGenerator_._sourceCloud, targetGenerator_._targetCloud, transformationMatrix_);
            //std::cout << "Transformation matrix: " << std::endl << transformationMatrix << std::endl;
            //std::cout << "Has converged: " << hasConverged << std::endl;
            ROS_INFO("\n******************** Has converged:  %d ********************\n", hasConverged);

            //calculate transformed position
            Eigen::Vector3d transformed_position = transformationMatrix_.block<3, 3>(0, 0) * position_ + transformationMatrix_.block<3, 1>(0, 3);
            std::cout << "Transformed position: " << transformed_position << std::endl;
            transformationFile_ << transformed_position.x() << " " << transformed_position.y() << " " << transformed_position.z() << std::endl;

            if(hasConverged){
              publishTransformation();
            }
        }
        else
        {
            std::cout << "Not enough points for GICP" << std::endl;

            // (TODO): do something
        }

        targetGenerator_.cleanTargetCloud();
        sourceGenerator_.cleanSourceCloud();
    
    }
}   

void SimpleLocalizer::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    // Here only save the current cloud
    // pcl::fromROSMsg(*msg, depthCloud_);
    
    sourceGenerator_.depthCloud_msg_.push_back(msg);
}


void SimpleLocalizer::syncCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg, const nav_msgs::OdometryConstPtr& odom_msg)
{
    // Here only save the current cloud
    pcl::fromROSMsg(*cloud_msg, depthCloud_);
    odomCallback(*odom_msg);
    ROS_INFO("\n******************** SYNCED********************\n");

}

