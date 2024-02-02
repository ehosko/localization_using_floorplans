#include "../../include/localization/simple_localizer.h"
// #include "../../include/localization/cloudGenerator/source_generator.h"
// #include "../../include/localization/cloudGenerator/target_generator.h"


SimpleLocalizer::SimpleLocalizer()
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
    targetGenerator_ = TargetGenerator(floorplan_path);
    targetGenerator_.initTargetGenerator(nh_);

    position_ = Eigen::Vector3d(0, 0, 0);
    orientation_ = Eigen::Quaterniond(1, 0, 0, 0);

    std::cout << "Floorplan path: " << floorplan_path << std::endl;

    // Set up subscribers
    odomSub_ = nh_.subscribe("/rovioli/odom_T_M_I", 1, &SimpleLocalizer::odomCallback, this);
    cloudSub_ = nh_.subscribe("/isaac/isaac_sensor_model/isaac_sensor_out", 1, &SimpleLocalizer::cloudCallback, this);

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
        transformationFile_ << "x y z qx qy qz qw" << std::endl;
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
    gicp.align(*aligned_source);

    transformationMatrix = gicp.getFinalTransformation().cast<double>();
  

    return gicp.hasConverged();
}

void SimpleLocalizer::publishTransformation()
{
    // TODO implement

    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "floorplan";
    transformStamped.child_frame_id = "odom";
    
    Eigen::Quaterniond quaternion(transformationMatrix_.block<3, 3>(0, 0));
    
    transformStamped.transform.translation.x = transformationMatrix_(0, 3);
    transformStamped.transform.translation.y = transformationMatrix_(1, 3);
    transformStamped.transform.translation.z = transformationMatrix_(2, 3);
    transformStamped.transform.rotation.x = quaternion.x();
    transformStamped.transform.rotation.y = quaternion.y();
    transformStamped.transform.rotation.z = quaternion.z();
    transformStamped.transform.rotation.w = quaternion.w();

    broadcaster.sendTransform(transformStamped);

    transformationFile_ << transformationMatrix_(0, 3) << " " << transformationMatrix_(1, 3) << " " << transformationMatrix_(2, 3) << " " 
                        << quaternion.x() << " " << quaternion.y() << " " << quaternion.z() << " " << quaternion.w() << std::endl;
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
        targetGenerator_.generateTargetCloud(position_, orientation_);
        sourceGenerator_.ProjectOnFloor(depthCloud_, orientation_);

        if(sourceGenerator_._sourceCloud.size() > 0 && targetGenerator_._targetCloud.size() > 0)
        {
            // Here compute the transformation matrix
            int hasConverged = computeTransformationGICP(sourceGenerator_._sourceCloud, targetGenerator_._targetCloud, transformationMatrix_);
            //std::cout << "Transformation matrix: " << std::endl << transformationMatrix << std::endl;
            //std::cout << "Has converged: " << hasConverged << std::endl;
            ROS_INFO("\n******************** Has converged:  %d ********************\n", hasConverged);

            if(hasConverged){
            publishTransformation();
            }
        }
        else
        {
            std::cout << "No source or target cloud" << std::endl;

            // (TODO): do something
        }

        targetGenerator_.cleanTargetCloud();
        sourceGenerator_.cleanSourceCloud();
    
    }
}   

void SimpleLocalizer::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    // Here only save the current cloud
    pcl::fromROSMsg(*msg, depthCloud_);
}


