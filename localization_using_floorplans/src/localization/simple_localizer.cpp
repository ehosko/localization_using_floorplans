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

    std::cout << "Setting up from param..." << std::endl;

    nh_.getParam("floorplan_node/floorplan_path", floorplan_path);
    
    nh_.getParam("floorplan_node/d_l", d_l_);
    nh_.getParam("floorplan_node/theta_l", theta_l_);

    nh_.getParam("floorplan_node/transformation_thresh", transformationThreshold_);

    sourceGenerator_ = SourceGenerator();
    sourceGenerator_.initSourceGenerator(nh_);
    targetGenerator_ = TargetGenerator(floorplan_path);
    targetGenerator_.initTargetGenerator(nh_);

    // Known inital position and orientation
    position_ = Eigen::Vector3d(0, 0, 0);
    orientation_ = Eigen::Quaterniond(1, 0, 0, 0);
    floorplan_position_ = Eigen::Vector3d(0, 0, 0);
    transformationMatrix_ = Eigen::Matrix4d::Identity();


    // Create subscribers for the topics
    message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub(nh_, "pointcloud", 1);
    message_filters::Subscriber<nav_msgs::Odometry> odom_sub(nh_, "odometry", 1);

    // Synchronize the messages using ApproximateTime with a one-second time tolerance
    //typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry> SyncPolicy;
    typedef message_filters::sync_policies::ExactTime<sensor_msgs::PointCloud2, nav_msgs::Odometry> SyncPolicy;
    message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(10), cloud_sub, odom_sub);

    // Register the callback to be called when synchronized messages are received
    sync.registerCallback(boost::bind(&SimpleLocalizer::syncCallback,boost::ref(*this), _1, _2));

    ros::spin();

}

int SimpleLocalizer::computeTransformationGICP(pcl::PointCloud<pcl::PointXYZ> sourceCloud, 
                                                pcl::PointCloud<pcl::PointXYZ> targetCloud,
                                                Eigen::Matrix4d& transformationMatrix)
{
    // Compute transformation matrix using GICP
    pcl::PointCloud<pcl::PointXYZ>::Ptr sourceCloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
    *sourceCloudPtr = sourceCloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr targetCloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
    *targetCloudPtr = targetCloud;

    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
    gicp.setInputSource(sourceCloudPtr);
    gicp.setInputTarget(targetCloudPtr);

    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_source =
    boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

    Eigen::Matrix4f initial_guess = Eigen::Matrix4f::Identity();

    gicp.align(*aligned_source, initial_guess);
    // gicp.align(*aligned_source);

    transformationMatrix = gicp.getFinalTransformation().cast<double>();

    // Debugging purpose - to visualize point clouds
    // std::string folder = "/home/michbaum/Projects/optag_EH/data/floorplan/";

    // pcl::PCDWriter wS;
    // std::string file_name = folder + "SourceCloud.pcd";
    // wS.write<pcl::PointXYZ> (file_name, sourceCloud, false);

    // pcl::PCDWriter wT;
    // file_name = folder + "TargetCloud.pcd";
    // wT.write<pcl::PointXYZ> (file_name, targetCloud, false);


    // pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_source(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::transformPointCloud(sourceCloud, *transformed_source, transformationMatrix.cast<float>());

    // pcl::PCDWriter wA;
    // file_name = folder + "AligendCloud.pcd";
    // wA.write<pcl::PointXYZ> (file_name, *transformed_source, false);

    return gicp.hasConverged();
}

void SimpleLocalizer::publishTransformation(Eigen::Vector3d position, Eigen::Quaterniond orientation)
{
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "world";
    transformStamped.child_frame_id = "floorplan";
    
    
    transformStamped.transform.translation.x = position.x();
    transformStamped.transform.translation.y = position.y();
    transformStamped.transform.translation.z = position.z();
    transformStamped.transform.rotation.x = orientation.x();
    transformStamped.transform.rotation.y = orientation.y();
    transformStamped.transform.rotation.z = orientation.z();
    transformStamped.transform.rotation.w = orientation.w();

    broadcaster_.sendTransform(transformStamped);
}


void SimpleLocalizer::odomCallback(const nav_msgs::Odometry& msg)
{
    // Here check if odometry changed enough to update the position
    Eigen::Vector3d position(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z);
    Eigen::Quaterniond orientation(msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z);

    double position_delta = (position - position_).norm();
    double rotation_delta = (orientation.inverse() * orientation_).vec().norm();

    floorplan_position_ += (position - position_);
    floorplan_position_.z() = 0;  

    if (position_delta > d_l_|| rotation_delta > theta_l_)
    {
        // Odometry has changed enough
        // Here update the position
        position_ = position;
        orientation_ = orientation;

        //targetGenerator_.generateTargetCloud(position_, orientation_);
        targetGenerator_.generateTargetCloud(floorplan_position_, orientation_);
        sourceGenerator_.generateSourceCloud(depthCloud_, orientation_, msg.header.stamp);

        // GICP needs at least 20 points
        if(sourceGenerator_._sourceCloud.size() >= 20 && targetGenerator_._targetCloud.size() >= 20)
        {
            // Here compute the transformation matrix
            int hasConverged = computeTransformationGICP(sourceGenerator_._sourceCloud, targetGenerator_._targetCloud, transformationMatrix_);
            ROS_DEBUG("Has converged:  %d ", hasConverged);

            //calculate transformed position
            Eigen::Quaterniond q(transformationMatrix_.block<3, 3>(0, 0));
            // Eigen::Vector3d transformed_position = q * position_ + transformationMatrix_.block<3, 1>(0, 3);
            Eigen::Vector3d transformed_position = q * floorplan_position_ + transformationMatrix_.block<3, 1>(0, 3);
            Eigen::Quaterniond transformed_orientation = q * orientation_;

            // Eigen::Matrix3d rotationMatrix = orientation_.normalized().toRotationMatrix();
            // Eigen::Vector3d normalVector(0.0, 0.0, 1.0); // Assuming floor is horizontal
            // normalVector = rotationMatrix * normalVector;

            // // Project the point onto the floor
            // double distance = -normalVector.dot(position_);
            // Eigen::Vector3d projectedVector = position_ - distance * normalVector;

            Eigen::Vector3d projectedVector = position_;
            projectedVector.z() = 0;

            if(hasConverged){
              publishTransformation(transformed_position, transformed_orientation);

              // If the transformed position is close enough to the projected position, use the transformed position
              if((transformed_position - projectedVector).norm() < transformationThreshold_){

                floorplan_position_ = transformed_position;
              }
              else{

                floorplan_position_ = projectedVector;
              }
            }
        }
        else
        {
            ROS_DEBUG("Not enough points for GICP");
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
}

