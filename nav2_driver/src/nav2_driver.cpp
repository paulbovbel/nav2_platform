#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

#include <nav2_driver/nav2remote.h>

#include <boost/interprocess/smart_ptr/shared_ptr.hpp>

namespace nav2_driver{

/**
 * @brief ROS Driver/Wrapper for nav2remote Remote API, implementing standard mobile robot subscribers, publishers, and tf frames as per REP 105.
 */
class Nav2Driver{

public:

    /**
     * @brief Constructor for driver node
     * @param name Name of node
     */
    Nav2Driver(std::string name) :
        nh_(),
        private_nh_("~"),
        robot_prefix_()
    {

        //get robot address and port
        private_nh_.param<std::string>("robot_address", robot_address_, "");
        if(robot_address_.empty()){
            std::string message = "Please provide address for Nav2";
            ROS_ERROR_STREAM(message);
            throw std::runtime_error(message);
        }
        private_nh_.param<int>("robot_port", robot_port_, 5010);

        //get parameter for unique tf names
        std::string robot_name;
        private_nh_.param<std::string>("robot_name", robot_name, "");
        if (!robot_name.empty()) { robot_prefix_ = robot_name + "_"; }

        //get parameter for inverted odometry (for use with robot_pose_ekf)
        private_nh_.param<bool>("invert_odom", invert_odom_, false);

        connect(robot_address_, robot_port_);

        odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 10);
        odom_loop_ = nh_.createTimer(ros::Duration(0.1), boost::bind(&Nav2Driver::publishOdometry, this, invert_odom_, robot_prefix_));
        cmd_sub_ = nh_.subscribe("cmd_vel", 5, &Nav2Driver::setVelocity, this);
    }

protected:

    /**
     * @brief Establishes connection to Nav2 base controller, replacing any existing connection if necessary.
     */
    void connect(std::string robot_address, int robot_port){

        //check if already connected
        if(remote_){
            ROS_INFO("Resetting connection to Nav2 base");
            remote_.reset();

            //save odometry offset for new base connection odometry
            Pose2D offset = base_odom_.getPose();
            base_odom_ = BaseOdometry(offset);
        }

        //attepmt to connect several times
        for(int retry = 0; retry < 5; retry++){
            try{
                //leave address:port validation to Nav2Remote. Must use shared_ptr since constructor can throw expception
                remote_ = boost::shared_ptr<Nav2Remote>(new Nav2Remote(robot_address.c_str(),robot_port));
                ROS_INFO_STREAM("Connected to Nav2 base on " << robot_address <<":"<< robot_port);
                return;
            }catch(std::exception& e){
                ROS_WARN_STREAM(e.what());
                ros::Duration(0.2).sleep();
            }
        }

        std::string message = "Failed to connect to Nav2 base";
        ROS_ERROR_STREAM(message << " on " << robot_address_ <<":"<< robot_port_);
        throw std::runtime_error(message);

    }

    /**
     * @brief Retrieves latest odometry information from the base controller, and publishes appropriate message and transforms
     * @param invert_odom Invert odometry for use with robot_pose_ekf
     * @param robot_prefix Tf prefix to use with odom and base_link frame
     */
    void publishOdometry(bool invert_odom, std::string robot_prefix){

        //get odometry from Nav2, reconnect on error
        Pose2D data;
        while(!remote_ || remote_->estimatePosition(data.x, data.y, data.th) < 0){
            connect(robot_address_, robot_port_);
        }

        //update internal state, and publish required message/tf information
        base_odom_.updateWithAbsolute(data);
        tf_broadcaster_.sendTransform(base_odom_.getTransform(invert_odom,robot_prefix));
        odom_pub_.publish(base_odom_.getMessage(robot_prefix));

    }

    /**
     * @brief Sets velocity goals for base controller
     * @param twist Incoming velocity command from ROS
     */
    void setVelocity(const geometry_msgs::TwistConstPtr& twist){

        //convert from vector velocity to relative angular velocity
        double vs = sqrt(pow(twist->linear.x,2) + pow(twist->linear.y,2));
        double vn = atan2(twist->linear.y,twist->linear.x) * (180 / M_PI);

        //send velocity command to Nav2, reconnect on error
        while(!remote_ || remote_->setRelativeVelocity(vn, vs, twist->angular.z) < 0){
            connect(robot_address_, robot_port_);
        }
    }

private:

    /**
     * @brief Internal structure for storing pose representation in 2D
     */
    struct Pose2D{

        Pose2D() : x(0), y(0), th(0) {}
        Pose2D(double x, double y, double th) : x(x), y(y), th(th) {}
        double x, y, th;
        Pose2D& operator +=(Pose2D const& other) {
            x += other.x;
            y += other.y;
            th += other.th;
            return *this;
        }
        Pose2D& operator -=(Pose2D const& other) {
            x -= other.x;
            y -= other.y;

            //detect rollover during velocity calculation
            if(abs(th - other.th) > M_PI){
                if(other.th>0){
                    th += 2*M_PI;
                }else{
                    th -= 2*M_PI;
                }
            }
            th -= other.th;
            return *this;
        }
        Pose2D& operator /=(double const& other) {
            x /= other;
            y /= other;
            th /= other;
            return *this;
        }
        inline Pose2D& operator +(const Pose2D& other) { return *this += other; }
        inline Pose2D& operator -(const Pose2D& other) { return *this -= other; }
        inline Pose2D& operator /(const double& other) { return *this /= other; }

    };

    /**
     * @brief Internal class for building and representing the base odometry state.
     */
    class BaseOdometry{

    public:

        /**
         * @brief Initialize state to 0,0,0
         */
        BaseOdometry(): last_time_(ros::Time::now()) {}

        /**
         * @brief Initialize state with offset, usually if connection to base was reset
         * @param offset Pose offset to use
         */
        BaseOdometry(Pose2D offset): offset_(offset), last_time_(ros::Time::now()) {}

        void updateWithAbsolute(Pose2D abs){
            updateWithRelative(abs - prev_);
            prev_ = abs;
        }

        void updateWithRelative(Pose2D delta){
            double elapsed = (ros::Time::now() - last_time_).toSec();
            last_time_ = ros::Time::now();

            pose_ += delta;
            vel_ = delta / elapsed;

        }

        /**
         * @brief Build transform from internal odometry state
         * @param invert_odom Invert odometry for use with robot_pose_ekf
         * @param robot_prefix Tf prefix to use with odom and base_link frame
         * @return transform
         */
        geometry_msgs::TransformStamped getTransform(bool invert_odom, std::string robot_prefix){

            //build transform message
            geometry_msgs::TransformStamped transform;
            transform.header.stamp = last_time_;

            //invert odometry if necessary
            tf::Transform temp = tf::Transform(tf::createQuaternionFromYaw(pose_.th + offset_.th), tf::Vector3(pose_.x + offset_.x, pose_.y + offset_.y, 0));
            if(invert_odom){
                temp = temp.inverse();
                transform.header.frame_id = robot_prefix + "base_footprint";
                transform.child_frame_id = robot_prefix + "odom";
            }else{
                transform.header.frame_id = robot_prefix + "odom";
                transform.child_frame_id = robot_prefix + "base_footprint";
            }
            tf::transformTFToMsg(temp, transform.transform);

            return transform;

        }

        /**
         * @brief Build Odometry message from internal odometry state
         * @param robot_prefix Tf prefix to use with odom and base_link frame
         * @return odometry message
         */
        nav_msgs::Odometry getMessage(std::string robot_prefix){

            //build odometry message
            nav_msgs::Odometry message;

            message.header.stamp = last_time_;
            message.header.frame_id = robot_prefix + "odom";
            message.child_frame_id = robot_prefix + "base_link";
            message.pose.pose.position.x = pose_.x + offset_.x;
            message.pose.pose.position.y = pose_.y + offset_.y;
            message.pose.pose.position.z = 0.0;
            message.pose.pose.orientation = tf::createQuaternionMsgFromYaw(pose_.th + offset_.th);
            message.pose.covariance[0] = 10^-3;
            message.pose.covariance[7] = 10^-3;
            message.pose.covariance[14] = 10^6;
            message.pose.covariance[21] = 10^6;
            message.pose.covariance[28] = 10^6;
            message.pose.covariance[35] = 10^3;

            message.twist.twist.linear.x = vel_.x;
            message.twist.twist.linear.y = vel_.y;
            message.twist.twist.angular.z = vel_.th;
            message.twist.covariance[0] = 10^-3;
            message.twist.covariance[7] = 10^-3;
            message.twist.covariance[14] = 10^6;
            message.twist.covariance[21] = 10^6;
            message.twist.covariance[28] = 10^6;
            message.twist.covariance[35] = 10^3;

            return message;

        }

        /**
         * @brief Get current odometry pose
         * @return current odometry pose
         */
        Pose2D getPose(){
            return pose_;
        }

    private:

        Pose2D pose_, vel_, prev_, offset_;
        ros::Time last_time_;

    };

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    tf::TransformBroadcaster tf_broadcaster_;

    boost::shared_ptr<Nav2Remote> remote_;

    ros::Publisher odom_pub_;
    ros::Subscriber cmd_sub_;
    ros::Timer odom_loop_;
    BaseOdometry base_odom_;

    std::string robot_address_, robot_prefix_;
    int robot_port_;
    bool invert_odom_;

};

}

int main(int argc, char **argv) {

    ros::init(argc, argv, "nav2_driver");
    nav2_driver::Nav2Driver nav2_driver(ros::this_node::getName());
    ros::spin();

    return 0;
}



