#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <fstream>
#include <iomanip>
#include <string>

class TrajectorySaver {
public:
    TrajectorySaver(ros::NodeHandle& nh) : nh_(nh), 
                                          mapping_start_time_(-1.0), 
                                          imu_start_time_(-1.0) {
        // 初始化参数（从launch文件或命令行获取）
        nh_.param<std::string>("output_dir", output_dir_, "~/liosam_trajectory");
        nh_.param<std::string>("mapping_path_topic", mapping_topic_, "/lio_sam/mapping/path");
        nh_.param<std::string>("imu_path_topic", imu_topic_, "/lio_sam/imu/path");

        // 创建输出目录（若不存在）
        createDirectory(output_dir_);

        // 初始化文件流（覆盖模式，默认截断原有内容）
        mapping_file_.open(output_dir_ + "/mapping_trajectory.txt");  // 覆盖模式
        imu_file_.open(output_dir_ + "/imu_trajectory.txt");          // 覆盖模式

        // 检查文件是否成功打开
        if (!mapping_file_.is_open() || !imu_file_.is_open()) {
            ROS_ERROR("Failed to open trajectory files!");
            ros::shutdown();
            return;
        }

        // 订阅路径话题
        mapping_sub_ = nh_.subscribe<nav_msgs::Path>(
            mapping_topic_, 10, &TrajectorySaver::mappingPathCallback, this);
        imu_sub_ = nh_.subscribe<nav_msgs::Path>(
            imu_topic_, 10, &TrajectorySaver::imuPathCallback, this);

        ROS_INFO("Trajectory saver node started.");
        ROS_INFO("Output directory: %s", output_dir_.c_str());
        ROS_INFO("Subscribing to: %s (mapping) and %s (imu)", 
                 mapping_topic_.c_str(), imu_topic_.c_str());
    }

    ~TrajectorySaver() {
        // 关闭文件流
        if (mapping_file_.is_open()) mapping_file_.close();
        if (imu_file_.is_open()) imu_file_.close();
    }

private:
    // 回调函数：处理映射路径（带增量时间）
    void mappingPathCallback(const nav_msgs::Path::ConstPtr& msg) {
        // 首次接收消息时，记录第一个时间戳作为起始时间
        if (mapping_start_time_ < 0.0 && !msg->poses.empty()) {
            mapping_start_time_ = msg->poses[0].header.stamp.toSec();
            ROS_INFO("Mapping path first timestamp: %.9f, start_time set to %.9f",
                     msg->poses[0].header.stamp.toSec(), mapping_start_time_);
        }
        // 写入轨迹（带增量时间）
        writeTrajectoryToFile(msg, mapping_file_, mapping_start_time_);
    }

    // 回调函数：处理IMU路径（带增量时间）
    void imuPathCallback(const nav_msgs::Path::ConstPtr& msg) {
        // 首次接收消息时，记录第一个时间戳作为起始时间
        if (imu_start_time_ < 0.0 && !msg->poses.empty()) {
            imu_start_time_ = msg->poses[0].header.stamp.toSec();
            ROS_INFO("IMU path first timestamp: %.9f, start_time set to %.9f",
                     msg->poses[0].header.stamp.toSec(), imu_start_time_);
        }
        // 写入轨迹（带增量时间）
        writeTrajectoryToFile(msg, imu_file_, imu_start_time_);
    }

    // 通用轨迹写入函数（TUM格式，增量时间）
    void writeTrajectoryToFile(const nav_msgs::Path::ConstPtr& msg, 
                              std::ofstream& file, 
                              double start_time) {
        // 检查起始时间是否初始化
        if (start_time < 0.0) {
            ROS_WARN("Start time not initialized yet, skipping trajectory writing.");
            return;
        }

        for (const auto& pose : msg->poses) {
            // 计算增量时间（当前时间戳 - 起始时间）
            double time = pose.header.stamp.toSec() - start_time;
            
            // 提取坐标和四元数
            double x = pose.pose.position.x;
            double y = pose.pose.position.y;
            double z = pose.pose.position.z;
            double qx = pose.pose.orientation.x;
            double qy = pose.pose.orientation.y;
            double qz = pose.pose.orientation.z;
            double qw = pose.pose.orientation.w;

            // 按TUM格式写入文件（增量时间 x y z qx qy qz qw）
            file << std::fixed << std::setprecision(9)
                 << time << " " 
              << x << " "<< y<< " "<< z << " " 
                 << qx << " " << qy << " " << qz << " " << qw << std::endl;
        }
    }

    // 创建输出目录（若不存在）
    void createDirectory(const std::string& dir) {
        std::string cmd = "mkdir -p " + dir;
        if (system(cmd.c_str()) != 0) {
            ROS_ERROR("Failed to create directory: %s", dir.c_str());
            ros::shutdown();
        }
    }

    // ROS节点句柄
    ros::NodeHandle nh_;

    // 参数
    std::string output_dir_;       // 输出目录
    std::string mapping_topic_;    // 映射路径话题
    std::string imu_topic_;        // IMU路径话题

    // 起始时间（首次接收消息的时间戳）
    double mapping_start_time_;    // 映射路径起始时间
    double imu_start_time_;        // IMU路径起始时间

    // 文件流
    std::ofstream mapping_file_;   // 映射轨迹文件（覆盖模式）
    std::ofstream imu_file_;       // IMU轨迹文件（覆盖模式）

    // 订阅器
    ros::Subscriber mapping_sub_;  // 映射路径订阅器
    ros::Subscriber imu_sub_;      // IMU路径订阅器
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "path_trajectory_saver");
    ros::NodeHandle nh("~");  // 使用私有命名空间读取参数
    TrajectorySaver saver(nh);
    ros::spin();
    return 0;
}