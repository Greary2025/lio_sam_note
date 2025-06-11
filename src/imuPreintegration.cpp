#include "utility.h"  // 自定义工具头文件，包含常用功能函数

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>

#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>

using gtsam::symbol_shorthand::X; // Pose3 (x,y,z,r,p,y) 位姿符号别名
using gtsam::symbol_shorthand::V; // Vel   (xdot,ydot,zdot) 速度符号别名
using gtsam::symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz) 偏置符号别名

// 里程计变换融合类：用于融合激光雷达和IMU的里程计数据
class TransformFusion : public ParamServer
{
public:
    std::mutex mtx;  // 互斥锁，保证线程安全

    ros::Subscriber subImuOdometry;     // 订阅IMU里程计话题
    ros::Subscriber subLaserOdometry;   // 订阅激光雷达里程计话题

    ros::Publisher pubImuOdometry;      // 发布融合后的IMU里程计
    ros::Publisher pubImuPath;          // 发布IMU路径

    Eigen::Affine3f lidarOdomAffine;        // 激光雷达里程计的仿射变换
    Eigen::Affine3f imuOdomAffineFront;     // 前一时刻IMU里程计仿射变换
    Eigen::Affine3f imuOdomAffineBack;      // 当前时刻IMU里程计仿射变换

    tf::TransformListener tfListener;       // TF监听器，用于获取坐标系变换
    tf::StampedTransform lidar2Baselink;    // 激光雷达到基坐标系的变换

    double lidarOdomTime = -1;              // 激光雷达里程计时间戳
    deque<nav_msgs::Odometry> imuOdomQueue; // IMU里程计消息队列

    // 构造函数：初始化TF变换、订阅器和发布器
    TransformFusion()
    {
        if(lidarFrame != baselinkFrame)  // 若激光雷达与基坐标系不同
        {
            try
            {
                // 等待并获取激光雷达到基坐标系的静态变换
                tfListener.waitForTransform(lidarFrame, baselinkFrame, ros::Time(0), ros::Duration(3.0));
                tfListener.lookupTransform(lidarFrame, baselinkFrame, ros::Time(0), lidar2Baselink);
            }
            catch (tf::TransformException ex)
            {
                ROS_ERROR("%s",ex.what());  // 打印变换获取失败错误
            }
        }

        // 订阅激光雷达里程计话题（来自建图模块）
        subLaserOdometry = nh.subscribe<nav_msgs::Odometry>("lio_sam/mapping/odometry", 5, &TransformFusion::lidarOdometryHandler, this, ros::TransportHints().tcpNoDelay());
        // 订阅IMU增量里程计话题
        subImuOdometry   = nh.subscribe<nav_msgs::Odometry>(odomTopic+"_incremental",   2000, &TransformFusion::imuOdometryHandler,   this, ros::TransportHints().tcpNoDelay());

        // 发布融合后的IMU里程计和路径
        pubImuOdometry   = nh.advertise<nav_msgs::Odometry>(odomTopic, 2000);
        pubImuPath       = nh.advertise<nav_msgs::Path>    ("lio_sam/imu/path", 1);
    }

    // 将里程计消息转换为仿射变换矩阵的函数
    Eigen::Affine3f odom2affine(nav_msgs::Odometry odom)
    {
        double x, y, z, roll, pitch, yaw;
        x = odom.pose.pose.position.x;
        y = odom.pose.pose.position.y;
        z = odom.pose.pose.position.z;
        tf::Quaternion orientation;
        tf::quaternionMsgToTF(odom.pose.pose.orientation, orientation);
        tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);  // 四元数转欧拉角
        return pcl::getTransformation(x, y, z, roll, pitch, yaw);  // 生成仿射变换
    }

    // 激光雷达里程计回调函数：更新激光雷达里程计数据
    void lidarOdometryHandler(const nav_msgs::Odometry::ConstPtr& odomMsg)
    {
        std::lock_guard<std::mutex> lock(mtx);  // 加锁保证线程安全

        lidarOdomAffine = odom2affine(*odomMsg);  // 转换并存储激光雷达里程计仿射变换

        lidarOdomTime = odomMsg->header.stamp.toSec();  // 记录时间戳
    }

    // IMU里程计回调函数：处理IMU数据并融合输出
    void imuOdometryHandler(const nav_msgs::Odometry::ConstPtr& odomMsg)
    {
        // 静态TF广播器：发布map到odom的固定变换（初始无偏移）
        static tf::TransformBroadcaster tfMap2Odom;
        static tf::Transform map_to_odom = tf::Transform(tf::createQuaternionFromRPY(0, 0, 0), tf::Vector3(0, 0, 0));
        tfMap2Odom.sendTransform(tf::StampedTransform(map_to_odom, odomMsg->header.stamp, mapFrame, odometryFrame));

        std::lock_guard<std::mutex> lock(mtx);  // 加锁保证线程安全

        imuOdomQueue.push_back(*odomMsg);  // 将IMU里程计消息加入队列

        // 若激光雷达里程计时间未初始化，直接返回
        if (lidarOdomTime == -1)
            return;
        // 移除队列中时间早于激光雷达里程计的旧数据
        while (!imuOdomQueue.empty())
        {
            if (imuOdomQueue.front().header.stamp.toSec() <= lidarOdomTime)
                imuOdomQueue.pop_front();
            else
                break;
        }
        // 计算IMU里程计的前后时刻仿射变换及增量变换
        Eigen::Affine3f imuOdomAffineFront = odom2affine(imuOdomQueue.front());
        Eigen::Affine3f imuOdomAffineBack = odom2affine(imuOdomQueue.back());
        Eigen::Affine3f imuOdomAffineIncre = imuOdomAffineFront.inverse() * imuOdomAffineBack;
        Eigen::Affine3f imuOdomAffineLast = lidarOdomAffine * imuOdomAffineIncre;  // 融合后的最终变换
        float x, y, z, roll, pitch, yaw;
        pcl::getTranslationAndEulerAngles(imuOdomAffineLast, x, y, z, roll, pitch, yaw);  // 提取位姿参数
        
        // 发布最新的融合里程计消息
        nav_msgs::Odometry laserOdometry = imuOdomQueue.back();
        laserOdometry.pose.pose.position.x = x;
        laserOdometry.pose.pose.position.y = y;
        laserOdometry.pose.pose.position.z = z;
        laserOdometry.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
        pubImuOdometry.publish(laserOdometry);

        // 发布odom到baselink的TF变换
        static tf::TransformBroadcaster tfOdom2BaseLink;
        tf::Transform tCur;
        tf::poseMsgToTF(laserOdometry.pose.pose, tCur);
        if(lidarFrame != baselinkFrame)  // 若激光雷达与基坐标系不同，转换到基坐标系
            tCur = tCur * lidar2Baselink;
        tf::StampedTransform odom_2_baselink = tf::StampedTransform(tCur, odomMsg->header.stamp, odometryFrame, baselinkFrame);
        tfOdom2BaseLink.sendTransform(odom_2_baselink);

        // 发布IMU路径（每隔0.1秒更新）
        static nav_msgs::Path imuPath;
        static double last_path_time = -1;
        double imuTime = imuOdomQueue.back().header.stamp.toSec();
        if (imuTime - last_path_time > 0.1)
        {
            last_path_time = imuTime;
            geometry_msgs::PoseStamped pose_stamped;
            pose_stamped.header.stamp = imuOdomQueue.back().header.stamp;
            pose_stamped.header.frame_id = odometryFrame;
            pose_stamped.pose = laserOdometry.pose.pose;
            imuPath.poses.push_back(pose_stamped);
            // 移除过时的路径点（早于激光雷达时间1秒前）
            while(!imuPath.poses.empty() && imuPath.poses.front().header.stamp.toSec() < lidarOdomTime - 1.0)
                imuPath.poses.erase(imuPath.poses.begin());
            if (pubImuPath.getNumSubscribers() != 0)
            {
                imuPath.header.stamp = imuOdomQueue.back().header.stamp;
                imuPath.header.frame_id = odometryFrame;
                pubImuPath.publish(imuPath);
            }
        }
    }
};

// IMU预积分类：用于IMU数据的预积分和状态优化
class IMUPreintegration : public ParamServer
{
public:

    std::mutex mtx;  // 互斥锁，保证线程安全

    ros::Subscriber subImu;            // 订阅IMU原始数据话题
    ros::Subscriber subOdometry;       // 订阅建图模块的增量里程计话题
    ros::Publisher pubImuOdometry;     // 发布IMU预积分后的增量里程计

    bool systemInitialized = false;    // 系统初始化标志

    // 先验噪声模型（位姿、速度、偏置）
    gtsam::noiseModel::Diagonal::shared_ptr priorPoseNoise;
    gtsam::noiseModel::Diagonal::shared_ptr priorVelNoise;
    gtsam::noiseModel::Diagonal::shared_ptr priorBiasNoise;
    gtsam::noiseModel::Diagonal::shared_ptr correctionNoise;    // 校正噪声（正常情况）
    gtsam::noiseModel::Diagonal::shared_ptr correctionNoise2;   // 校正噪声（退化情况）
    gtsam::Vector noiseModelBetweenBias;  // 偏置间噪声模型

    // IMU预积分器（优化线程和IMU线程各一个）
    gtsam::PreintegratedImuMeasurements *imuIntegratorOpt_;
    gtsam::PreintegratedImuMeasurements *imuIntegratorImu_;

    std::deque<sensor_msgs::Imu> imuQueOpt;  // 优化线程的IMU数据队列
    std::deque<sensor_msgs::Imu> imuQueImu;  // IMU线程的IMU数据队列

    gtsam::Pose3 prevPose_;          // 上一时刻位姿
    gtsam::Vector3 prevVel_;         // 上一时刻速度
    gtsam::NavState prevState_;      // 上一时刻导航状态（位姿+速度）
    gtsam::imuBias::ConstantBias prevBias_;  // 上一时刻IMU偏置（加速度+角速度）

    gtsam::NavState prevStateOdom;   // IMU线程的上一导航状态
    gtsam::imuBias::ConstantBias prevBiasOdom;  // IMU线程的上一偏置

    bool doneFirstOpt = false;       // 首次优化完成标志
    double lastImuT_imu = -1;        // IMU线程的上一IMU时间戳
    double lastImuT_opt = -1;        // 优化线程的上一IMU时间戳

    gtsam::ISAM2 optimizer;          // 增量平滑优化器（用于实时优化）
    gtsam::NonlinearFactorGraph graphFactors;  // 因子图（约束条件）
    gtsam::Values graphValues;       // 因子图中的变量值

    const double delta_t = 0;        // 时间差（用于数据对齐）

    int key = 1;                     // 优化变量的关键帧序号

    // 坐标系变换：IMU到激光雷达（及反向）
    // T_bl: 激光雷达坐标系到IMU坐标系的变换
    gtsam::Pose3 imu2Lidar = gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0), gtsam::Point3(-extTrans.x(), -extTrans.y(), -extTrans.z()));
    // T_lb: IMU坐标系到激光雷达坐标系的变换
    gtsam::Pose3 lidar2Imu = gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0), gtsam::Point3(extTrans.x(), extTrans.y(), extTrans.z()));

    // 构造函数：初始化订阅器、预积分参数、噪声模型
    IMUPreintegration()
    {
        subImu      = nh.subscribe<sensor_msgs::Imu>  (imuTopic,                   2000, &IMUPreintegration::imuHandler,      this, ros::TransportHints().tcpNoDelay());
        subOdometry = nh.subscribe<nav_msgs::Odometry>("lio_sam/mapping/odometry_incremental", 5,    &IMUPreintegration::odometryHandler, this, ros::TransportHints().tcpNoDelay());

        pubImuOdometry = nh.advertise<nav_msgs::Odometry> (odomTopic+"_incremental", 2000);  // 发布增量里程计

        // 初始化预积分参数（重力加速度）
        boost::shared_ptr<gtsam::PreintegrationParams> p = gtsam::PreintegrationParams::MakeSharedU(imuGravity);
        p->accelerometerCovariance  = gtsam::Matrix33::Identity(3,3) * pow(imuAccNoise, 2);  // 加速度计白噪声协方差
        p->gyroscopeCovariance      = gtsam::Matrix33::Identity(3,3) * pow(imuGyrNoise, 2);  // 陀螺仪白噪声协方差
        p->integrationCovariance    = gtsam::Matrix33::Identity(3,3) * pow(1e-4, 2);         // 积分误差协方差
        gtsam::imuBias::ConstantBias prior_imu_bias((gtsam::Vector(6) << 0, 0, 0, 0, 0, 0).finished());  // 初始零偏置假设

        // 初始化先验噪声模型
        priorPoseNoise  = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2).finished()); // 位姿噪声（rad,rad,rad,m,m,m）
        priorVelNoise   = gtsam::noiseModel::Isotropic::Sigma(3, 1e4);  // 速度噪声（m/s）
        priorBiasNoise  = gtsam::noiseModel::Isotropic::Sigma(6, 1e-3);  // 偏置噪声
        correctionNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 0.05, 0.05, 0.05, 0.1, 0.1, 0.1).finished());  // 正常校正噪声
        correctionNoise2 = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1, 1, 1, 1, 1, 1).finished());  // 退化校正噪声
        noiseModelBetweenBias = (gtsam::Vector(6) << imuAccBiasN, imuAccBiasN, imuAccBiasN, imuGyrBiasN, imuGyrBiasN, imuGyrBiasN).finished();
        
        imuIntegratorImu_ = new gtsam::PreintegratedImuMeasurements(p, prior_imu_bias); // setting up the IMU integration for IMU message thread
        imuIntegratorOpt_ = new gtsam::PreintegratedImuMeasurements(p, prior_imu_bias); // setting up the IMU integration for optimization        
    }

    // 重置优化器（清空因子图和优化值，重新初始化）
    void resetOptimization()
    {
        gtsam::ISAM2Params optParameters;  // ISAM2优化器参数配置
        optParameters.relinearizeThreshold = 0.1;  // 重新线性化阈值（误差超过此值时重新线性化）
        optParameters.relinearizeSkip = 1;         // 跳过重新线性化的步数（每1步检查一次）
        optimizer = gtsam::ISAM2(optParameters);   // 初始化ISAM2优化器

        gtsam::NonlinearFactorGraph newGraphFactors;  // 新建空因子图
        graphFactors = newGraphFactors;               // 重置当前因子图

        gtsam::Values NewGraphValues;  // 新建空变量值集合
        graphValues = NewGraphValues;  // 重置当前变量值
    }

    // 重置系统参数（用于异常恢复或重新初始化）
    void resetParams()
    {
        lastImuT_imu = -1;       // 重置IMU线程的上一时间戳
        doneFirstOpt = false;    // 重置首次优化完成标志
        systemInitialized = false;  // 重置系统初始化标志
    }

    // 建图模块增量里程计回调函数（处理激光雷达提供的校正数据）
    void odometryHandler(const nav_msgs::Odometry::ConstPtr& odomMsg)
    {
        std::lock_guard<std::mutex> lock(mtx);  // 加锁保证线程安全

        double currentCorrectionTime = ROS_TIME(odomMsg);  // 获取当前校正时间戳

        // 确保有IMU数据可供积分（无数据则直接返回）
        if (imuQueOpt.empty())
            return;

        // 提取激光雷达里程计的位置和四元数
        float p_x = odomMsg->pose.pose.position.x;
        float p_y = odomMsg->pose.pose.position.y;
        float p_z = odomMsg->pose.pose.position.z;
        float r_x = odomMsg->pose.pose.orientation.x;
        float r_y = odomMsg->pose.pose.orientation.y;
        float r_z = odomMsg->pose.pose.orientation.z;
        float r_w = odomMsg->pose.pose.orientation.w;
        // 判断是否为退化情况（通过协方差矩阵标志位）
        bool degenerate = (int)odomMsg->pose.covariance[0] == 1 ? true : false;
        // 将激光雷达位姿转换为gtsam的Pose3（考虑IMU与激光雷达外参）
        gtsam::Pose3 lidarPose = gtsam::Pose3(gtsam::Rot3::Quaternion(r_w, r_x, r_y, r_z), gtsam::Point3(p_x, p_y, p_z));


        // 0. 系统初始化（首次接收到里程计数据时执行）
        if (systemInitialized == false)
        {
            resetOptimization();  // 重置优化器

            // 移除早于当前校正时间的旧IMU数据（对齐时间戳）
            while (!imuQueOpt.empty())
            {
                if (ROS_TIME(&imuQueOpt.front()) < currentCorrectionTime - delta_t)
                {
                    lastImuT_opt = ROS_TIME(&imuQueOpt.front());  // 记录最后一个旧数据时间戳
                    imuQueOpt.pop_front();                        // 弹出旧数据
                }
                else
                    break;
            }
            // 初始位姿：激光雷达位姿转换为IMU坐标系位姿（通过外参变换）
            prevPose_ = lidarPose.compose(lidar2Imu);
            // 添加位姿先验因子（约束初始位姿）
            gtsam::PriorFactor<gtsam::Pose3> priorPose(X(0), prevPose_, priorPoseNoise);
            graphFactors.add(priorPose);
            // 初始速度设为0（静止假设）
            prevVel_ = gtsam::Vector3(0, 0, 0);
            // 添加速度先验因子
            gtsam::PriorFactor<gtsam::Vector3> priorVel(V(0), prevVel_, priorVelNoise);
            graphFactors.add(priorVel);
            // 初始偏置设为0（无偏置假设）
            prevBias_ = gtsam::imuBias::ConstantBias();
            // 添加偏置先验因子
            gtsam::PriorFactor<gtsam::imuBias::ConstantBias> priorBias(B(0), prevBias_, priorBiasNoise);
            graphFactors.add(priorBias);
            // 向因子图插入初始变量值
            graphValues.insert(X(0), prevPose_);
            graphValues.insert(V(0), prevVel_);
            graphValues.insert(B(0), prevBias_);
            // 首次优化（初始化优化器）
            optimizer.update(graphFactors, graphValues);
            graphFactors.resize(0);  // 清空因子图（后续增量更新）
            graphValues.clear();     // 清空变量值

            // 重置IMU预积分器（使用初始偏置）
            imuIntegratorImu_->resetIntegrationAndSetBias(prevBias_);
            imuIntegratorOpt_->resetIntegrationAndSetBias(prevBias_);
            
            key = 1;  // 关键帧序号初始化为1
            systemInitialized = true;  // 标记系统初始化完成
            return;
        }


        // 定期重置因子图（防止优化器计算量过大）
        if (key == 100)
        {
            // 获取上一关键帧的协方差（用于重置后的先验噪声）
            gtsam::noiseModel::Gaussian::shared_ptr updatedPoseNoise = gtsam::noiseModel::Gaussian::Covariance(optimizer.marginalCovariance(X(key-1)));
            gtsam::noiseModel::Gaussian::shared_ptr updatedVelNoise  = gtsam::noiseModel::Gaussian::Covariance(optimizer.marginalCovariance(V(key-1)));
            gtsam::noiseModel::Gaussian::shared_ptr updatedBiasNoise = gtsam::noiseModel::Gaussian::Covariance(optimizer.marginalCovariance(B(key-1)));
            // 重置优化器
            resetOptimization();
            // 添加重置后的先验因子（使用更新后的噪声模型）
            gtsam::PriorFactor<gtsam::Pose3> priorPose(X(0), prevPose_, updatedPoseNoise);
            graphFactors.add(priorPose);
            gtsam::PriorFactor<gtsam::Vector3> priorVel(V(0), prevVel_, updatedVelNoise);
            graphFactors.add(priorVel);
            gtsam::PriorFactor<gtsam::imuBias::ConstantBias> priorBias(B(0), prevBias_, updatedBiasNoise);
            graphFactors.add(priorBias);
            // 插入重置后的初始变量值
            graphValues.insert(X(0), prevPose_);
            graphValues.insert(V(0), prevVel_);
            graphValues.insert(B(0), prevBias_);
            // 优化一次（初始化重置后的优化器）
            optimizer.update(graphFactors, graphValues);
            graphFactors.resize(0);
            graphValues.clear();

            key = 1;  // 关键帧序号重置为1
        }


        // 1. 积分IMU数据并优化（处理当前校正时间前的IMU数据）
        while (!imuQueOpt.empty())
        {
            sensor_msgs::Imu *thisImu = &imuQueOpt.front();  // 取队列首元素（最旧的IMU数据）
            double imuTime = ROS_TIME(thisImu);              // 获取IMU数据时间戳
            // 仅处理早于当前校正时间的IMU数据（时间对齐）
            if (imuTime < currentCorrectionTime - delta_t)
            {
                // 计算时间间隔（首次取1/500秒，否则取相邻数据时间差）
                double dt = (lastImuT_opt < 0) ? (1.0 / 500.0) : (imuTime - lastImuT_opt);
                // 预积分IMU测量值（加速度和角速度）
                imuIntegratorOpt_->integrateMeasurement(
                        gtsam::Vector3(thisImu->linear_acceleration.x, thisImu->linear_acceleration.y, thisImu->linear_acceleration.z),
                        gtsam::Vector3(thisImu->angular_velocity.x,    thisImu->angular_velocity.y,    thisImu->angular_velocity.z), dt);
                
                lastImuT_opt = imuTime;  // 更新上一IMU时间戳
                imuQueOpt.pop_front();   // 弹出已处理的IMU数据
            }
            else
                break;  // 剩余数据时间戳较新，暂不处理
        }
        // 向因子图添加IMU预积分因子（约束相邻关键帧的位姿、速度和偏置）
        const gtsam::PreintegratedImuMeasurements& preint_imu = dynamic_cast<const gtsam::PreintegratedImuMeasurements&>(*imuIntegratorOpt_);
        gtsam::ImuFactor imu_factor(X(key - 1), V(key - 1), X(key), V(key), B(key - 1), preint_imu);
        graphFactors.add(imu_factor);
        // 添加偏置间因子（约束相邻关键帧的IMU偏置变化）
        graphFactors.add(gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(B(key - 1), B(key), gtsam::imuBias::ConstantBias(),
                         gtsam::noiseModel::Diagonal::Sigmas(sqrt(imuIntegratorOpt_->deltaTij()) * noiseModelBetweenBias)));
        // 添加位姿因子（激光雷达校正位姿作为观测约束）
        gtsam::Pose3 curPose = lidarPose.compose(lidar2Imu);  // 转换为IMU坐标系位姿
        gtsam::PriorFactor<gtsam::Pose3> pose_factor(X(key), curPose, degenerate ? correctionNoise2 : correctionNoise);  // 退化情况使用更宽松的噪声
        graphFactors.add(pose_factor);
        // 插入预测值（基于预积分结果预测当前关键帧的状态）
        gtsam::NavState propState_ = imuIntegratorOpt_->predict(prevState_, prevBias_);
        graphValues.insert(X(key), propState_.pose());  // 预测位姿
        graphValues.insert(V(key), propState_.v());      // 预测速度
        graphValues.insert(B(key), prevBias_);           // 预测偏置（假设偏置不变）
        // 执行优化（更新因子图和变量值）
        optimizer.update(graphFactors, graphValues);
        optimizer.update();  // 触发增量优化
        graphFactors.resize(0);  // 清空因子图（为下次优化准备）
        graphValues.clear();     // 清空变量值
        // 获取优化结果（更新当前状态）
        gtsam::Values result = optimizer.calculateEstimate();
        prevPose_  = result.at<gtsam::Pose3>(X(key));       // 更新位姿
        prevVel_   = result.at<gtsam::Vector3>(V(key));      // 更新速度
        prevState_ = gtsam::NavState(prevPose_, prevVel_);   // 更新导航状态
        prevBias_  = result.at<gtsam::imuBias::ConstantBias>(B(key));  // 更新偏置
        // Reset the optimization preintegration object.
        imuIntegratorOpt_->resetIntegrationAndSetBias(prevBias_);
        // check optimization
        if (failureDetection(prevVel_, prevBias_))
        {
            resetParams();
            return;
        }


        // 2. after optiization, re-propagate imu odometry preintegration
        prevStateOdom = prevState_;
        prevBiasOdom  = prevBias_;
        // first pop imu message older than current correction data
        double lastImuQT = -1;
        while (!imuQueImu.empty() && ROS_TIME(&imuQueImu.front()) < currentCorrectionTime - delta_t)
        {
            lastImuQT = ROS_TIME(&imuQueImu.front());
            imuQueImu.pop_front();
        }
        // repropogate
        if (!imuQueImu.empty())
        {
            // reset bias use the newly optimized bias
            imuIntegratorImu_->resetIntegrationAndSetBias(prevBiasOdom);
            // integrate imu message from the beginning of this optimization
            for (int i = 0; i < (int)imuQueImu.size(); ++i)
            {
                sensor_msgs::Imu *thisImu = &imuQueImu[i];
                double imuTime = ROS_TIME(thisImu);
                double dt = (lastImuQT < 0) ? (1.0 / 500.0) :(imuTime - lastImuQT);

                imuIntegratorImu_->integrateMeasurement(gtsam::Vector3(thisImu->linear_acceleration.x, thisImu->linear_acceleration.y, thisImu->linear_acceleration.z),
                                                        gtsam::Vector3(thisImu->angular_velocity.x,    thisImu->angular_velocity.y,    thisImu->angular_velocity.z), dt);
                lastImuQT = imuTime;
            }
        }

        ++key;
        doneFirstOpt = true;
    }

    bool failureDetection(const gtsam::Vector3& velCur, const gtsam::imuBias::ConstantBias& biasCur)
    {
        Eigen::Vector3f vel(velCur.x(), velCur.y(), velCur.z());
        // if (vel.norm() > 30)
        // 适应低频IMU
        if (vel.norm() > 50)
        {
            ROS_WARN("Large velocity, reset IMU-preintegration!");
            return true;
        }

        Eigen::Vector3f ba(biasCur.accelerometer().x(), biasCur.accelerometer().y(), biasCur.accelerometer().z());
        Eigen::Vector3f bg(biasCur.gyroscope().x(), biasCur.gyroscope().y(), biasCur.gyroscope().z());
        if (ba.norm() > 1.0 || bg.norm() > 1.0)
        // if (ba.norm() > 3.0 || bg.norm() > 3.0)
        {
            ROS_WARN("Large bias, reset IMU-preintegration!");
            return true;
        }

        return false;
    }

    void imuHandler(const sensor_msgs::Imu::ConstPtr& imu_raw)
    {
        std::lock_guard<std::mutex> lock(mtx);

        sensor_msgs::Imu thisImu = imuConverter(*imu_raw);

        imuQueOpt.push_back(thisImu);
        imuQueImu.push_back(thisImu);

        if (doneFirstOpt == false)
            return;

        double imuTime = ROS_TIME(&thisImu);
        double dt = (lastImuT_imu < 0) ? (1.0 / 500.0) : (imuTime - lastImuT_imu);
        lastImuT_imu = imuTime;

        // integrate this single imu message
        imuIntegratorImu_->integrateMeasurement(gtsam::Vector3(thisImu.linear_acceleration.x, thisImu.linear_acceleration.y, thisImu.linear_acceleration.z),
                                                gtsam::Vector3(thisImu.angular_velocity.x,    thisImu.angular_velocity.y,    thisImu.angular_velocity.z), dt);

        // predict odometry
        gtsam::NavState currentState = imuIntegratorImu_->predict(prevStateOdom, prevBiasOdom);

        // publish odometry
        nav_msgs::Odometry odometry;
        odometry.header.stamp = thisImu.header.stamp;
        odometry.header.frame_id = odometryFrame;
        odometry.child_frame_id = "odom_imu";

        // transform imu pose to ldiar
        gtsam::Pose3 imuPose = gtsam::Pose3(currentState.quaternion(), currentState.position());
        gtsam::Pose3 lidarPose = imuPose.compose(imu2Lidar);

        odometry.pose.pose.position.x = lidarPose.translation().x();
        odometry.pose.pose.position.y = lidarPose.translation().y();
        odometry.pose.pose.position.z = lidarPose.translation().z();
        odometry.pose.pose.orientation.x = lidarPose.rotation().toQuaternion().x();
        odometry.pose.pose.orientation.y = lidarPose.rotation().toQuaternion().y();
        odometry.pose.pose.orientation.z = lidarPose.rotation().toQuaternion().z();
        odometry.pose.pose.orientation.w = lidarPose.rotation().toQuaternion().w();
        
        odometry.twist.twist.linear.x = currentState.velocity().x();
        odometry.twist.twist.linear.y = currentState.velocity().y();
        odometry.twist.twist.linear.z = currentState.velocity().z();
        odometry.twist.twist.angular.x = thisImu.angular_velocity.x + prevBiasOdom.gyroscope().x();
        odometry.twist.twist.angular.y = thisImu.angular_velocity.y + prevBiasOdom.gyroscope().y();
        odometry.twist.twist.angular.z = thisImu.angular_velocity.z + prevBiasOdom.gyroscope().z();
        pubImuOdometry.publish(odometry);
    }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "roboat_loam");
    
    IMUPreintegration ImuP;

    TransformFusion TF;

    ROS_INFO("\033[1;32m----> IMU Preintegration Started.\033[0m");
    
    ros::MultiThreadedSpinner spinner(4);
    spinner.spin();
    
    return 0;
}
