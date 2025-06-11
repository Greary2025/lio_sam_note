#include "utility.h"
#include "lio_sam/cloud_info.h"

struct VelodynePointXYZIRT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    uint16_t ring;
    float time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT (VelodynePointXYZIRT,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (uint16_t, ring, ring) (float, time, time)
)

struct OusterPointXYZIRT {
    PCL_ADD_POINT4D;
    float intensity;
    uint32_t t;
    uint16_t reflectivity;
    uint8_t ring;
    uint16_t noise;
    uint32_t range;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(OusterPointXYZIRT,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (uint32_t, t, t) (uint16_t, reflectivity, reflectivity)
    (uint8_t, ring, ring) (uint16_t, noise, noise) (uint32_t, range, range)
)

// Use the Velodyne point format as a common representation
using PointXYZIRT = VelodynePointXYZIRT;

const int queueLength = 2000;

// 这个语法表示 ImageProjection 继承了 ParamServer 类
// 继承关系意味着 ImageProjection 类可以访问 ParamServer 中的所有public 和 protected 成员（包括函数和变量），并且能够复用 ParamServer 类中的功能。
class ImageProjection : public ParamServer
{
private:

    std::mutex imuLock;
    std::mutex odoLock;

    ros::Subscriber subLaserCloud;
    ros::Publisher  pubLaserCloud;
    
    ros::Publisher pubExtractedCloud;
    ros::Publisher pubLaserCloudInfo;

    ros::Subscriber subImu;
    std::deque<sensor_msgs::Imu> imuQueue;

    ros::Subscriber subOdom;
    std::deque<nav_msgs::Odometry> odomQueue;

    std::deque<sensor_msgs::PointCloud2> cloudQueue;
    sensor_msgs::PointCloud2 currentCloudMsg;

    double *imuTime = new double[queueLength];
    double *imuRotX = new double[queueLength];
    double *imuRotY = new double[queueLength];
    double *imuRotZ = new double[queueLength];

    int imuPointerCur;
    bool firstPointFlag;
    Eigen::Affine3f transStartInverse;

    pcl::PointCloud<PointXYZIRT>::Ptr laserCloudIn;
    pcl::PointCloud<OusterPointXYZIRT>::Ptr tmpOusterCloudIn;
    pcl::PointCloud<PointType>::Ptr   fullCloud;
    pcl::PointCloud<PointType>::Ptr   extractedCloud;

    int deskewFlag;
    cv::Mat rangeMat;

    bool odomDeskewFlag;
    float odomIncreX;
    float odomIncreY;
    float odomIncreZ;

    // 通过cloud_info.msg构建cloud_info.h包含的cloud_info_结构体
    lio_sam::cloud_info cloudInfo;
    double timeScanCur;
    double timeScanEnd;
    std_msgs::Header cloudHeader;

    vector<int> columnIdnCountVec;


public:
    ImageProjection():
    // 点云校正;初始值设为 0，表示还未检查过点云数据中的时间戳字段。
    // 是否需要对图像或点云数据进行去畸变处理（deskew）。
    deskewFlag(0)
    {
        // imu
        // 这里设置为 2000，意味着最多存储 2000 条 IMU 消息。
        // this 指针是 ImageProjection 类的实例指针，表示回调函数 imuHandler 属于该类的成员函数，ROS 会在回调时自动传递该类的实例指针。
        // ros::TransportHints() 是用来设置传输参数的对象。tcpNoDelay() 表示启用 TCP 的“无延迟”模式（Nagle 算法禁用），这可以减少网络延迟，特别是对于实时系统中的 IMU 数据传输。
        subImu        = nh.subscribe<sensor_msgs::Imu>(imuTopic, 2000, &ImageProjection::imuHandler, this, ros::TransportHints().tcpNoDelay());
        // std::cout << "subImu:" << imuTopic << std::endl;
        // 这是一个字符串，表示订阅的里程计数据话题。odomTopic 是一个变量（可能是 odomTopic = "odom"），然后 _incremental 被附加到话题名上，形成最终的订阅话题名。
        // 例如，如果 odomTopic 是 "odom"，最终订阅的话题将是 "odom_incremental"。
        // 这种命名方式可能是为了区分增量里程计数据与其他类型的里程计数据。
        // 这是回调函数指针，当订阅到新的 Odometry 消息时，odometryHandler 函数将被调用来处理这些数据。
        // odometryHandler 是 ImageProjection 类的成员函数，它负责处理从 odomTopic+"_incremental" 话题接收到的增量里程计数据。
        // subOdom       = nh.subscribe<nav_msgs::Odometry>(odomTopic+"_incremental", 2000, &ImageProjection::odometryHandler, this, ros::TransportHints().tcpNoDelay());
        // 设置为去除IMU紧耦合的回环
        subOdom       = nh.subscribe<nav_msgs::Odometry>("lio_sam/mapping/odometry_incremental", 5,    &ImageProjection::odometryHandler, this, ros::TransportHints().tcpNoDelay());
        // nh.subscribe<sensor_msgs::PointCloud2>：使用 nh（ros::NodeHandle 对象）创建一个订阅者，
        // 订阅的消息类型是 sensor_msgs::PointCloud2，这是 ROS 中用于表示点云数据的标准消息类型。
        // pointCloudTopic：这是订阅的主题名称，类型为 std::string，需要在前面定义。比如 pointCloudTopic 可以是 "/laser_cloud" 或其他包含点云数据的主题。
        // 5：消息队列大小。在消息接收频率较高的情况下，此参数决定缓存多少条未处理的消息。
        // 这里设置为 5 表示缓存 5 条未处理的点云消息，如果处理速度低于接收速度，旧消息会被丢弃以腾出空间。
        // &ImageProjection::cloudHandler：这是处理接收到的消息的回调函数。在接收到点云消息时，ROS 会调用该函数。
        // 这里的 cloudHandler 是 ImageProjection 类的一个成员函数，负责处理接收到的点云数据。
    // this：表示回调函数的对象指针。因为 cloudHandler 是 ImageProjection 类的成员函数，所以需要传入当前对象 this。
        // this传入的应该是 subLaserCloud 的实例指针,传给laserCloudMsg,laserCloudMsg相当于订阅了一部分的pointCloudTopic数据
        // ros::TransportHints().tcpNoDelay()：这是传输设置。
        // tcpNoDelay() 用于禁用 TCP 的 Nagle 算法，确保消息低延迟地传输，适用于实时性要求较高的点云数据传输，减少延迟。
        subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>(pointCloudTopic, 5, &ImageProjection::cloudHandler, this, ros::TransportHints().tcpNoDelay());

        // 话题名称为 "lio_sam/deskew/cloud_deskewed"，队列大小为 1
        // 用于发布去畸变后的点云数据
        pubExtractedCloud = nh.advertise<sensor_msgs::PointCloud2> ("lio_sam/deskew/cloud_deskewed", 1);
        // 发布者将发布与点云数据相关的额外信息(准备去畸变的相关信息）
        pubLaserCloudInfo = nh.advertise<lio_sam::cloud_info> ("lio_sam/deskew/cloud_info", 1);

        allocateMemory();
        resetParameters();

        pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
    }

    // 为 PCL 点云对象分配内存
    void allocateMemory()
    {
        laserCloudIn.reset(new pcl::PointCloud<PointXYZIRT>());
        tmpOusterCloudIn.reset(new pcl::PointCloud<OusterPointXYZIRT>());
        fullCloud.reset(new pcl::PointCloud<PointType>());
        extractedCloud.reset(new pcl::PointCloud<PointType>());

        fullCloud->points.resize(N_SCAN*Horizon_SCAN);

        cloudInfo.startRingIndex.assign(N_SCAN, 0);
        cloudInfo.endRingIndex.assign(N_SCAN, 0);

        cloudInfo.pointColInd.assign(N_SCAN*Horizon_SCAN, 0);
        cloudInfo.pointRange.assign(N_SCAN*Horizon_SCAN, 0);

        resetParameters();
    }

    // 重置参数
    void resetParameters()
    {
        laserCloudIn->clear();
        extractedCloud->clear();
        // reset range matrix for range image projection
        rangeMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32F, cv::Scalar::all(FLT_MAX));

        imuPointerCur = 0;
        firstPointFlag = true;
        odomDeskewFlag = false;

        for (int i = 0; i < queueLength; ++i)
        {
            imuTime[i] = 0;
            imuRotX[i] = 0;
            imuRotY[i] = 0;
            imuRotZ[i] = 0;
        }

        columnIdnCountVec.assign(N_SCAN, 0);
    }

    // 析构函数的名称与类名相同，但前面有一个波浪号 ~
    ~ImageProjection(){}

    // imuHandler 函数用于处理接收到的 IMU 数据
    void imuHandler(const sensor_msgs::Imu::ConstPtr& imuMsg)
    {
        sensor_msgs::Imu thisImu = imuConverter(*imuMsg);

        std::lock_guard<std::mutex> lock1(imuLock);
        imuQueue.push_back(thisImu);
        // printf("imuHandler\n");
        // cout << "imuQueue.size(): " << imuQueue.size() << endl;

        // debug IMU data(测试IMU数据）
        // cout << std::setprecision(6);
        // cout << "IMU acc: " << endl;
        // cout << "x: " << thisImu.linear_acceleration.x << 
        //       ", y: " << thisImu.linear_acceleration.y << 
        //       ", z: " << thisImu.linear_acceleration.z << endl;
        // cout << "IMU gyro: " << endl;
        // cout << "x: " << thisImu.angular_velocity.x << 
        //       ", y: " << thisImu.angular_velocity.y << 
        //       ", z: " << thisImu.angular_velocity.z << endl;
        // double imuRoll, imuPitch, imuYaw;
        // tf::Quaternion orientation;
        // tf::quaternionMsgToTF(thisImu.orientation, orientation);
        // tf::Matrix3x3(orientation).getRPY(imuRoll, imuPitch, imuYaw);
        // cout << "IMU roll pitch yaw: " << endl;
        // cout << "roll: " << imuRoll << ", pitch: " << imuPitch << ", yaw: " << imuYaw << endl << endl;
    }

    // odometryHandler 函数用于处理接收到的里程计数据
    void odometryHandler(const nav_msgs::Odometry::ConstPtr& odometryMsg)
    {
        std::lock_guard<std::mutex> lock2(odoLock);
        odomQueue.push_back(*odometryMsg);
    }

    // const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg：函数的输入参数，是一个指向 sensor_msgs::PointCloud2 消息的常量指针，表示接收到的激光雷达点云数据。
    // sensor_msgs::PointCloud2 是ROS中的标准消息类型，通常用于存储三维点云数据。
    // void cloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg)
    void cloudHandler(const sensor_msgs::PointCloud2::ConstPtr& laserCloudMsg)
    {
        // printf("cloudHandler\n");
        // 缓存点云数据
        if (!cachePointCloud(laserCloudMsg))
            return;

        // 点云数据预处理
        // 调用 deskewInfo 函数进行信息校正（如时间同步、去畸变等）。
        if (!deskewInfo())
            return;

        // printf("projectPointCloud init\n");
        // 点云数据投影
        // 调用 projectPointCloud 函数进行点云的投影，可能是将点云数据从激光雷达坐标系转换到其他坐标系，或者进行某种映射处理。
        projectPointCloud();
        // printf("projectPointCloud success\n");

        // printf("cloudExtraction init\n");
        // 点云数据提取
        // 调用 cloudExtraction 函数来提取点云中的特定信息或对象，例如地面、障碍物或其他感兴趣的部分。
        cloudExtraction();
        // printf("cloudExtraction success\n");

        // printf("publishClouds init\n");
        // 发布处理后的点云数据
        // 调用 publishClouds 函数将处理后的点云数据发布到ROS网络，以供其他节点使用。
        publishClouds();
        // printf("publishClouds success\n");

        // 重置参数
        // 为了准备下一轮处理，确保参数状态清洁。
        resetParameters();
    }

    // bool：函数返回一个布尔值，表示处理是否成功。
    // laserCloudMsg：是传入的激光雷达点云消息，类型为 sensor_msgs::PointCloud2ConstPtr。
    bool cachePointCloud(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg)
    // bool cachePointCloud(const sensor_msgs::PointCloud2::ConstPtr& laserCloudMsg)
    {
        // 遍历点云数据;查看
        // 创建一个PCL点云对象来存储转换后的数据
        // pcl::PointCloud<pcl::PointXYZ> cloud;
        // for (const auto& point : cloud.points) {
        //     ROS_INFO("Point (x, y, z): (%.3f, %.3f, %.3f)", point.x, point.y, point.z);
        // }

        // cache point cloud
        cloudQueue.push_back(*laserCloudMsg);
        if (cloudQueue.size() <= 2)
            return false;

        // convert cloud
        // 将队列中的第一个点云数据移到 currentCloudMsg，并从队列中移除。
        currentCloudMsg = std::move(cloudQueue.front());
        cloudQueue.pop_front();
        if (sensor == SensorType::VELODYNE || sensor == SensorType::LIVOX)
        {
            // 将 currentCloudMsg 转换为 laserCloudIn（PCL 格式）。这些传感器的数据通常是标准的点云数据。
            pcl::moveFromROSMsg(currentCloudMsg, *laserCloudIn);
            // 跑KITTI数据集
            // Remove Nan points                                （为跑kitti数据集改，本没有下面两行）
            // indices 是一个索引向量，用于存储保留的点的索引
            std::vector<int> indices;
            // 通过去除无效点获取索引值，（原始点云，目标点云，索引值）
            pcl::removeNaNFromPointCloud(*laserCloudIn, *laserCloudIn, indices);
            // 跑KITTI数据集/end
        }
        else if (sensor == SensorType::OUSTER)
        {
            // Convert to Velodyne format(OUSTER数据格式不同，转换为Velodyne格式）
            pcl::moveFromROSMsg(currentCloudMsg, *tmpOusterCloudIn);
            laserCloudIn->points.resize(tmpOusterCloudIn->size());
            laserCloudIn->is_dense = tmpOusterCloudIn->is_dense;
            for (size_t i = 0; i < tmpOusterCloudIn->size(); i++)
            {
                auto &src = tmpOusterCloudIn->points[i];
                auto &dst = laserCloudIn->points[i];
                dst.x = src.x;
                dst.y = src.y;
                dst.z = src.z;
                dst.intensity = src.intensity;
                dst.ring = src.ring;
                dst.time = src.t * 1e-9f;
            }
        }
        else
        {
            ROS_ERROR_STREAM("Unknown sensor type: " << int(sensor));
            ros::shutdown();
        }

        // get timestamp
        // 获取当前点云数据的时间戳，消息头中的时间戳是 ROS::Time 类型，通过 toSec() 函数转换为 double 类型
        cloudHeader = currentCloudMsg.header;
        // 扫描开始时间
        timeScanCur = cloudHeader.stamp.toSec();
        // 扫描结束时间
        timeScanEnd = timeScanCur + laserCloudIn->points.back().time;
        
        // check dense flag
        if (laserCloudIn->is_dense == false)
        {
            ROS_ERROR("Point cloud is not in dense format, please remove NaN points first!");
            ros::shutdown();
        }

        // 跑KITTI数据集

        if (!has_ring)
            return true;
        // 跑KITTI数据集/end

        // check ring channel
        // 检查点云数据是否包含 "ring" 通道信息
        // 环通道是某些激光雷达（如Velodyne、Ouster）提供的点的环索引，通常标识点云中每个点所属的激光发射器通道。
        static int ringFlag = 0;
        if (ringFlag == 0)
        {
            ringFlag = -1;
            for (int i = 0; i < (int)currentCloudMsg.fields.size(); ++i)
            {
                if (currentCloudMsg.fields[i].name == "ring")
                {
                    ringFlag = 1;
                    printf("have ring\r\n");
                    break;
                }
            }
            if (ringFlag == -1)
            {
                // 点云数据缺少环通道（ring channel）信息。
                ROS_ERROR("Point cloud ring channel not available, please configure your point cloud data!");
                ros::shutdown();
            }
        }

        // check point time
        // 检查点云数据是否包含 "time" 通道信息
        if (deskewFlag == 0)
        {
            deskewFlag = -1;
            for (auto &field : currentCloudMsg.fields)
            {
                // 有时间信息，表示可以进行去畸变处理
                if (field.name == "time" || field.name == "t")
                {
                    deskewFlag = 1;
                    break;
                }
            }
            if (deskewFlag == -1)
                // 点云数据缺少时间信息，和缺少环通道不同，仅仅警告
                ROS_WARN("Point cloud timestamp not available, deskew function disabled, system will drift significantly!");
        }

        // printf("PointCloud success\r\n");
        return true;
    }

    // 检查并准备去畸变处理所需的信息
    bool deskewInfo()
    {
        // printf("deskewInfo init\n");
        // 确保对IMU数据的访问是线程安全的
        std::lock_guard<std::mutex> lock1(imuLock);
        // 确保对里程计数据的访问是线程安全的
        std::lock_guard<std::mutex> lock2(odoLock);

        // make sure IMU data available for the scan
        // 如果IMU数据队列为空，或者最早的IMU数据时间戳晚于扫描开始时间，或者最晚的IMU数据时间戳早于扫描结束时间，等待IMU数据
        // 要求IMU开始于扫描之前，结束于扫描之后
        if (imuQueue.empty() 
                    || imuQueue.front().header.stamp.toSec() > timeScanCur 
                    || imuQueue.back().header.stamp.toSec() < timeScanEnd)
        {
            // printf("deskewInfo error\n");
            // printf("imuQueue.empty() = %d\r\n", imuQueue.empty());
            // printf("imuQueue.front().header.stamp.toSec() = %f\r\n", imuQueue.front().header.stamp.toSec());
            // printf("timeScanCur = %f\r\n", timeScanCur);
            // printf("imuQueue.back().header.stamp.toSec() = %f\r\n", imuQueue.back().header.stamp.toSec());
            // cout << "imuQueue.back(): " << imuQueue.back() << endl;
            // printf("timeScanEnd = %f\r\n", timeScanEnd);
            ROS_DEBUG("Waiting for IMU data ...");
            return false;
        }
        // printf("deskewInfo init2\n");
        // 准备imu去畸变
        imuDeskewInfo();
        // cout << "cloudInfo.imuAvailable: " << cloudInfo.imuAvailable << endl;
        // printf("deskewInfo init3\n");
        // 准备odom去畸变
        odomDeskewInfo();

        return true;
    }

    void imuDeskewInfo()
    {
        // 判断IMU数据是否可用
        cloudInfo.imuAvailable = false;

        while (!imuQueue.empty())
        {
            // 如果IMU数据的时间戳早于扫描开始时间，弹出队列
            if (imuQueue.front().header.stamp.toSec() < timeScanCur - 0.01)
                imuQueue.pop_front();
            else
                break;
        }

        if (imuQueue.empty())
            return;

        // 计数，用于记录当前IMU数据的索引
        imuPointerCur = 0;

        for (int i = 0; i < (int)imuQueue.size(); ++i)
        {
            sensor_msgs::Imu thisImuMsg = imuQueue[i];
            double currentImuTime = thisImuMsg.header.stamp.toSec();

            // get roll, pitch, and yaw estimation for this scan
            // 对扫描之前和扫描之后的数据做处理
            if (currentImuTime <= timeScanCur)
                imuRPY2rosRPY(&thisImuMsg, &cloudInfo.imuRollInit, &cloudInfo.imuPitchInit, &cloudInfo.imuYawInit);

            if (currentImuTime > timeScanEnd + 0.01)
                break;

            if (imuPointerCur == 0){
                imuRotX[0] = 0;
                imuRotY[0] = 0;
                imuRotZ[0] = 0;
                imuTime[0] = currentImuTime;
                ++imuPointerCur;
                continue;
            }

            // get angular velocity
            // 获取IMU数据的角速度
            double angular_x, angular_y, angular_z;
            imuAngular2rosAngular(&thisImuMsg, &angular_x, &angular_y, &angular_z);

            // integrate rotation
            // 计算IMU数据的旋转信息，将IMU数据的旋转速度与时间差相乘，得到旋转角度的增量。
            double timeDiff = currentImuTime - imuTime[imuPointerCur-1];
            imuRotX[imuPointerCur] = imuRotX[imuPointerCur-1] + angular_x * timeDiff;
            imuRotY[imuPointerCur] = imuRotY[imuPointerCur-1] + angular_y * timeDiff;
            imuRotZ[imuPointerCur] = imuRotZ[imuPointerCur-1] + angular_z * timeDiff;
            imuTime[imuPointerCur] = currentImuTime;
            ++imuPointerCur;
        }
        // 保证IMU数据的时间戳早于扫描结束时间(减少一个，最后溢出才退出循环）
        --imuPointerCur;

        if (imuPointerCur <= 0)
            return;

        cloudInfo.imuAvailable = true;
    }

    void odomDeskewInfo()
    {
        cloudInfo.odomAvailable = false;

        while (!odomQueue.empty())
        {
            if (odomQueue.front().header.stamp.toSec() < timeScanCur - 0.01)
                odomQueue.pop_front();
            else
                break;
        }

        if (odomQueue.empty())
            return;

        if (odomQueue.front().header.stamp.toSec() > timeScanCur)
            return;

        // get start odometry at the beinning of the scan
        nav_msgs::Odometry startOdomMsg;

        for (int i = 0; i < (int)odomQueue.size(); ++i)
        {
            startOdomMsg = odomQueue[i];

            if (ROS_TIME(&startOdomMsg) < timeScanCur)
                continue;
            else
                break;
        }

        tf::Quaternion orientation;
        tf::quaternionMsgToTF(startOdomMsg.pose.pose.orientation, orientation);

        double roll, pitch, yaw;
        tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

        // Initial guess used in mapOptimization
        cloudInfo.initialGuessX = startOdomMsg.pose.pose.position.x;
        cloudInfo.initialGuessY = startOdomMsg.pose.pose.position.y;
        cloudInfo.initialGuessZ = startOdomMsg.pose.pose.position.z;
        cloudInfo.initialGuessRoll  = roll;
        cloudInfo.initialGuessPitch = pitch;
        cloudInfo.initialGuessYaw   = yaw;

        cloudInfo.odomAvailable = true;

        // get end odometry at the end of the scan
        odomDeskewFlag = false;

        if (odomQueue.back().header.stamp.toSec() < timeScanEnd)
            return;

        nav_msgs::Odometry endOdomMsg;

        for (int i = 0; i < (int)odomQueue.size(); ++i)
        {
            endOdomMsg = odomQueue[i];

            if (ROS_TIME(&endOdomMsg) < timeScanEnd)
                continue;
            else
                break;
        }

        if (int(round(startOdomMsg.pose.covariance[0])) != int(round(endOdomMsg.pose.covariance[0])))
            return;

        Eigen::Affine3f transBegin = pcl::getTransformation(startOdomMsg.pose.pose.position.x, startOdomMsg.pose.pose.position.y, startOdomMsg.pose.pose.position.z, roll, pitch, yaw);

        tf::quaternionMsgToTF(endOdomMsg.pose.pose.orientation, orientation);
        tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
        Eigen::Affine3f transEnd = pcl::getTransformation(endOdomMsg.pose.pose.position.x, endOdomMsg.pose.pose.position.y, endOdomMsg.pose.pose.position.z, roll, pitch, yaw);

        Eigen::Affine3f transBt = transBegin.inverse() * transEnd;

        float rollIncre, pitchIncre, yawIncre;
        pcl::getTranslationAndEulerAngles(transBt, odomIncreX, odomIncreY, odomIncreZ, rollIncre, pitchIncre, yawIncre);

        odomDeskewFlag = true;
    }

    // 找到给定时间点的旋转信息，求解旋转矩阵
    void findRotation(double pointTime, float *rotXCur, float *rotYCur, float *rotZCur)
    {
        *rotXCur = 0; *rotYCur = 0; *rotZCur = 0;

        int imuPointerFront = 0;
        // imuPointerCur 是IMU数据的索引
        while (imuPointerFront < imuPointerCur)
        {
            // 从某个时间点前对imu数据进行处理，找到时间点前的最后一个imu数据
            if (pointTime < imuTime[imuPointerFront])
                break;
            ++imuPointerFront;
        }

        if (pointTime > imuTime[imuPointerFront] || imuPointerFront == 0)
        {
            // 如果IMU数据的时间戳早于扫描开始时间，或者IMU数据的时间戳晚于扫描结束时间，返回初始值
            //                                      point
            // 错误         |----------------|
            // 错误                                             |----------------|
            // 正确                             |------^---------|
            //                        imuPointerBack * * imuPointerFront
            *rotXCur = imuRotX[imuPointerFront];
            *rotYCur = imuRotY[imuPointerFront];
            *rotZCur = imuRotZ[imuPointerFront];
        } else {
            int imuPointerBack = imuPointerFront - 1;
            double ratioFront = (pointTime - imuTime[imuPointerBack]) / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
            double ratioBack = (imuTime[imuPointerFront] - pointTime) / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
            // 通过线性插值计算给定时间点的旋转信息
            *rotXCur = imuRotX[imuPointerFront] * ratioFront + imuRotX[imuPointerBack] * ratioBack;
            *rotYCur = imuRotY[imuPointerFront] * ratioFront + imuRotY[imuPointerBack] * ratioBack;
            *rotZCur = imuRotZ[imuPointerFront] * ratioFront + imuRotZ[imuPointerBack] * ratioBack;
        }
    }

    // 位姿线性插值
    void findPosition(double relTime, float *posXCur, float *posYCur, float *posZCur)
    {
        *posXCur = 0; *posYCur = 0; *posZCur = 0;

        // If the sensor moves relatively slow, like walking speed, positional deskew seems to have little benefits. Thus code below is commented.
        // 如果传感器的移动速度相对较慢，比如走路的速度，位置偏移校正似乎几乎没有什么好处。因此，下面的代码被注释掉了。

        // if (cloudInfo.odomAvailable == false || odomDeskewFlag == false)
        //     return;

        // float ratio = relTime / (timeScanEnd - timeScanCur);

        // *posXCur = ratio * odomIncreX;
        // *posYCur = ratio * odomIncreY;
        // *posZCur = ratio * odomIncreZ;
    }

    // 对点云数据进行去畸变处理
    PointType deskewPoint(PointType *point, double relTime)
    {
        if (deskewFlag == -1 || cloudInfo.imuAvailable == false)
            return *point;

        double pointTime = timeScanCur + relTime;

        float rotXCur, rotYCur, rotZCur;
        findRotation(pointTime, &rotXCur, &rotYCur, &rotZCur);

        float posXCur, posYCur, posZCur;
        findPosition(relTime, &posXCur, &posYCur, &posZCur);

        if (firstPointFlag == true)
        {
            transStartInverse = (pcl::getTransformation(posXCur, posYCur, posZCur, rotXCur, rotYCur, rotZCur)).inverse();
            firstPointFlag = false;
        }

        // transform points to start
        Eigen::Affine3f transFinal = pcl::getTransformation(posXCur, posYCur, posZCur, rotXCur, rotYCur, rotZCur);
        Eigen::Affine3f transBt = transStartInverse * transFinal;

        PointType newPoint;
        newPoint.x = transBt(0,0) * point->x + transBt(0,1) * point->y + transBt(0,2) * point->z + transBt(0,3);
        newPoint.y = transBt(1,0) * point->x + transBt(1,1) * point->y + transBt(1,2) * point->z + transBt(1,3);
        newPoint.z = transBt(2,0) * point->x + transBt(2,1) * point->y + transBt(2,2) * point->z + transBt(2,3);
        newPoint.intensity = point->intensity;

        return newPoint;
    }

    // 对点云数据进行配准处理，求解旋转矩阵
    void projectPointCloud()
    {
        int cloudSize = laserCloudIn->points.size();
        // 跑KITTI数据集
        bool halfPassed = false;
        cloudInfo.startOrientation = -atan2(laserCloudIn->points[0].y, laserCloudIn->points[0].x);
        cloudInfo.endOrientation   = -atan2(laserCloudIn->points[laserCloudIn->points.size() - 1].y,
                                                     laserCloudIn->points[laserCloudIn->points.size() - 1].x) + 2 * M_PI;
        if (cloudInfo.endOrientation - cloudInfo.startOrientation > 3 * M_PI) {
            cloudInfo.endOrientation -= 2 * M_PI;
        } else if (cloudInfo.endOrientation - cloudInfo.startOrientation < M_PI)
            cloudInfo.endOrientation += 2 * M_PI;
        cloudInfo.orientationDiff = cloudInfo.endOrientation - cloudInfo.startOrientation;
        // 跑KITTI数据集/end

        // range image projection
        for (int i = 0; i < cloudSize; ++i)
        {
            PointType thisPoint;
            thisPoint.x = laserCloudIn->points[i].x;
            thisPoint.y = laserCloudIn->points[i].y;
            thisPoint.z = laserCloudIn->points[i].z;
            thisPoint.intensity = laserCloudIn->points[i].intensity;

            float range = pointDistance(thisPoint);
            if (range < lidarMinRange || range > lidarMaxRange)
                continue;

            // 跑KITTI数据集
            // int rowIdn = laserCloudIn->points[i].ring;
            // printf("rowIdn: %d\r\n", rowIdn);

            // 跑KITTI数据集
            int rowIdn = -1;
            if (has_ring == true){
                rowIdn = laserCloudIn->points[i].ring;
            }
            else{
                float verticalAngle, horizonAngle;
                verticalAngle = atan2(thisPoint.z, sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y)) * 180 / M_PI;
                rowIdn = (verticalAngle + ang_bottom) / ang_res_y;
            }
            // 跑KITTI数据集/end

            if (rowIdn < 0 || rowIdn >= N_SCAN)
                continue;

            if (rowIdn % downsampleRate != 0)
                continue;

            int columnIdn = -1;
            if (sensor == SensorType::VELODYNE || sensor == SensorType::OUSTER)
            {
                float horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;
                static float ang_res_x = 360.0/float(Horizon_SCAN);
                columnIdn = -round((horizonAngle-90.0)/ang_res_x) + Horizon_SCAN/2;
                if (columnIdn >= Horizon_SCAN)
                    columnIdn -= Horizon_SCAN;
            }
            else if (sensor == SensorType::LIVOX)
            {
                columnIdn = columnIdnCountVec[rowIdn];
                columnIdnCountVec[rowIdn] += 1;
            }
            
            if (columnIdn < 0 || columnIdn >= Horizon_SCAN)
                continue;

            if (rangeMat.at<float>(rowIdn, columnIdn) != FLT_MAX)
                continue;

            // 跑KITTI数据集
            thisPoint = deskewPoint(&thisPoint, laserCloudIn->points[i].time);
            // 跑KITTI数据集

            if (has_ring == true)
                thisPoint = deskewPoint(&thisPoint, laserCloudIn->points[i].time);
            else {
                    float ori = -atan2(thisPoint.y, thisPoint.x);
                    if (!halfPassed) {
                        if (ori < cloudInfo.startOrientation - M_PI / 2) {
                            ori += 2 * M_PI;
                        } else if (ori > cloudInfo.startOrientation + M_PI * 3 / 2) {
                            ori -= 2 * M_PI;
                        }
                        if (ori - cloudInfo.startOrientation > M_PI) {
                            halfPassed = true;
                        }
                    } else {
                        ori += 2 * M_PI;
                        if (ori < cloudInfo.endOrientation - M_PI * 3 / 2) {
                            ori += 2 * M_PI;
                        } else if (ori > cloudInfo.endOrientation + M_PI / 2) {
                            ori -= 2 * M_PI;
                        }
                    }
                    float relTime = (ori - cloudInfo.startOrientation) / cloudInfo.orientationDiff;
                    // 激光雷达10Hz，周期0.1
                    laserCloudIn->points[i].time = 0.1 * relTime;
                    thisPoint = deskewPoint(&thisPoint, laserCloudIn->points[i].time);
            }
            // 跑KITTI数据集/end

            rangeMat.at<float>(rowIdn, columnIdn) = range;

            int index = columnIdn + rowIdn * Horizon_SCAN;
            fullCloud->points[index] = thisPoint;
        }
        // printf("projectPointCloud success\r\n");
    }

    void cloudExtraction()
    {
        int count = 0;
        // extract segmented cloud for lidar odometry
        for (int i = 0; i < N_SCAN; ++i)
        {
            cloudInfo.startRingIndex[i] = count - 1 + 5;

            for (int j = 0; j < Horizon_SCAN; ++j)
            {
                if (rangeMat.at<float>(i,j) != FLT_MAX)
                {
                    // mark the points' column index for marking occlusion later
                    cloudInfo.pointColInd[count] = j;
                    // save range info
                    cloudInfo.pointRange[count] = rangeMat.at<float>(i,j);
                    // save extracted cloud
                    extractedCloud->push_back(fullCloud->points[j + i*Horizon_SCAN]);
                    // size of extracted cloud
                    ++count;
                }
            }
            cloudInfo.endRingIndex[i] = count -1 - 5;
        }
        // printf("cloudExtraction success\r\n");
    }
    
    void publishClouds()
    {
        // cloudInfo是一个自定义的消息类型，用于发布与点云数据相关的额外信息
        // 具体查看lio_sam包中的msg/cloud_info.msg
        cloudInfo.header = cloudHeader;
        // cloud_deskewed是一个sensor_msgs::PointCloud2类型的消息，用于发布去畸变后的点云数据
        // 
        // 
        // 
        // 
        cloudInfo.cloud_deskewed  = publishCloud(pubExtractedCloud, extractedCloud, cloudHeader.stamp, lidarFrame);
        pubLaserCloudInfo.publish(cloudInfo);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lio_sam");

    ImageProjection IP;
    
    // \033[1;32m：这是一个ANSI转义序列，用于设置文本颜色和样式。
    // \033表示转义字符的开始，通常用来更改文本的颜色和格式。
    // [1;32m表示粗体绿色文本。
    // \033[0m：这个转义序列重置格式，使得后续的文本恢复到默认样式。
    ROS_INFO("\033[1;32m----> Image Projection Started.\033[0m");

    // ros::MultiThreadedSpinner是ROS中的一个类，用于多线程处理回调。
    // 括号内的数字 3 表示使用3个线程来处理ROS消息。这样可以提高回调处理效率，尤其在消息频繁或处理时间较长的情况下。
    ros::MultiThreadedSpinner spinner(3);
    // 在调用 spin() 后，程序会一直等待和处理来自ROS的消息，直到节点关闭。
    spinner.spin();
    
    return 0;
}
