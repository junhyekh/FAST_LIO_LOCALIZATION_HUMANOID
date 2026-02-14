#include <iostream>
#include <queue>
#include <thread>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float32.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sstream>
#include <iomanip>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <open3d/Open3D.h>

#include "open3d_registration/open3d_registration.h"
#include "open3d_conversions/open3d_conversions.h"

#define PI 3.1415926

/**
 * @brief Simple 1D Kalman Filter implementation for pose estimation smoothing
 * 
 * This class implements a basic Kalman filter to reduce noise in pose estimates.
 * It's used to smooth the localization results and reduce jitter in the output.
 */
class KalmanFilter
{
public:
    KalmanFilter()
    {
    }

    /**
     * @brief Initialize the Kalman filter with process and measurement noise parameters
     * @param processVar Process noise variance (how much the state can change)
     * @param estimatedMeasVar Measurement noise variance (how noisy the measurements are)
     * @param posteriEstimate Initial posterior estimate
     * @param posteriErrorEstimate Initial posterior error estimate
     */
    void KalmanFilterInit(double processVar, double estimatedMeasVar, double posteriEstimate = 0.0, double posteriErrorEstimate = 1.0)
    {
        processVar_ = processVar;
        estimatedMeasVar_ = estimatedMeasVar;
        posteriEstimate_ = posteriEstimate;
        posteriErrorEstimate_ = posteriErrorEstimate;
    }

    /**
     * @brief Update the filter with a new noisy measurement
     * @param measurement The new measurement to incorporate
     * 
     * This implements the standard Kalman filter update equations:
     * 1. Predict step (priori estimates)
     * 2. Update step (posterior estimates using measurement)
     */
    void inputLatestNoisyMeasurement(double measurement)
    {
        // Predict step: estimate current state based on previous state
        double prioriEstimate = posteriEstimate_;
        double prioriErrorEstimate = posteriErrorEstimate_ + processVar_;

        // Update step: incorporate new measurement
        double blendingFactor = prioriErrorEstimate / (prioriErrorEstimate + estimatedMeasVar_);
        posteriEstimate_ = prioriEstimate + blendingFactor * (measurement - prioriEstimate);
        posteriErrorEstimate_ = (1 - blendingFactor) * prioriErrorEstimate;
    }

    /**
     * @brief Get the latest filtered estimate
     * @return The current posterior estimate
     */
    double getLatestEstimatedMeasurement()
    {
        return posteriEstimate_;
    }

private:
    double processVar_;           ///< Process noise variance
    double estimatedMeasVar_;     ///< Measurement noise variance
    double posteriEstimate_;      ///< Current posterior estimate
    double posteriErrorEstimate_; ///< Current posterior error estimate
};

/**
 * @brief Global localization node for humanoid robots using Open3D point cloud registration
 * 
 * This class implements a robust localization system that:
 * 1. Subscribes to FAST-LIO odometry and point cloud data
 * 2. Performs point cloud registration against a pre-built map
 * 3. Publishes localization results and transforms
 * 4. Uses Kalman filtering for pose smoothing
 * 5. Supports both coarse and fine registration for efficiency
 */
class GlobalLocalization : public rclcpp::Node
{
private:
    /* data */
public:
    GlobalLocalization();
    ~GlobalLocalization();

    /// @brief Initialize localization system and load map
    void LocalizationInitialize();

    /// @brief Callback for FAST-LIO odometry data (base_link to odom transform)
    /// @param baselink2odom Odometry message containing pose and twist information
    void CallbackBaselink2Odom(const nav_msgs::msg::Odometry::SharedPtr baselink2odom);

    /// @brief Callback for point cloud data in base_link frame
    /// @param scan_in_baselink Point cloud message from LiDAR
    void CallbackScan(const sensor_msgs::msg::PointCloud2::SharedPtr scan_in_baselink);

    /// @brief Callback for initial pose estimate (e.g., from RViz 2D Pose Estimate)
    /// @param initialpose Initial pose with covariance for localization initialization
    void CallbackInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr initialpose);

    /// @brief Start the localization thread
    void StartLoc();

    /// @brief Main localization loop running in separate thread
    void Localization();

    /// @brief Convert Euler angles to 3x3 rotation matrix
    /// @param euler Euler angles in radians (roll, pitch, yaw)
    /// @return 3x3 rotation matrix
    Eigen::Matrix3d Euler2Matrix3d(const Eigen::Vector3d euler);

    /// @brief Get TF transform between two frames and convert to 4x4 matrix
    /// @param frame_id Parent frame ID
    /// @param child_frame_id Child frame ID
    /// @param matrix Output 4x4 transformation matrix
    /// @return True if transform was found, false otherwise
    bool GetTfTransformToMatrix(
        std::string frame_id, std::string child_frame_id, Eigen::Matrix4d &matrix);

    /// @brief Compute Euclidean distance between two 3D points
    /// @param a First 3D point
    /// @param b Second 3D point
    /// @return Euclidean distance
    double ComputeMotionDis(const Eigen::Vector3d &a, const Eigen::Vector3d &b);

    // ===== Helpers to reduce repetition when writing messages =====
    void setPoseFromMatrix(const Eigen::Matrix4d &T, geometry_msgs::msg::Pose &pose);
    void setTransformFromMatrix(const Eigen::Matrix4d &T, geometry_msgs::msg::Transform &tf);
    std::string matrix4ToString(const Eigen::Matrix4d &T);

private:
    // ===== ROS2 Communication Components =====
    
    /// @brief Subscribers for incoming data
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_baselink2odom_;           ///< FAST-LIO odometry
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_scan_cur_;         ///< Current LiDAR scan
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_initialpose_; ///< Initial pose

    /// @brief Latest odometry data and thread safety
    nav_msgs::msg::Odometry pose_baselink2odom_;
    std::mutex lock_pose_baselink2odom_;

    // ===== Transformation Matrices =====
    
    /// @brief Key transformation matrices for the localization system
    Eigen::Matrix4d mat_baselink2odom_;      ///< Base link to odom transform
    Eigen::Matrix4d mat_odom2map_;           ///< Odom to map transform (main localization result)
    Eigen::Matrix4d mat_odom2map_kalman_;    ///< Kalman-filtered odom to map transform
    std::mutex lock_mat_odom2map_;           ///< Thread safety for transform matrices

    Eigen::Matrix4d mat_baselink2map_;       ///< Base link to map transform
    Eigen::Matrix4d mat_initialpose_;        ///< Initial pose estimate

    /// @brief Static transforms for robot configuration
    Eigen::Matrix4d mat_baselink2motionlink_; ///< Base link to motion link transform
    Eigen::Matrix4d mat_imulink2baselink_;    ///< IMU link to base link transform

    std::vector<float> initialpose_;         ///< Initial pose parameters

    // ===== Point Cloud Data =====
    
    /// @brief Open3D point cloud objects for registration
    std::shared_ptr<open3d::geometry::PointCloud> pcd_map_ori_;      ///< Original map point cloud
    std::shared_ptr<open3d::geometry::PointCloud> pcd_map_coarse_;   ///< Downsampled map for coarse registration
    std::shared_ptr<open3d::geometry::PointCloud> pcd_map_fine_;     ///< Downsampled map for fine registration
    std::shared_ptr<open3d::geometry::PointCloud> pcd_map_cur_;      ///< Current map region of interest
    std::shared_ptr<open3d::geometry::PointCloud> pcd_scan_cur_;     ///< Current LiDAR scan

    /// @brief Point cloud queue for processing multiple scans
    std::queue<open3d::geometry::PointCloud> que_pcd_scan_;
    int queue_maxsize_;                      ///< Maximum size of scan queue

    // ===== Registration Parameters =====
    
    double voxelsize_coarse_;                ///< Voxel size for coarse registration (larger = faster)
    double voxelsize_fine_;                  ///< Voxel size for fine registration (smaller = more accurate)
    double threshold_fitness_;               ///< Registration fitness threshold for regular updates
    double threshold_fitness_init_;          ///< Registration fitness threshold for initialization

    // ===== Threading and Control =====
    
    std::thread thread_loc_;                 ///< Localization processing thread
    std::mutex lock_scan_;                   ///< Thread safety for scan data
    std::mutex lock_exit_;                   ///< Thread safety for exit flag
    bool flag_exit_;                         ///< Flag to signal thread termination

    // ===== Publishers for Output Data =====
    
    /// @brief Odometry publishers
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_baselink2map_;        ///< Base link to map transform
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_baselink2map_kalman_; ///< Kalman-filtered transform
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_motionlink2map_;      ///< Motion link to map transform
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom2map_;            ///< Odom to map transform
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom2map_kalman_;     ///< Kalman-filtered odom to map

    rclcpp::Time timestamp_odom_;            ///< Timestamp of latest odometry
    std::mutex lock_timestamp_;              ///< Thread safety for timestamps

    /// @brief Point cloud publishers for visualization
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_map_;           ///< Published map
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_scan_;          ///< Current scan
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_scan2map_;      ///< Scan transformed to map frame
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_submap_;        ///< Current submap region

    /// @brief Localization result publishers
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_localization_3d_;           ///< 3D localization pose
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_localization_3d_confidence_;         ///< Registration confidence
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_localization_3d_delay_ms_;           ///< Processing delay

    /// @brief Localization result messages
    geometry_msgs::msg::PoseStamped localization_3d_;
    std_msgs::msg::Float32 localization_3d_confidence_;
    std_msgs::msg::Float32 localization_3d_delay_ms_;

    // ===== TF2 Components =====
    
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_; ///< For publishing static transforms
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;           ///< For publishing dynamic transforms

    // ===== Configuration Parameters =====
    
    bool save_scan_;                         ///< Whether to save scan data to file
    std::string path_map_;                   ///< Path to the map file (.ply format)
    double loc_frequence_;                   ///< Localization frequency in Hz
    std::vector<double> voxel_scales_;       ///< Multiscale voxel factors for initialization/ICP
    int num_trial_init_ = 0;                 ///< Max number of re-initialization trials on timeout
    bool use_cuda_ = false;                  ///< Whether to use CUDA ICP path when available

    // ===== Point Cloud Processing Limits =====
    
    int maxpoints_source_ = 50000;           ///< Maximum points in source (scan) for registration
    int maxpoints_target_ = 200000;          ///< Maximum points in target (map) for registration

    // ===== Localization State =====
    
    bool loc_initialized_ = false;           ///< Whether localization has been initialized
    rclcpp::Time time_loc_init_;             ///< Time when localization was initialized
    double loc_fitness_;                     ///< Current registration fitness score
    double confidence_loc_th_;               ///< Confidence threshold for localization

    // ===== Initialization behavior control =====
    bool init_loc_enable_ = true;            ///< Enable first localization trial at startup
    double init_loc_timeout_s_ = 10.0;       ///< Timeout for initial localization (seconds)

    // ===== Kalman Filters for Pose Smoothing =====
    
    KalmanFilter kf_baselink_x_;             ///< Kalman filter for X position
    KalmanFilter kf_baselink_y_;             ///< Kalman filter for Y position
    KalmanFilter kf_baselink_z_;             ///< Kalman filter for Z position
    KalmanFilter kalman_filter_odom2map_;    ///< Kalman filter for odom to map transform

    /// @brief Kalman filter parameters for each axis
    std::vector<float> kf_param_x_;          ///< X-axis Kalman filter parameters
    std::vector<float> kf_param_y_;          ///< Y-axis Kalman filter parameters
    std::vector<float> kf_param_z_;          ///< Z-axis Kalman filter parameters

    // ===== Kalman Filter Configuration =====
    
    bool filter_odom2map_ = false;           ///< Whether to apply Kalman filtering to odom2map
    double kalman_processVar2_ = 0.0;        ///< Process variance for odom2map Kalman filter
    double kalman_estimatedMeasVar2_ = 0.0;  ///< Measurement variance for odom2map Kalman filter

    // ===== TF2 Components for Transform Handling =====
    
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;           ///< TF2 buffer for transform queries
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_; ///< TF2 listener for transform updates

    // ===== Motion Tracking =====
    
    Eigen::Vector3d last_loc_;               ///< Last localization position
    rclcpp::Time time_last_loc_;             ///< Time of last localization
    double dis_updatemap_;                   ///< Distance threshold for map updates
};

// ===== Constructor Implementation =====

/**
 * @brief Constructor for the GlobalLocalization node
 * 
 * Initializes all ROS2 components, parameters, publishers, subscribers,
 * and sets up the localization system for operation.
 */
GlobalLocalization::GlobalLocalization() : Node("global_localization_node")
{
    // Initialize TF2 components for transform handling
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    static_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

    // ===== Parameter Declaration and Retrieval =====
    
    // Declare all parameters with default values
    this->declare_parameter("path_map", "");                           ///< Path to map file
    this->declare_parameter("pcd_queue_maxsize", 10);                  ///< Maximum scan queue size
    this->declare_parameter("voxelsize_coarse", 0.15);                 ///< Coarse registration voxel size
    this->declare_parameter("voxelsize_fine", 0.1);                    ///< Fine registration voxel size
    this->declare_parameter("threshold_fitness", 0.5);                 ///< Regular registration threshold
    this->declare_parameter("threshold_fitness_init", 0.5);            ///< Initial registration threshold
    this->declare_parameter("loc_frequence", 2.5);                     ///< Localization frequency (Hz)
    this->declare_parameter("save_scan", false);                       ///< Whether to save scans
    this->declare_parameter("hidden_removal", false);                  ///< Whether to remove hidden points
    this->declare_parameter("maxpoints_source", 80000);                ///< Max source points for registration
    this->declare_parameter("maxpoints_target", 400000);               ///< Max target points for registration
    this->declare_parameter("filter_odom2map", false);                 ///< Enable Kalman filtering
    this->declare_parameter("kalman_processVar2", 0.001);              ///< Kalman process variance
    this->declare_parameter("kalman_estimatedMeasVar2", 0.02);         ///< Kalman measurement variance
    this->declare_parameter("confidence_loc_th", 0.7);                 ///< Confidence threshold
    this->declare_parameter("init_loc_enable", true);                  ///< Enable initial localization trial
    this->declare_parameter("init_loc_timeout_s", 10.0);               ///< Initial localization timeout (seconds)
    this->declare_parameter<std::vector<double>>("initialpose", {});   ///< Initial pose [x y z roll pitch yaw] in degrees
    this->declare_parameter("dis_updatemap", 3.5);                     ///< Map update distance threshold
    // Kalman filter parameters (per-axis) for baselink->map smoothing
    this->declare_parameter<std::vector<double>>("kf_baselink2map.x", std::vector<double>{});
    this->declare_parameter<std::vector<double>>("kf_baselink2map.y", std::vector<double>{});
    this->declare_parameter<std::vector<double>>("kf_baselink2map.z", std::vector<double>{});
    // Voxel scale for downsampling in initial localization
    this->declare_parameter("voxel_scales", std::vector<double>{1, 4, 6, 10});
    this->declare_parameter("num_trial_init", 5);
                      ///< Voxel scale for downsampling in initial localization
                    
    // Use CUDA for registration
    this->declare_parameter("use_cuda", false);

    // Retrieve parameter values
    path_map_ = this->get_parameter("path_map").as_string();
    queue_maxsize_ = this->get_parameter("pcd_queue_maxsize").as_int();
    voxelsize_coarse_ = this->get_parameter("voxelsize_coarse").as_double();
    voxelsize_fine_ = this->get_parameter("voxelsize_fine").as_double();
    threshold_fitness_ = this->get_parameter("threshold_fitness").as_double();
    threshold_fitness_init_ = this->get_parameter("threshold_fitness_init").as_double();
    loc_frequence_ = this->get_parameter("loc_frequence").as_double();
    save_scan_ = this->get_parameter("save_scan").as_bool();
    maxpoints_source_ = this->get_parameter("maxpoints_source").as_int();
    maxpoints_target_ = this->get_parameter("maxpoints_target").as_int();
    filter_odom2map_ = this->get_parameter("filter_odom2map").as_bool();
    kalman_processVar2_ = this->get_parameter("kalman_processVar2").as_double();
    kalman_estimatedMeasVar2_ = this->get_parameter("kalman_estimatedMeasVar2").as_double();
    confidence_loc_th_ = this->get_parameter("confidence_loc_th").as_double();
    init_loc_enable_ = this->get_parameter("init_loc_enable").as_bool();
    init_loc_timeout_s_ = this->get_parameter("init_loc_timeout_s").as_double();
    dis_updatemap_ = this->get_parameter("dis_updatemap").as_double();
    voxel_scales_ = this->get_parameter("voxel_scales").as_double_array();
    use_cuda_ = this->get_parameter("use_cuda").as_bool();
    num_trial_init_ = this->get_parameter("num_trial_init").as_int();

    // Retrieve Kalman filter params if provided
    {
        auto vx = this->get_parameter("kf_baselink2map.x").as_double_array();
        auto vy = this->get_parameter("kf_baselink2map.y").as_double_array();
        auto vz = this->get_parameter("kf_baselink2map.z").as_double_array();
        kf_param_x_.assign(vx.begin(), vx.end());
        kf_param_y_.assign(vy.begin(), vy.end());
        kf_param_z_.assign(vz.begin(), vz.end());
    }

    // Optional initialpose from parameters
    auto iparam = this->get_parameter("initialpose").as_double_array();
    if (iparam.size() >= 6) {
        initialpose_.assign(iparam.begin(), iparam.begin() + 6);
        mat_initialpose_.setIdentity();
        mat_initialpose_.block<3,3>(0,0) = Euler2Matrix3d(Eigen::Vector3d(initialpose_[3], initialpose_[4], initialpose_[5]));
        mat_initialpose_.block<3,1>(0,3) = Eigen::Vector3d(initialpose_[0], initialpose_[1], initialpose_[2]);
    }

    // ===== Publisher Initialization =====
    
    // Initialize odometry publishers
    pub_baselink2map_ = this->create_publisher<nav_msgs::msg::Odometry>("/baselink2map", 10);
    pub_baselink2map_kalman_ = this->create_publisher<nav_msgs::msg::Odometry>("/baselink2map_kalman", 10);
    pub_motionlink2map_ = this->create_publisher<nav_msgs::msg::Odometry>("/motionlink2map", 10);
    pub_odom2map_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom2map", 10);
    pub_odom2map_kalman_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom2map_kalman", 10);

    // Initialize point cloud publishers
    pub_map_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/map", 10);
    pub_scan_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/scan", 10);
    pub_scan2map_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/scan2map", 10);
    pub_submap_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/submap", 10);

    // Initialize localization result publishers
    pub_localization_3d_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/localization_3d", 10);
    pub_localization_3d_confidence_ = this->create_publisher<std_msgs::msg::Float32>("/localization_3d_confidence", 10);
    pub_localization_3d_delay_ms_ = this->create_publisher<std_msgs::msg::Float32>("/localization_3d_delay_ms", 10);

    // ===== Subscriber Initialization =====
    
    // Subscribe to FAST-LIO odometry output
    sub_baselink2odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/Odometry_LIO", 10, std::bind(&GlobalLocalization::CallbackBaselink2Odom, this, std::placeholders::_1));
    
    // Subscribe to registered point cloud from FAST-LIO
    sub_scan_cur_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/cloud_registered", 10, std::bind(&GlobalLocalization::CallbackScan, this, std::placeholders::_1));
    
    // Subscribe to initial pose estimates (e.g., from RViz)
    sub_initialpose_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initialpose", 10, std::bind(&GlobalLocalization::CallbackInitialPose, this, std::placeholders::_1));

    // ===== Matrix Initialization =====
    
    // Initialize all transformation matrices to identity
    mat_odom2map_ = Eigen::Matrix4d::Identity();
    mat_odom2map_kalman_ = Eigen::Matrix4d::Identity();
    mat_baselink2map_ = Eigen::Matrix4d::Identity();
    // Only default to identity if no initial pose was provided via parameters
    if (initialpose_.empty()) {
        mat_initialpose_ = Eigen::Matrix4d::Identity();
    }
    mat_baselink2motionlink_ = Eigen::Matrix4d::Identity();
    mat_imulink2baselink_ = Eigen::Matrix4d::Identity();

    // ===== Kalman Filter Initialization =====
    
    // Initialize Kalman filters for pose smoothing
    // Note: kf_param vectors need to be populated before calling KalmanFilterInit
    // kf_baselink_x_.KalmanFilterInit(kf_param_x_[0], kf_param_x_[1]);
    // kf_baselink_y_.KalmanFilterInit(kf_param_y_[0], kf_param_y_[1]);
    // kf_baselink_z_.KalmanFilterInit(kf_param_z_[0], kf_param_z_[1]);
    // kalman_filter_odom2map_.KalmanFilterInit(kalman_processVar2_, kalman_estimatedMeasVar2_);

    // ===== System State Initialization =====
    
    flag_exit_ = false;                      ///< Thread control flag
    time_last_loc_ = this->now();            ///< Initialize last localization time
    last_loc_ = Eigen::Vector3d(0.0, 0.0, -5000.0); // Force first submap update

    RCLCPP_INFO(this->get_logger(), "GlobalLocalization node initialized successfully");

    // Start localization thread
    StartLoc();
}

/**
 * @brief Destructor for the GlobalLocalization node
 * 
 * Ensures proper cleanup of threads and resources when the node is shut down.
 */
GlobalLocalization::~GlobalLocalization()
{
    flag_exit_ = true;                       ///< Signal threads to exit
    if (thread_loc_.joinable()) {
        thread_loc_.join();                  ///< Wait for localization thread to finish
    }
}

// ===== Method Implementations =====
// Note: The following methods need to be implemented based on the original ROS1 code
// This is a framework - the actual implementation would convert ROS1 specific code
// to ROS2 equivalents

/**
 * @brief Initialize the localization system
 * 
 * This method should:
 * 1. Load the point cloud map from file
 * 2. Create coarse and fine versions of the map
 * 3. Set up initial transforms
 * 4. Prepare the registration system
 */
void GlobalLocalization::LocalizationInitialize()
{
    // Load map
    if (path_map_.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Parameter 'path_map' is empty. Please set a valid .ply path.");
        return;
    }

    pcd_map_ori_.reset(new open3d::geometry::PointCloud);
    pcd_map_coarse_.reset(new open3d::geometry::PointCloud);
    pcd_map_cur_.reset(new open3d::geometry::PointCloud);
    pcd_scan_cur_.reset(new open3d::geometry::PointCloud);
    pcd_map_fine_.reset(new open3d::geometry::PointCloud);

    if (!open3d::io::ReadPointCloud(path_map_, *pcd_map_ori_) || pcd_map_ori_->IsEmpty()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to read map: %s", path_map_.c_str());
        return;
    }
    pcd_map_ori_->PaintUniformColor({1, 0, 0});

    // Downsample and estimate normals
    // Validate and sanitize voxel sizes to avoid Open3D exceptions
    if (voxelsize_coarse_ <= 0.0) {
        RCLCPP_WARN(this->get_logger(), "voxelsize_coarse (%.6f) invalid; resetting to 0.10 m", voxelsize_coarse_);
        voxelsize_coarse_ = 0.10;
    }
    if (voxelsize_fine_ <= 0.0) {
        RCLCPP_WARN(this->get_logger(), "voxelsize_fine (%.6f) invalid; resetting to 0.05 m", voxelsize_fine_);
        voxelsize_fine_ = 0.05;
    }
    if (voxelsize_fine_ >= voxelsize_coarse_) {
        voxelsize_fine_ = std::max(1e-3, voxelsize_coarse_ * 0.5);
        RCLCPP_WARN(this->get_logger(), "Adjusted voxelsize_fine to %.6f (coarse=%.6f)", voxelsize_fine_, voxelsize_coarse_);
    }
    RCLCPP_INFO(this->get_logger(), "voxelsize_coarse: %.6f, voxelsize_fine: %.6f", voxelsize_coarse_, voxelsize_fine_);

    try {
        pcd_map_coarse_ = pcd_map_ori_->VoxelDownSample(voxelsize_coarse_);
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "VoxelDownSample(coarse=%.6f) failed: %s. Using original map.", voxelsize_coarse_, e.what());
        pcd_map_coarse_ = std::make_shared<open3d::geometry::PointCloud>(*pcd_map_ori_);
    }
    try {
        pcd_map_coarse_->EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(voxelsize_coarse_ * 2, 30));
    } catch (const std::exception &e) {
        RCLCPP_WARN(this->get_logger(), "EstimateNormals(coarse) failed: %s", e.what());
    }

    try {
        pcd_map_fine_ = pcd_map_ori_->VoxelDownSample(voxelsize_fine_);
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "VoxelDownSample(fine=%.6f) failed: %s. Using original map.", voxelsize_fine_, e.what());
        pcd_map_fine_ = std::make_shared<open3d::geometry::PointCloud>(*pcd_map_ori_);
    }
    try {
        pcd_map_fine_->EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(voxelsize_fine_ * 2, 30));
    } catch (const std::exception &e) {
        RCLCPP_WARN(this->get_logger(), "EstimateNormals(fine) failed: %s", e.what());
    }


    // Fetch static transforms
    GetTfTransformToMatrix("base_link", "imu_link", mat_imulink2baselink_);
    GetTfTransformToMatrix("motion_link", "base_link", mat_baselink2motionlink_);

    auto imubase_str = matrix4ToString(mat_imulink2baselink_);
    auto basemotion_str = matrix4ToString(mat_baselink2motionlink_);
    RCLCPP_WARN(this->get_logger(), "imulink->baselink: %s", imubase_str.c_str());
    RCLCPP_WARN(this->get_logger(), "baselink->motion_link: %s", basemotion_str.c_str());

    // Optional first localization trial (robust initialization)
    if (init_loc_enable_) {
        auto map_coarse_crop = std::make_shared<open3d::geometry::PointCloud>();
        auto map_fine_crop = std::make_shared<open3d::geometry::PointCloud>();
        auto pcd_scan = std::make_shared<open3d::geometry::PointCloud>();
        auto pcd_scan2map = std::make_shared<open3d::geometry::PointCloud>();
        auto source = std::make_shared<open3d::geometry::PointCloud>();
        auto target = std::make_shared<open3d::geometry::PointCloud>();
        auto OBB_map = std::make_shared<open3d::geometry::OrientedBoundingBox>();
        auto OBB_scan = std::make_shared<open3d::geometry::OrientedBoundingBox>();

        // Fixed OBB extents as in ROS1
        OBB_map->extent_ = Eigen::Vector3d(60, 60, 40);
        OBB_map->color_ = Eigen::Vector3d(1, 0.5, 0);
        OBB_scan->extent_ = Eigen::Vector3d(60, 60, 40);
        OBB_scan->color_ = Eigen::Vector3d(0, 1, 0);

        int count_success = 0;
        double fitness_initial = 0.0;
        auto t_start = std::chrono::high_resolution_clock::now();
        int num_trial = 0;
        Eigen::Matrix4d best_reg_matrix = Eigen::Matrix4d::Identity();
        double best_fitness = 0.0;
        while (rclcpp::ok()) {
            // Timeout guard
            auto t_now = std::chrono::high_resolution_clock::now();
            double elapsed_s = std::chrono::duration_cast<std::chrono::milliseconds>(t_now - t_start).count() / 1000.0;
            if (elapsed_s > init_loc_timeout_s_) {
                RCLCPP_WARN(this->get_logger(), "Initial localization timeout (%.2f s).", elapsed_s);
                // reset the timer and try again
                if (num_trial < num_trial_init_) {
                    num_trial += 1;
                    t_start = std::chrono::high_resolution_clock::now();
                } else {
                    RCLCPP_WARN(this->get_logger(), "Initial localization failed with num_trial %d (fitness %.3f).", num_trial, best_fitness);
                    {
                        std::lock_guard<std::mutex> lk(lock_mat_odom2map_);
                        mat_odom2map_ = best_reg_matrix;
                    }
                    break;
                }
            }

            // Build a scan from the pending queue (accumulate like ROS1)
            lock_scan_.lock();
            if (pcd_scan_cur_->IsEmpty()) { 
                lock_scan_.unlock();
                open3d::utility::LogInfo("wait for pcd_scan_cur_");
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
                continue;
            }
            *pcd_scan = *pcd_scan_cur_;
            lock_scan_.unlock();

            // Update estimate under mutex
            {
                std::lock_guard<std::mutex> lk(lock_mat_odom2map_);

                // Snapshot transforms
                Eigen::Matrix4d mat_baselink2odom_cur = mat_baselink2odom_;
                Eigen::Matrix4d mat_baselink2map_cur = mat_baselink2map_;

                // Prepare OBBs around current estimates
                OBB_map->center_ = mat_baselink2map_cur.block<3,1>(0,3);
                OBB_map->R_ = mat_baselink2map_cur.block<3,3>(0,0);
                OBB_scan->center_ = mat_baselink2odom_cur.block<3,1>(0,3);
                OBB_scan->R_ = mat_baselink2odom_cur.block<3,3>(0,0);

                *map_fine_crop = *pcd_map_fine_->Crop(*OBB_map);

                // Initial registration matrix starts from current odom->map estimate
                Eigen::Matrix4d reg_matrix = Eigen::Matrix4d::Identity();
                reg_matrix = mat_odom2map_;
                // Enforce proper homogeneous form to avoid w=0 issues in Open3D Transform
                // reg_matrix.row(3) = Eigen::Vector4d(0.0, 0.0, 0.0, 1.0);


                *target = *map_fine_crop;
                sensor_msgs::msg::PointCloud2 target_msg;
                // Convert using reusable helper
                open3d_conversions::open3dToRos(*target, target_msg, "map");
                target_msg.header.stamp = this->now();
                pub_submap_->publish(target_msg);
                
                if (target->points_.size() > static_cast<size_t>(maxpoints_target_)) {
                    target = target->RandomDownSample(double(maxpoints_target_) / target->points_.size());
                }

                source = pcd_scan->Crop(*OBB_scan);
                if (source->points_.size() > static_cast<size_t>(maxpoints_source_)) {
                    source = source->RandomDownSample(double(maxpoints_source_) / source->points_.size());
                }

                if (target->points_.empty() || source->points_.empty()) {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Init ICP skipped: empty target (%zu) or source (%zu)", target->points_.size(), source->points_.size());
                    continue;
                }
                // sensor_msgs::msg::PointCloud2 source_msg;
                // open3d_conversions::open3dToRos(*source, source_msg, "map");
                // source_msg.header.stamp = this->now();
                // pub_scan2map_->publish(source_msg);
                // source_msg.header.frame_id = "camera_init";
                // pub_scan_->publish(source_msg);

                // Apply current transform, then multiscale ICP
                source->Transform(reg_matrix);
                *pcd_scan2map = *source;

                sensor_msgs::msg::PointCloud2 source_msg;
                open3d_conversions::open3dToRos(*source, source_msg, "map");
                source_msg.header.stamp = this->now();
                pub_scan2map_->publish(source_msg);
                try {
                    auto multiScale_reg_matrix = pcd_tools::RegistrationMultiScaleIcp(source, target, voxelsize_fine_, 1, voxel_scales_, true);
                    reg_matrix = multiScale_reg_matrix * reg_matrix;
                    source->Transform(multiScale_reg_matrix);
                } catch (const std::exception &e) {
                    RCLCPP_WARN(this->get_logger(), "RegistrationMultiScaleIcp failed: %s", e.what());
                    continue;
                }

                auto eva_result = open3d::pipelines::registration::EvaluateRegistration(*source, *target, std::max(1e-3, voxelsize_fine_ * 3));
                fitness_initial = eva_result.fitness_;
                *pcd_scan2map = *source;

                // Final guard: keep odom->map homogeneous
                reg_matrix.row(3) = Eigen::Vector4d(0.0, 0.0, 0.0, 1.0);
                mat_odom2map_ = reg_matrix;
            }

            if (fitness_initial > threshold_fitness_init_) {
                count_success += 1;
                if (count_success >= 2) {
                    RCLCPP_INFO(this->get_logger(), "Initial localization succeeded (fitness %.3f)", fitness_initial);
                    break;
                }
            } else {
                count_success = 0;
                if (fitness_initial > best_fitness) {
                    best_fitness = fitness_initial;
                    best_reg_matrix = mat_odom2map_;
                }
                RCLCPP_WARN(this->get_logger(), "Initial localization failed (fitness %.3f). Retrying...", fitness_initial);
            }
        }
    }

    // Publish coarse map for visualization
    sensor_msgs::msg::PointCloud2 map_msg;
    // Convert using reusable helper
    open3d_conversions::open3dToRos(*pcd_map_coarse_, map_msg, "map");
    map_msg.header.stamp = this->now();
    pub_map_->publish(map_msg);
    RCLCPP_INFO(this->get_logger(), "Published coarse map: %zu points", pcd_map_coarse_->points_.size());

    RCLCPP_WARN(this->get_logger(), "Localization initialization finished");
}

/**
 * @brief Callback for odometry data from FAST-LIO
 * @param baselink2odom Odometry message containing pose and twist
 * 
 * This callback should:
 * 1. Extract pose information from the odometry message
 * 2. Convert to transformation matrix
 * 3. Update internal state with thread safety
 */
void GlobalLocalization::CallbackBaselink2Odom(const nav_msgs::msg::Odometry::SharedPtr baselink2odom)
{
    // Update timestamp
    {
        std::lock_guard<std::mutex> lk(lock_timestamp_);
        timestamp_odom_ = baselink2odom->header.stamp;
    }

    // Convert odometry pose (imu_link in odom) to Eigen
    const auto &pose = baselink2odom->pose.pose;
    Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
    Eigen::Vector3d t(pose.position.x, pose.position.y, pose.position.z);
    Eigen::Matrix4d mat_imulink2odom = Eigen::Matrix4d::Identity();
    mat_imulink2odom.block<3,3>(0,0) = q.toRotationMatrix();
    mat_imulink2odom.block<3,1>(0,3) = t;

    // baselink = odom * (imulink * baselink)^-1
    mat_baselink2odom_ = mat_imulink2odom * mat_imulink2baselink_.inverse();

    // baselink->map
    mat_baselink2map_ = mat_odom2map_ * mat_baselink2odom_;

    // Publish baselink->map odometry
    nav_msgs::msg::Odometry baselink2map_msg;
    baselink2map_msg.header.frame_id = "map";
    baselink2map_msg.child_frame_id = "base_link";
    baselink2map_msg.header.stamp = baselink2odom->header.stamp;
    setPoseFromMatrix(mat_baselink2map_, baselink2map_msg.pose.pose);
    pub_baselink2map_->publish(baselink2map_msg);

    // Publish odom->map odometry
    nav_msgs::msg::Odometry odom2map_msg;
    odom2map_msg.header.frame_id = "map";
    odom2map_msg.child_frame_id = "odom";
    odom2map_msg.header.stamp = baselink2odom->header.stamp;
    setPoseFromMatrix(mat_odom2map_, odom2map_msg.pose.pose);
    pub_odom2map_->publish(odom2map_msg);

    // Publish TF map->odom
    geometry_msgs::msg::TransformStamped tfm;
    tfm.header.stamp = baselink2odom->header.stamp;
    tfm.header.frame_id = "map";
    tfm.child_frame_id = "odom";
    setTransformFromMatrix(mat_odom2map_, tfm.transform);
    tf_broadcaster_->sendTransform(tfm);

    // Kalman filtering and motion_link publish if initialized
    if (loc_initialized_) {
        Eigen::Matrix4d mat_baselink2map_kalman = Eigen::Matrix4d::Identity();
        if (filter_odom2map_) {
            // Publish odom->map_kalman as separate child frame
            nav_msgs::msg::Odometry odom2map_kalman_msg;
            odom2map_kalman_msg.header.frame_id = "map";
            odom2map_kalman_msg.child_frame_id = "odom_kalman";
            odom2map_kalman_msg.header.stamp = baselink2odom->header.stamp;
            setPoseFromMatrix(mat_odom2map_kalman_, odom2map_kalman_msg.pose.pose);
            pub_odom2map_kalman_->publish(odom2map_kalman_msg);

            kf_baselink_z_.inputLatestNoisyMeasurement((mat_odom2map_kalman_ * mat_baselink2odom_)(2,3));
            mat_baselink2map_kalman = mat_odom2map_kalman_ * mat_baselink2odom_;
        } else {
            kf_baselink_x_.inputLatestNoisyMeasurement(mat_baselink2map_(0,3));
            kf_baselink_y_.inputLatestNoisyMeasurement(mat_baselink2map_(1,3));
            kf_baselink_z_.inputLatestNoisyMeasurement(mat_baselink2map_(2,3));
            mat_baselink2map_kalman = mat_baselink2map_;
        }

        // Override Z with filtered value
        mat_baselink2map_kalman(2,3) = kf_baselink_z_.getLatestEstimatedMeasurement();

        // Publish baselink->map kalman odometry
        nav_msgs::msg::Odometry baselink2map_kalman_msg;
        baselink2map_kalman_msg.header.frame_id = "map";
        baselink2map_kalman_msg.header.stamp = baselink2odom->header.stamp;
        setPoseFromMatrix(mat_baselink2map_kalman, baselink2map_kalman_msg.pose.pose);
        pub_baselink2map_kalman_->publish(baselink2map_kalman_msg);

        // motion_link to map
        Eigen::Matrix4d mat_motionlink2map = mat_baselink2map_kalman * mat_baselink2motionlink_.inverse();
        nav_msgs::msg::Odometry motionlink2map_msg;
        motionlink2map_msg.header.frame_id = "map";
        motionlink2map_msg.header.stamp = baselink2odom->header.stamp;
        setPoseFromMatrix(mat_motionlink2map, motionlink2map_msg.pose.pose);
        pub_motionlink2map_->publish(motionlink2map_msg);

        // Publish TF map->motion_link
        geometry_msgs::msg::TransformStamped tfm_ml;
        tfm_ml.header.stamp = baselink2odom->header.stamp;
        tfm_ml.header.frame_id = "map";
        tfm_ml.child_frame_id = "motion_link";
        setTransformFromMatrix(mat_motionlink2map, tfm_ml.transform);
        tf_broadcaster_->sendTransform(tfm_ml);

        // Publish auxiliary results
        localization_3d_confidence_.data = static_cast<float>(loc_fitness_);
        pub_localization_3d_confidence_->publish(localization_3d_confidence_);
        localization_3d_delay_ms_.data = static_cast<float>((this->now() - baselink2odom->header.stamp).seconds() * 1000.0);
        pub_localization_3d_delay_ms_->publish(localization_3d_delay_ms_);
        localization_3d_.header.frame_id = "map";
        localization_3d_.header.stamp = baselink2odom->header.stamp;
        localization_3d_.pose = motionlink2map_msg.pose.pose;
        pub_localization_3d_->publish(localization_3d_);
    }
}

/**
 * @brief Callback for point cloud data
 * @param scan_in_baselink Point cloud message from LiDAR
 * 
 * This callback should:
 * 1. Convert ROS2 point cloud to Open3D format
 * 2. Add to processing queue
 * 3. Trigger localization if conditions are met
 */
void GlobalLocalization::CallbackScan(const sensor_msgs::msg::PointCloud2::SharedPtr scan_in_baselink)
{
    // Convert to Open3D
    open3d::geometry::PointCloud pcd_received;
    {
        sensor_msgs::PointCloud2ConstIterator<float> it_x(*scan_in_baselink, "x");
        sensor_msgs::PointCloud2ConstIterator<float> it_y(*scan_in_baselink, "y");
        sensor_msgs::PointCloud2ConstIterator<float> it_z(*scan_in_baselink, "z");
        pcd_received.points_.reserve(scan_in_baselink->height * scan_in_baselink->width);
        for (size_t i = 0; i < scan_in_baselink->height * scan_in_baselink->width; ++i, ++it_x, ++it_y, ++it_z) {
            pcd_received.points_.emplace_back(*it_x, *it_y, *it_z);
        }
    }

    // Robust and efficient producer-driven update:
    // - Bound queue size under one mutex
    // - Push newest
    // - If window full, (re)build pcd_scan_cur_ from queue snapshot under the same lock
    {
        // std::lock_guard<std::mutex> lk(lock_scan_);
        lock_scan_.lock();
        const size_t maxsize = queue_maxsize_ > 0 ? static_cast<size_t>(queue_maxsize_) : 0;
        if (maxsize == 0) {
            // No queueing desired; just publish the latest as current
            *pcd_scan_cur_ = std::move(pcd_received);
            return;
        }

        // Ensure there is room for the new element
        while (que_pcd_scan_.size() >= maxsize) {
            que_pcd_scan_.pop();
        }
        que_pcd_scan_.push(std::move(pcd_received));

        // If window reached capacity, build the merged current scan
        if (que_pcd_scan_.size() == maxsize) {
            // Drain to temp, accumulate, then restore queue
            std::vector<open3d::geometry::PointCloud> tmp;
            tmp.reserve(maxsize);
            while (!que_pcd_scan_.empty()) {
                tmp.emplace_back(std::move(que_pcd_scan_.front()));
                que_pcd_scan_.pop();
            }
            pcd_scan_cur_->Clear();
            for (const auto &pc : tmp) {
                *pcd_scan_cur_ += pc;
            }
            for (auto &pc : tmp) {
                que_pcd_scan_.push(std::move(pc));
            }
            lock_scan_.unlock();
            sensor_msgs::msg::PointCloud2 scan_cur_msg;
            open3d_conversions::open3dToRos(*pcd_scan_cur_, scan_cur_msg, "camera_init");
            scan_cur_msg.header.stamp = this->now();
            pub_scan_->publish(scan_cur_msg);
        }
        else{
            lock_scan_.unlock();
        }
    }
}

/**
 * @brief Callback for initial pose estimate
 * @param initialpose Initial pose with covariance
 * 
 * This callback should:
 * 1. Extract pose information
 * 2. Set initial localization estimate
 * 3. Trigger localization initialization
 */
void GlobalLocalization::CallbackInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr initialpose)
{
    if (!(loc_initialized_ && loc_fitness_ > 0.99)) {
        Eigen::Quaterniond q_init(
            initialpose->pose.pose.orientation.w,
            initialpose->pose.pose.orientation.x,
            initialpose->pose.pose.orientation.y,
            initialpose->pose.pose.orientation.z);
        Eigen::Quaterniond q_roll_180(0, 1, 0, 0);
        Eigen::Quaterniond rq = q_roll_180 * q_init;
        mat_initialpose_.setIdentity();
        mat_initialpose_.block<3,3>(0,0) = rq.toRotationMatrix();
        mat_initialpose_.block<3,1>(0,3) = Eigen::Vector3d(
            initialpose->pose.pose.position.x,
            initialpose->pose.pose.position.y,
            initialpose->pose.pose.position.z);
        std::lock_guard<std::mutex> lk(lock_mat_odom2map_);
        mat_odom2map_ = mat_initialpose_;
        RCLCPP_INFO(this->get_logger(), "Updated initial pose (odom->map) from /initialpose");
    }
}

/**
 * @brief Start the localization processing thread
 * 
 * This method should:
 * 1. Create and start the localization thread
 * 2. Set up periodic processing
 */
void GlobalLocalization::StartLoc()
{
    thread_loc_ = std::thread(&GlobalLocalization::Localization, this);
}

/**
 * @brief Main localization processing loop
 * 
 * This method should:
 * 1. Process point cloud data from queue
 * 2. Perform point cloud registration
 * 3. Update transformation matrices
 * 4. Publish results
 * 5. Apply Kalman filtering if enabled
 */
void GlobalLocalization::Localization()
{
    // Initialize odom->map with initial pose (if provided)
    {
        std::lock_guard<std::mutex> lk(lock_mat_odom2map_);
        mat_odom2map_ = mat_initialpose_;
    }

    // Ensure map and transforms are prepared
    LocalizationInitialize();

    // Initialize Kalman filters using current baselink->map
    kf_baselink_x_.KalmanFilterInit(kf_param_x_.size() > 0 ? kf_param_x_[0] : 0.02,
                                    kf_param_x_.size() > 1 ? kf_param_x_[1] : 0.04,
                                    mat_baselink2map_(0,3), 1.0);
    kf_baselink_y_.KalmanFilterInit(kf_param_y_.size() > 0 ? kf_param_y_[0] : 0.02,
                                    kf_param_y_.size() > 1 ? kf_param_y_[1] : 0.04,
                                    mat_baselink2map_(1,3), 1.0);
    kf_baselink_z_.KalmanFilterInit(kf_param_z_.size() > 0 ? kf_param_z_[0] : 0.02,
                                    kf_param_z_.size() > 1 ? kf_param_z_[1] : 0.04,
                                    mat_baselink2map_(2,3), 1.0);
    kalman_filter_odom2map_.KalmanFilterInit(kalman_processVar2_, kalman_estimatedMeasVar2_, mat_baselink2map_(2,3), 1.0);

    loc_initialized_ = true;

    // Prepare containers
    auto coordinate_ori = open3d::geometry::TriangleMesh::CreateCoordinateFrame(2.0);
    auto coordinate_loc = open3d::geometry::TriangleMesh::CreateCoordinateFrame(2.0);
    auto coordinate_OBB_scan = open3d::geometry::TriangleMesh::CreateCoordinateFrame(2.0);
    std::shared_ptr<open3d::geometry::PointCloud> pcd_scan(new open3d::geometry::PointCloud);
    std::shared_ptr<open3d::geometry::PointCloud> pcd_scancrop(new open3d::geometry::PointCloud);
    std::shared_ptr<open3d::geometry::PointCloud> pcd_scan2map(new open3d::geometry::PointCloud);
    std::shared_ptr<open3d::geometry::PointCloud> source(new open3d::geometry::PointCloud);
    std::shared_ptr<open3d::geometry::PointCloud> target(new open3d::geometry::PointCloud);
    std::shared_ptr<open3d::geometry::PointCloud> map_coarse_crop(new open3d::geometry::PointCloud);
    std::shared_ptr<open3d::geometry::PointCloud> map_fine_crop(new open3d::geometry::PointCloud);
    std::shared_ptr<open3d::geometry::PointCloud> pcd_submap(new open3d::geometry::PointCloud);
    std::shared_ptr<open3d::geometry::OrientedBoundingBox> OBB_map(new open3d::geometry::OrientedBoundingBox);
    std::shared_ptr<open3d::geometry::OrientedBoundingBox> OBB_scan(new open3d::geometry::OrientedBoundingBox);
    OBB_map->color_ = Eigen::Vector3d(1, 0.5, 0);
    OBB_map->extent_ = Eigen::Vector3d(60, 60, 40);
    OBB_scan->extent_ = Eigen::Vector3d(60, 60, 40);
    OBB_scan->color_ = Eigen::Vector3d(0, 1, 0);

    // Loop
    auto last_loc_end = std::chrono::high_resolution_clock::now();
    const double period_s = (loc_frequence_ > 0.0) ? (1.0 / loc_frequence_) : 0.0;

    int scan_count = 0;
    double loc_cost = 0.0;
    while (rclcpp::ok()) {
        // Wait for odom timestamp to change
        rclcpp::Time time_current;
        {
            std::lock_guard<std::mutex> lk(lock_timestamp_);
            time_current = timestamp_odom_;
        }

        // Enforce localization frequency (Hz) by sleeping the remaining time in the target period
        auto now_tp = std::chrono::high_resolution_clock::now();
        double elapsed_s = std::chrono::duration_cast<std::chrono::microseconds>(now_tp - last_loc_end).count() / 1e6 + loc_cost / 1000.0;
        if (elapsed_s < period_s) {
            int wait_ms = static_cast<int>((period_s - elapsed_s) * 1000.0);
            if (wait_ms > 0) {
                std::this_thread::sleep_for(std::chrono::milliseconds(wait_ms));
            }
        }

        auto loc_s = std::chrono::high_resolution_clock::now();

        // Build current scan from queue if available
        lock_scan_.lock();
        if (pcd_scan_cur_->IsEmpty()) {
            lock_scan_.unlock();
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            continue;
        } else {
            // Optionally filter odom2map with Kalman
            if (filter_odom2map_) {
                kalman_filter_odom2map_.inputLatestNoisyMeasurement(mat_odom2map_(2,3));
                kalman_filter_odom2map_.inputLatestNoisyMeasurement(mat_odom2map_(2,3));
                mat_odom2map_kalman_ = mat_odom2map_;
                mat_odom2map_kalman_(2,3) = kalman_filter_odom2map_.getLatestEstimatedMeasurement();
            }

            Eigen::Matrix4d mat_baselink2odom_cur = mat_baselink2odom_;
            Eigen::Matrix4d mat_baselink2map_cur = mat_baselink2map_;

            // Use current accumulated scan as snapshot
            *pcd_scan = *pcd_scan_cur_;
            lock_scan_.unlock();

            Eigen::Vector3d cur_loc(mat_baselink2map_cur(0,3), mat_baselink2map_cur(1,3), mat_baselink2map_cur(2,3));
            auto dis_motion = ComputeMotionDis(last_loc_, cur_loc);
            if (dis_motion > dis_updatemap_) {
                last_loc_ = cur_loc;
                OBB_map->center_ = mat_baselink2map_cur.block<3,1>(0,3);
                OBB_map->R_ = mat_baselink2map_cur.block<3,3>(0,0);
                *map_fine_crop = *pcd_map_fine_->Crop(*OBB_map);
            }

            OBB_scan->center_ = mat_baselink2odom_cur.block<3,1>(0,3);
            OBB_scan->R_ = mat_baselink2odom_cur.block<3,3>(0,0);

            auto reg0_s = std::chrono::high_resolution_clock::now();

            Eigen::Matrix4d reg_matrix = Eigen::Matrix4d::Identity();

            lock_mat_odom2map_.lock();
            reg_matrix = mat_odom2map_;

            *target = *map_fine_crop;
            if (target->points_.size() > static_cast<size_t>(maxpoints_target_)) {
                target = target->RandomDownSample(double(maxpoints_target_) / target->points_.size());
            }

            source = pcd_scan->Crop(*OBB_scan);
            
            if (use_cuda_) {
                try {
                    // Prefer CUDA if available; fall back to CPU at runtime on failure
                    open3d::core::Device dev("CUDA:0");

                    // Convert legacy -> tensor and move to device
                    auto t_source = std::make_shared<open3d::t::geometry::PointCloud>(open3d::t::geometry::PointCloud::FromLegacy(*source, open3d::core::Float32));
                    auto t_target = std::make_shared<open3d::t::geometry::PointCloud>(open3d::t::geometry::PointCloud::FromLegacy(*target, open3d::core::Float32));
                    try {
                        *t_source = t_source->To(dev);
                        *t_target = t_target->To(dev);
                    } catch (...) {
                        dev = open3d::core::Device("CPU:0");
                        *t_source = t_source->To(dev);
                        *t_target = t_target->To(dev);
                    }

                    // Optional downsample for speed (match CPU path behavior: downsample source only)
                    if (voxelsize_fine_ > 0.0) {
                        *t_source = t_source->VoxelDownSample(voxelsize_fine_);
                        // keep legacy source consistent for evaluation/visualization
                        try { source = source->VoxelDownSample(voxelsize_fine_); } catch (...) {}
                    }

                    // Call shared CUDA/CPU tensor ICP helper
                    Eigen::Matrix4d T = pcd_tools::RegistrationIcpCUDA(t_source, t_target, voxelsize_fine_, 1, reg_matrix, 30);
                    reg_matrix = T;
                } catch (const std::exception &e) {
                    RCLCPP_WARN(this->get_logger(), "CUDA ICP failed (%s). Falling back to CPU ICP.", e.what());
                    try {
                        if (voxelsize_fine_ > 0.0) {
                            source = source->VoxelDownSample(voxelsize_fine_);
                        }
                    } catch (const std::exception &e2) {
                        RCLCPP_WARN(this->get_logger(), "VoxelDownSample(scan loop, %.6f) failed: %s", voxelsize_fine_, e2.what());
                    }
                    if (source->points_.size() > static_cast<size_t>(maxpoints_source_)) {
                        source = source->RandomDownSample(double(maxpoints_source_) / source->points_.size());
                    }
                    if (!target->points_.empty() && !source->points_.empty()) {
                        try {
                            auto reg_result2 = pcd_tools::RegistrationIcp(source, target, std::max(1e-3, voxelsize_fine_ * 2), reg_matrix, 1);
                            reg_matrix = reg_result2.transformation_ * reg_matrix;
                        } catch (const std::exception &e3) {
                            RCLCPP_WARN(this->get_logger(), "RegistrationIcp failed: %s", e3.what());
                        }
                    }
                }
            } else {
                try {
                    if (voxelsize_fine_ > 0.0) {
                        source = source->VoxelDownSample(voxelsize_fine_);
                    }
                } catch (const std::exception &e) {
                    RCLCPP_WARN(this->get_logger(), "VoxelDownSample(scan loop, %.6f) failed: %s", voxelsize_fine_, e.what());
                }
                if (source->points_.size() > static_cast<size_t>(maxpoints_source_)) {
                    source = source->RandomDownSample(double(maxpoints_source_) / source->points_.size());
                }

                if (!target->points_.empty() && !source->points_.empty()) {
                    try {
                        auto reg_result2 = pcd_tools::RegistrationIcp(source, target, std::max(1e-3, voxelsize_fine_ * 2), reg_matrix, 1);
                        reg_matrix = reg_result2.transformation_ * reg_matrix;
                    } catch (const std::exception &e) {
                        RCLCPP_WARN(this->get_logger(), "RegistrationIcp failed: %s", e.what());
                    }
                }
            }
            auto eva_result2 = open3d::pipelines::registration::EvaluateRegistration(*source, *target, std::max(1e-3, voxelsize_fine_ * 4), reg_matrix);
            loc_fitness_ = eva_result2.fitness_;
            if (loc_fitness_ > threshold_fitness_) {
                mat_odom2map_ = reg_matrix;
            }
            lock_mat_odom2map_.unlock();

            if (save_scan_) {
                // Optional saving of point clouds
                // open3d::io::WritePointCloud("scan_" + std::to_string(scan_count) + ".ply", *pcd_scan);
                // scan_count++;
            }

            auto loc_e = std::chrono::high_resolution_clock::now();
            loc_cost = std::chrono::duration_cast<std::chrono::microseconds>(loc_e - loc_s).count() / 1000.0;
            last_loc_end = loc_e;
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Localization cost: %.3f ms, fitness: %.3f", loc_cost, loc_fitness_);
            *pcd_scan2map = *source;
            pcd_scan2map->Transform(reg_matrix);
            sensor_msgs::msg::PointCloud2 source_msg;
            open3d_conversions::open3dToRos(*pcd_scan2map, source_msg, "map");
            source_msg.header.stamp = this->now();
            pub_scan2map_->publish(source_msg);
        }
    }
}

/**
 * @brief Convert Euler angles to rotation matrix
 * @param euler Euler angles (roll, pitch, yaw) in radians
 * @return 3x3 rotation matrix
 */
Eigen::Matrix3d GlobalLocalization::Euler2Matrix3d(const Eigen::Vector3d euler)
{
    // Input expected in degrees as in ROS1 code
    Eigen::Vector3d euler_rad = euler / 180.0 * PI;
    Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(euler_rad[0], Eigen::Vector3d::UnitX()));
    Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(euler_rad[1], Eigen::Vector3d::UnitY()));
    Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(euler_rad[2], Eigen::Vector3d::UnitZ()));
    Eigen::Matrix3d mat3d = (rollAngle * pitchAngle * yawAngle).toRotationMatrix();
    return mat3d;
}

/**
 * @brief Get TF transform between frames
 * @param frame_id Parent frame
 * @param child_frame_id Child frame
 * @param matrix Output transformation matrix
 * @return True if transform found, false otherwise
 */
bool GlobalLocalization::GetTfTransformToMatrix(std::string frame_id, std::string child_frame_id, Eigen::Matrix4d &matrix)
{
    try {
        auto tfstamped = tf_buffer_->lookupTransform(frame_id, child_frame_id, tf2::TimePointZero, std::chrono::seconds(3));
        const auto &tr = tfstamped.transform.translation;
        const auto &qr = tfstamped.transform.rotation;
        Eigen::Vector3d t(tr.x, tr.y, tr.z);
        Eigen::Quaterniond q(qr.w, qr.x, qr.y, qr.z);
        matrix = Eigen::Matrix4d::Identity();
        matrix.block<3,3>(0,0) = q.toRotationMatrix();
        matrix.block<3,1>(0,3) = t;
        return true;
    } catch (const tf2::TransformException &ex) {
        RCLCPP_ERROR(this->get_logger(), "GetTfTransformToMatrix failed: %s", ex.what());
        return false;
    }
}

/**
 * @brief Compute Euclidean distance between two 3D points
 * @param a First point
 * @param b Second point
 * @return Euclidean distance
 */
double GlobalLocalization::ComputeMotionDis(const Eigen::Vector3d &a, const Eigen::Vector3d &b)
{
    return (a - b).norm();
}

// ===== Main Function =====

void GlobalLocalization::setPoseFromMatrix(const Eigen::Matrix4d &T, geometry_msgs::msg::Pose &pose)
{
    Eigen::Matrix3d R = T.block<3,3>(0,0);
    Eigen::Vector3d t = T.block<3,1>(0,3);
    Eigen::Quaterniond q(R);
    pose.position.x = t.x();
    pose.position.y = t.y();
    pose.position.z = t.z();
    pose.orientation.w = q.w();
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
}

void GlobalLocalization::setTransformFromMatrix(const Eigen::Matrix4d &T, geometry_msgs::msg::Transform &tf)
{
    Eigen::Matrix3d R = T.block<3,3>(0,0);
    Eigen::Vector3d t = T.block<3,1>(0,3);
    Eigen::Quaterniond q(R);
    tf.translation.x = t.x();
    tf.translation.y = t.y();
    tf.translation.z = t.z();
    tf.rotation.w = q.w();
    tf.rotation.x = q.x();
    tf.rotation.y = q.y();
    tf.rotation.z = q.z();
}

std::string GlobalLocalization::matrix4ToString(const Eigen::Matrix4d &T)
{
    std::ostringstream oss;
    oss.setf(std::ios::fixed, std::ios::floatfield);
    oss << std::setprecision(3);
    oss << "[\n"
        << "  " << T(0,0) << ", " << T(0,1) << ", " << T(0,2) << ", " << T(0,3) << "\n"
        << "  " << T(1,0) << ", " << T(1,1) << ", " << T(1,2) << ", " << T(1,3) << "\n"
        << "  " << T(2,0) << ", " << T(2,1) << ", " << T(2,2) << ", " << T(2,3) << "\n"
        << "  " << T(3,0) << ", " << T(3,1) << ", " << T(3,2) << ", " << T(3,3) << "\n]";
    return oss.str();
}

/**
 * @brief Main function for the global localization node
 * @param argc Command line argument count
 * @param argv Command line arguments
 * @return Exit code
 * 
 * This function:
 * 1. Initializes ROS2
 * 2. Creates the GlobalLocalization node
 * 3. Spins the node until shutdown
 */
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GlobalLocalization>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
