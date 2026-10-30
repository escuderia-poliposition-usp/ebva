#include <algorithm>
#include "ebva_pointcloud_fusion/map_path.hpp"
#include <chrono>
#include <ctime>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "message_filters/subscriber.hpp"
#include "message_filters/synchronizer.hpp"
#include "message_filters/sync_policies/approximate_time.hpp"
#include "laser_geometry/laser_geometry.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

using LaserScan    = sensor_msgs::msg::LaserScan;
using PointCloud2  = sensor_msgs::msg::PointCloud2;
using ApproxPolicy = message_filters::sync_policies::ApproximateTime<LaserScan, PointCloud2>;

class FusionNode : public rclcpp::Node
{
public:
  FusionNode()
  : Node("fusion_node"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    this->declare_parameter<std::string>("scan_topic",   "/scan");
    this->declare_parameter<std::string>("cloud_topic",  "/zed/zed_node/point_cloud/cloud_registered");
    this->declare_parameter<std::string>("target_frame", "zed_left_camera_frame");
    this->declare_parameter<int>("queue_size", 10);
    this->declare_parameter<std::string>("map_frame", "map");
    this->declare_parameter<double>("voxel_leaf_size", 0.05);
    this->declare_parameter<double>("map_height_tolerance", 0.05);
    this->declare_parameter<double>("map_publish_rate", 2.0);
    this->declare_parameter<bool>("accumulate", true);
    this->declare_parameter<std::string>("map_save_path", "");
    // When false the node is LiDAR-only: it does not subscribe to cloud_topic and
    // /fused_cloud is just the projected scan (Phase 1 with IMU odometry, no ZED).
    this->declare_parameter<bool>("use_cloud", true);

    const auto scan_topic  = this->get_parameter("scan_topic").as_string();
    const auto cloud_topic = this->get_parameter("cloud_topic").as_string();
    target_frame_          = this->get_parameter("target_frame").as_string();
    const auto queue_size  = static_cast<uint32_t>(this->get_parameter("queue_size").as_int());
    map_frame_             = this->get_parameter("map_frame").as_string();
    voxel_leaf_size_       = this->get_parameter("voxel_leaf_size").as_double();
    map_height_tolerance_  = this->get_parameter("map_height_tolerance").as_double();
    accumulate_            = this->get_parameter("accumulate").as_bool();
    map_save_path_         = this->get_parameter("map_save_path").as_string();
    use_cloud_             = this->get_parameter("use_cloud").as_bool();
    const auto map_publish_rate = this->get_parameter("map_publish_rate").as_double();

    RCLCPP_INFO(this->get_logger(), "scan_topic          : %s", scan_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "use_cloud           : %s", use_cloud_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "cloud_topic         : %s",
      use_cloud_ ? cloud_topic.c_str() : "(disabled, LiDAR-only)");
    RCLCPP_INFO(this->get_logger(), "target_frame        : %s", target_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "queue_size          : %u", queue_size);
    RCLCPP_INFO(this->get_logger(), "accumulate          : %s", accumulate_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "map_frame           : %s", map_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "voxel_leaf_size     : %.3f", voxel_leaf_size_);
    RCLCPP_INFO(this->get_logger(), "map_height_tolerance: %.3f", map_height_tolerance_);
    RCLCPP_INFO(this->get_logger(), "map_publish_rate    : %.2f", map_publish_rate);
    RCLCPP_INFO(this->get_logger(), "map_save_path       : %s",
      map_save_path_.empty() ? "(disabled)" : map_save_path_.c_str());

    auto qos = rclcpp::QoS(queue_size).best_effort();

    if (use_cloud_) {
      scan_sub_.subscribe(this, scan_topic,  qos.get_rmw_qos_profile());
      cloud_sub_.subscribe(this, cloud_topic, qos.get_rmw_qos_profile());

      sync_ = std::make_shared<message_filters::Synchronizer<ApproxPolicy>>(
        ApproxPolicy(queue_size), scan_sub_, cloud_sub_);
      sync_->registerCallback(
        std::bind(&FusionNode::syncCallback, this,
          std::placeholders::_1, std::placeholders::_2));
    } else {
      // LiDAR-only: no cloud to synchronize against, so subscribe to the scan directly.
      scan_only_sub_ = this->create_subscription<LaserScan>(
        scan_topic, qos,
        std::bind(&FusionNode::scanCallback, this, std::placeholders::_1));
    }

    fused_pub_ = this->create_publisher<PointCloud2>("/fused_cloud", 10);

    if (accumulate_) {
      fused_map_pub_ = this->create_publisher<PointCloud2>("/fused_map", 10);
      const auto period = std::chrono::duration<double>(1.0 / std::max(map_publish_rate, 0.1));
      map_timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&FusionNode::publishMap, this));

      save_map_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "~/save_map",
        std::bind(&FusionNode::saveMapService, this,
          std::placeholders::_1, std::placeholders::_2));
    }
  }

  // Save the accumulated map on shutdown. The make_shared<FusionNode>() temporary
  // in main() is destroyed right after rclcpp::spin() returns (on Ctrl+C) and
  // before rclcpp::shutdown(), so the context is still up and logging works here.
  ~FusionNode()
  {
    if (accumulate_ && !map_save_path_.empty()) {
      saveMap(map_save_path_);
    }
  }

private:
  // ZED path: synchronized scan + cloud.
  void syncCallback(
    const LaserScan::ConstSharedPtr & scan_msg,
    const PointCloud2::ConstSharedPtr & cloud_msg)
  {
    processFrame(scan_msg, cloud_msg);
  }

  // LiDAR-only path (use_cloud=false): scan with no ZED cloud.
  void scanCallback(const LaserScan::ConstSharedPtr & scan_msg)
  {
    processFrame(scan_msg, nullptr);
  }

  // Shared processing. cloud_msg may be null (LiDAR-only): then /fused_cloud is just
  // the projected scan and nothing is concatenated.
  void processFrame(
    const LaserScan::ConstSharedPtr & scan_msg,
    const PointCloud2::ConstSharedPtr & cloud_msg)
  {
    // Project LaserScan → PointCloud2 in the scan's own frame
    PointCloud2 scan_cloud;
    try {
      projector_.projectLaser(*scan_msg, scan_cloud);
    } catch (const std::exception & e) {
      RCLCPP_WARN(this->get_logger(), "LaserProjection failed: %s", e.what());
      return;
    }

    // Transform scan cloud into target_frame
    PointCloud2 scan_cloud_tf;
    try {
      auto transform = tf_buffer_.lookupTransform(
        target_frame_, scan_cloud.header.frame_id,
        rclcpp::Time(scan_cloud.header.stamp),
        rclcpp::Duration::from_seconds(0.1));
      tf2::doTransform(scan_cloud, scan_cloud_tf, transform);
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "TF lookup failed: %s", e.what());
      return;
    }

    // Convert scan to PCL XYZ; concatenate the ZED cloud only if one was provided.
    pcl::PointCloud<pcl::PointXYZ> pcl_scan, pcl_fused;
    pcl::fromROSMsg(scan_cloud_tf, pcl_scan);
    pcl_fused = pcl_scan;

    std::size_t zed_size = 0;
    if (cloud_msg) {
      pcl::PointCloud<pcl::PointXYZ> pcl_zed;
      pcl::fromROSMsg(*cloud_msg, pcl_zed);
      // Remove NaN points from ZED cloud
      std::vector<int> indices;
      pcl::removeNaNFromPointCloud(pcl_zed, pcl_zed, indices);
      pcl_fused += pcl_zed;
      zed_size = pcl_zed.size();
    }

    PointCloud2 fused_msg;
    pcl::toROSMsg(pcl_fused, fused_msg);
    fused_msg.header.stamp    = cloud_msg ? cloud_msg->header.stamp
                                          : scan_msg->header.stamp;
    fused_msg.header.frame_id = target_frame_;

    fused_pub_->publish(fused_msg);

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "Fused cloud: %zu LiDAR + %zu ZED points",
      pcl_scan.size(), zed_size);

    if (accumulate_) {
      if (pcl_scan.empty()) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
          "pcl_scan empty, no valid laser height reference; skipping accumulate() for this frame");
      } else {
        double z_sum = 0.0;
        for (const auto & pt : pcl_scan.points) {
          z_sum += pt.z;
        }
        const double z_ref = z_sum / static_cast<double>(pcl_scan.size());
        accumulate(fused_msg, z_ref);
      }
    }
  }

  // Slice the fused cloud to the LiDAR's scan plane (z_ref ± map_height_tolerance_)
  // while still in target_frame (body-relative, tilt-invariant), then transform to
  // map_frame_ and flatten Z so the accumulated map is a true 2D top-down map.
  void accumulate(const PointCloud2 & fused_msg, double z_ref)
  {
    pcl::PointCloud<pcl::PointXYZ> pcl_full, pcl_sliced;
    pcl::fromROSMsg(fused_msg, pcl_full);

    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(pcl_full.makeShared());
    pass.setFilterFieldName("z");
    pass.setFilterLimits(
      static_cast<float>(z_ref - map_height_tolerance_),
      static_cast<float>(z_ref + map_height_tolerance_));
    pass.filter(pcl_sliced);

    PointCloud2 sliced_msg;
    pcl::toROSMsg(pcl_sliced, sliced_msg);
    sliced_msg.header = fused_msg.header;

    PointCloud2 fused_map;
    try {
      auto transform = tf_buffer_.lookupTransform(
        map_frame_, sliced_msg.header.frame_id,
        rclcpp::Time(sliced_msg.header.stamp),
        rclcpp::Duration::from_seconds(0.1));
      tf2::doTransform(sliced_msg, fused_map, transform);
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "Map TF lookup failed, skipping frame: %s", e.what());
      return;
    }

    pcl::PointCloud<pcl::PointXYZ> pcl_frame;
    pcl::fromROSMsg(fused_map, pcl_frame);
    for (auto & pt : pcl_frame.points) {
      pt.z = 0.0f;
    }
    map_cloud_ += pcl_frame;

    // Bound the accumulated map's density with a voxel grid.
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setInputCloud(map_cloud_.makeShared());
    voxel.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
    voxel.filter(map_cloud_);
  }

  // Single-threaded executor: this runs only when syncCallback is not, so the
  // shared map_cloud_ needs no mutex.
  void publishMap()
  {
    if (map_cloud_.empty()) {
      return;
    }

    PointCloud2 map_msg;
    pcl::toROSMsg(map_cloud_, map_msg);
    map_msg.header.stamp    = this->now();
    map_msg.header.frame_id = map_frame_;
    fused_map_pub_->publish(map_msg);

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
      "Accumulated map: %zu points in '%s'",
      map_cloud_.size(), map_frame_.c_str());
  }

  // Persist map_cloud_ to disk. Format chosen by extension: .ply -> PLY, else PCD.
  bool saveMap(const std::string & path)
  {
    if (map_cloud_.empty()) {
      RCLCPP_WARN(this->get_logger(), "Map is empty, nothing to save");
      return false;
    }

    std::string out;
    try {
      out = ebva_pointcloud_fusion::reserve_map_path(path);
      int rc;
      if (out.size() >= 4 && out.compare(out.size() - 4, 4, ".ply") == 0) {
        rc = pcl::io::savePLYFileBinary(out, map_cloud_);
      } else {
        rc = pcl::io::savePCDFileBinary(out, map_cloud_);
      }
      if (rc == 0) {
        RCLCPP_INFO(this->get_logger(), "Saved map (%zu points) to '%s'",
          map_cloud_.size(), out.c_str());
        return true;
      } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to save map to '%s' (rc=%d)",
          out.c_str(), rc);
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "Exception while saving map to '%s': %s",
        out.c_str(), e.what());
    }
    return false;
  }

  void saveMapService(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    if (map_save_path_.empty()) {
      res->success = false;
      res->message = "map_save_path parameter is not set";
      return;
    }
    if (map_cloud_.empty()) {
      res->success = false;
      res->message = "map is empty, nothing to save";
      return;
    }
    res->success = saveMap(map_save_path_);
    res->message = res->success ?
      "map saved (" + std::to_string(map_cloud_.size()) + " points)" : "map save failed";
  }

  message_filters::Subscriber<LaserScan>   scan_sub_;
  message_filters::Subscriber<PointCloud2> cloud_sub_;
  std::shared_ptr<message_filters::Synchronizer<ApproxPolicy>> sync_;
  rclcpp::Subscription<LaserScan>::SharedPtr scan_only_sub_;

  laser_geometry::LaserProjection           projector_;
  tf2_ros::Buffer                           tf_buffer_;
  tf2_ros::TransformListener                tf_listener_;
  rclcpp::Publisher<PointCloud2>::SharedPtr fused_pub_;
  rclcpp::Publisher<PointCloud2>::SharedPtr fused_map_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_map_srv_;
  rclcpp::TimerBase::SharedPtr              map_timer_;
  pcl::PointCloud<pcl::PointXYZ>            map_cloud_;
  std::string                               target_frame_;
  std::string                               map_frame_;
  std::string                               map_save_path_;
  double                                    voxel_leaf_size_{0.05};
  double                                    map_height_tolerance_{0.05};
  bool                                      accumulate_{true};
  bool                                      use_cloud_{true};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FusionNode>());
  rclcpp::shutdown();
  return 0;
}
