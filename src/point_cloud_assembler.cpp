// Copyright 2018 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <string>
#include "laser_assembler/base_assembler.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_sensor_msgs/tf2_sensor_msgs.h"
#include "tf2/convert.h"
#include "tf2/transform_datatypes.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "filters/filter_chain.hpp"



namespace laser_assembler
{

/**
 * \brief Maintains a history of incremental point clouds (usually from laser
 * scans) and generates a point cloud upon request \todo Clean up the doxygen
 * part of this header params
 *  * (Several params are inherited from BaseAssemblerSrv)
 */
class PointCloudAssembler : public BaseAssembler<sensor_msgs::msg::PointCloud>
{
public:
  explicit PointCloudAssembler(rclcpp::Node::SharedPtr node)
  : BaseAssembler<sensor_msgs::msg::PointCloud>("max_clouds", node),
      filter_chain_("sensor_msgs::msg::PointCloud")
  {

        // ***** Set Laser Projection Method *****
    n_->get_parameter_or("ignore_laser_skew", ignore_laser_skew_,
      true);

    // configure the filter chain from the parameter server
    filter_chain_.configure("filters", n_->get_node_logging_interface(), n_->get_node_parameters_interface());

    // Have different callbacks, depending on whether or not we want to ignore
    // laser skews.
    if (ignore_laser_skew_) {
      start("scan");
    } else {
      start();
      skew_scan_sub_ = n_->create_subscription<sensor_msgs::msg::PointCloud2>(
        "scan",
        rclcpp::QoS(1),
        std::bind(&PointCloudAssembler::scanCallback, this, std::placeholders::_1)
      );
    }

  }

  ~PointCloudAssembler() {}

  unsigned int GetPointsInScan(const sensor_msgs::msg::PointCloud & scan)
  {
    return scan.points.size();
  }

  void ConvertToCloud(
    const std::string & fixed_frame_id,
    const sensor_msgs::msg::PointCloud & scan_in,
    sensor_msgs::msg::PointCloud & cloud_out)
  {
    // Doesnt exist anymore : <
    //tf_->transformPointCloud(fixed_frame_id, scan_in, cloud_out);
    sensor_msgs::msg::PointCloud2 cloud_in;
    auto temp1 = sensor_msgs::convertPointCloudToPointCloud2(scan_in, cloud_in);
    sensor_msgs::msg::PointCloud2 cloud_out_2;
    auto transformStamped = buffer_.lookupTransform(fixed_frame_id, scan_in.header.frame_id, rclcpp::Time(0));
    tf2::doTransform(cloud_in, cloud_out_2, transformStamped);
    temp1 = sensor_msgs::convertPointCloud2ToPointCloud(cloud_out_2, cloud_out);
  }

    void
  scanCallback(std::shared_ptr<sensor_msgs::msg::PointCloud2> laser_scan)
  {
    if (!ignore_laser_skew_) 
    {
      // rclcpp::Duration cur_tolerance = rclcpp::Duration(
      //   laser_scan->time_increment * laser_scan->ranges.size());
      // if (cur_tolerance > max_tolerance_) {
      //   RCLCPP_DEBUG(g_logger, "Upping tf tolerance from [%.4fs] to [%.4fs]",
      //     max_tolerance_.nanoseconds() / 1e+9,
      //     cur_tolerance.nanoseconds() / 1e+9);
      //   assert(tf_filter_);
      //   tf_filter_->setTolerance(cur_tolerance);
      //   max_tolerance_ = cur_tolerance;
      // }
      sensor_msgs::msg::PointCloud laser_scan_pc;
      sensor_msgs::convertPointCloud2ToPointCloud(*laser_scan, laser_scan_pc);
      tf_filter_->add(std::make_shared<sensor_msgs::msg::PointCloud>(laser_scan_pc));
    }
  }

private:

  bool ignore_laser_skew_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr skew_scan_sub_;
  //rclcpp::Duration max_tolerance_ = rclcpp::Duration(0, 0);   // The longest tolerance we've needed on a scan so far

  filters::FilterChain<sensor_msgs::msg::PointCloud> filter_chain_;
};

}  // namespace laser_assembler

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;

  node_options.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("point_cloud_assembler", node_options);
  laser_assembler::PointCloudAssembler pc_assembler(node);
  pc_assembler.start("cloud");
  rclcpp::spin(node);

  return 0;
}
