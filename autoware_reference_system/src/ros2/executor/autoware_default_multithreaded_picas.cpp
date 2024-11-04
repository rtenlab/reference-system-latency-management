// Copyright 2021 Apex.AI, Inc.
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

#include "rclcpp/rclcpp.hpp"

#include "reference_system/system/systems.hpp"

#include "autoware_reference_system/autoware_system_builder.hpp"
#include "autoware_reference_system/system/timing/benchmark.hpp"
#include "autoware_reference_system/system/timing/default.hpp"

template<typename Base, typename T> inline bool instanceof(const T) {
   return std::is_base_of<Base, T>::value;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  //int n_threads = std::thread::hardware_concurrency();
  int n_threads = 4; // default
  if (argc >= 2) {
  	n_threads = atoi(argv[1]);
  }

  using TimeConfig = nodes::timing::Default;
  // uncomment for benchmarking
  // using TimeConfig = nodes::timing::BenchmarkCPUUsage;
  // set_benchmark_mode(true);

  auto nodes = create_autoware_nodes<RclcppSystem, TimeConfig>();

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), n_threads); 

  executor.enable_callback_priority();
  executor.cpus = {1, 3, 5, 7, 9, 11, 13, 15};
  //executor.rt_attr.sched_policy = SCHED_FIFO;
  //executor.rt_attr.sched_priority = 80;
  
  // SCHED_DEADLINE (by default, Linux doesn't allow SCHED_DEADLINE with CPU affinity)
  //executor.cpus.clear();
  //executor.rt_attr.sched_policy = SCHED_DEADLINE;
  //executor.rt_attr.sched_priority = 0;
  //executor.rt_attr.sched_runtime = 100 * 1000 * 1000; // 100ms budget
  //executor.rt_attr.sched_period  = 100 * 1000 * 1000; // 100ms period
  //executor.rt_attr.sched_deadline= 100 * 1000 * 1000; // 100ms deadline

  for (auto & node : nodes) {
    //if (node->get_name() == std::string("FrontLidarDriver")
    //   || node->get_name() == std::string("PointsTransformerFront"))
    executor.add_node(node);
    
    // affinity testing
    //unsigned long affinity = (1 << 0) | (1 << 1) | (1 << 2) | (1 << 3);
    //unsigned long affinity = (1 << 0) | (1 << 1);
    unsigned long affinity = (1 << 0);
    if (node->get_name() == std::string("FrontLidarDriver")
      || node->get_name() == std::string("RearLidarDriver") 
      || node->get_name() == std::string("PointCloudMap") 
      || node->get_name() == std::string("Visualizer") 
      || node->get_name() == std::string("Lanelet2Map") 
      || node->get_name() == std::string("EuclideanClusterSettings") 
      ) {
      std::cout << "Sensor affinity" << std::endl;
      static_cast<nodes::rclcpp_system::Sensor*>(node.get())->set_affinity_timer(affinity);
    }
    else if (node->get_name() == std::string("PointsTransformerFront")
      || node->get_name() == std::string("PointsTransformerRear") 
      || node->get_name() == std::string("VoxelGridDownsampler") 
      || node->get_name() == std::string("PointCloudMapLoader") 
      || node->get_name() == std::string("RayGroundFilter") 
      || node->get_name() == std::string("ObjectCollisionEstimator") 
      || node->get_name() == std::string("MPCController") 
      || node->get_name() == std::string("ParkingPlanner") 
      || node->get_name() == std::string("LanePlanner") 
      ) {
      std::cout << "Transform affinity" << std::endl;
      //unsigned long affinity = (1 << 4);
      static_cast<nodes::rclcpp_system::Transform*>(node.get())->set_affinity_subscription(affinity);
    }
    else if (node->get_name() == std::string("PointCloudFusion")
      || node->get_name() == std::string("NDTLocalizer") 
      || node->get_name() == std::string("VehicleInterface") 
      || node->get_name() == std::string("Lanelet2GlobalPlanner") 
      || node->get_name() == std::string("Lanelet2MapLoader") 
      ) {
      std::cout << "Fusion affinity" << std::endl;
      static_cast<nodes::rclcpp_system::Fusion*>(node.get())->set_affinity_subscriptions(0, affinity);
      static_cast<nodes::rclcpp_system::Fusion*>(node.get())->set_affinity_subscriptions(1, affinity);
    }
    else if (node->get_name() == std::string("BehaviorPlanner")) {
      std::cout << "Cyclic affinity" << std::endl;
      static_cast<nodes::rclcpp_system::Cyclic*>(node.get())->set_affinity_timer(affinity);
      static_cast<nodes::rclcpp_system::Cyclic*>(node.get())->set_affinity_subscriptions(0, affinity);
      static_cast<nodes::rclcpp_system::Cyclic*>(node.get())->set_affinity_subscriptions(1, affinity);
      static_cast<nodes::rclcpp_system::Cyclic*>(node.get())->set_affinity_subscriptions(2, affinity);
      static_cast<nodes::rclcpp_system::Cyclic*>(node.get())->set_affinity_subscriptions(3, affinity);
      static_cast<nodes::rclcpp_system::Cyclic*>(node.get())->set_affinity_subscriptions(4, affinity);
      static_cast<nodes::rclcpp_system::Cyclic*>(node.get())->set_affinity_subscriptions(5, affinity);
    }
    else if (node->get_name() == std::string("EuclideanClusterDetector")) {
      std::cout << "Intersection affinity" << std::endl;
      static_cast<nodes::rclcpp_system::Intersection*>(node.get())->set_affinity_subscriptions(0, affinity);
      static_cast<nodes::rclcpp_system::Intersection*>(node.get())->set_affinity_subscriptions(1, affinity);
    }
    else if (node->get_name() == std::string("VehicleDBWSystem")
      || node->get_name() == std::string("IntersectionOutput") 
      ) {
      std::cout << "Command affinity" << std::endl;
      static_cast<nodes::rclcpp_system::Command*>(node.get())->set_affinity_subscription(affinity);
    }
    else {
      std::cout << "UNKNOWN: " << node->get_name() << std::endl;
    }
  }
  executor.spin();

  nodes.clear();
  rclcpp::shutdown();

  return 0;
}
