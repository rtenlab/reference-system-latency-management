#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <sys/time.h>

// For ROS2RTF
#include <unistd.h>
#include <sys/types.h>
#include <errno.h>
#include <sys/syscall.h>
#include <mutex>

#include "trace_picas/trace.hpp"

#include "rclcpp/rclcpp.hpp"
//#include "std_msgs/msg/string.hpp"
#include "test_interfaces/msg/test_string.hpp"

#include <callback.hpp>
#include <chain.hpp>
#include <thread.hpp>
#include <state.hpp>
#include <mpccontroller.hpp>
#include <executor.hpp>

using std::placeholders::_1;
//std::mutex mtx;

#define gettid() syscall(__NR_gettid)

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "PID: %ld run in ROS2.", gettid());

    int n_cpus = 4;
    executor ex(n_cpus);

    auto chain1_2 = std::make_shared<Chain>(0);
    auto c0_cb1 = std::make_shared<Callback>(CallbackType::TIMER, timeval{0, 200000}, 0, 1, 22, "front_lidar_driver", "", "c0_cb1", 1000);
    auto c0_cb2 = std::make_shared<Callback>(CallbackType::SUBSCRIPTION, timeval{0, 0}, 0, 2, 24, "points_transformer_front", "c0_cb1", "c0_cb2", 65536);
    auto c0_cb3 = std::make_shared<Callback>(CallbackType::SUBSCRIPTION, timeval{0, 0}, 0, 3, 27, 0, "point_cloud_fusion", "c0_cb2", "c0_cb3", 65536, true);
    auto c0_cb4 = std::make_shared<Callback>(CallbackType::SUBSCRIPTION, timeval{0, 0}, 0, 4, 27, 0, "ray_ground_filter", "c0_cb3", "c0_cb4", 65536);
    auto c0_cb5 = std::make_shared<Callback>(CallbackType::SUBSCRIPTION, timeval{0, 0}, 0, 5, 28, 0, "euclidean_cluster_detector", "c0_cb4", "c0_cb5", 65536);
    auto c0_cb6 = std::make_shared<Callback>(CallbackType::SUBSCRIPTION, timeval{0, 0}, 0, 6, 29, 0, "object_collision_estimator", "c0_cb5", "c0_cb6", 65536);
    auto c0_cb7 = std::make_shared<Callback>(CallbackType::SUBSCRIPTION, timeval{0, 0}, 0, 7, 30, 0, "behavior_planner_input_0", "c0_cb6", "", 65536); // postfix _0: unique node name needed
    auto c0_cb8 = std::make_shared<Callback>(CallbackType::SUBSCRIPTION, timeval{0, 0}, 0, 4, 2, 1, "voxel_grid_downsampler", "c0_cb3", "c0_cb8", 65536);
    auto c0_cb9 = std::make_shared<Callback>(CallbackType::SUBSCRIPTION, timeval{0, 0}, 0, 5, 3, 1, "ndt_localizer_input", "c0_cb8", "", 32768);
    c0_cb1->setChain(chain1_2);
    c0_cb2->setChain(chain1_2);
    c0_cb3->setChain(chain1_2);
    c0_cb4->setChain(chain1_2);
    c0_cb5->setChain(chain1_2);
    c0_cb6->setChain(chain1_2);
    c0_cb7->setChain(chain1_2);
    c0_cb8->setChain(chain1_2);
    c0_cb9->setChain(chain1_2);
    chain1_2->setLatencyTarget({0, 200000}, 0, true);
    chain1_2->setLatencyTarget({0, 0}, 1, false);
    chain1_2->addCallback(c0_cb1);
    chain1_2->addCallback(c0_cb2);
    chain1_2->addCallback(c0_cb3);
    chain1_2->addCallback(c0_cb4);
    chain1_2->addCallback(c0_cb5);
    chain1_2->addCallback(c0_cb6);
    chain1_2->addCallback(c0_cb7);
    chain1_2->addCallback(c0_cb8);
    chain1_2->addCallback(c0_cb9);
    chain1_2->setPeriod({0, 200000});
    chain1_2->setDeadline({0, 200000});
    std::vector<int> chain_criticalities;
    chain_criticalities.push_back(2);
    chain_criticalities.push_back(0);
    chain1_2->setPriorities(chain_criticalities);

    auto chain3_4 = std::make_shared<Chain>(1);
    auto c1_cb1 = std::make_shared<Callback>(CallbackType::TIMER, timeval{0, 100000}, 1, 1, 32, 0, "behavior_planner_timer", "", "c1_cb1", 1000, true);
    auto c1_cb2 = std::make_shared<Callback>(CallbackType::SUBSCRIPTION, timeval{0, 0}, 1, 2, 35, 0, "mpc_controller", "c1_cb1", "c1_cb2", 65536);
    auto c1_cb3 = std::make_shared<Callback>(CallbackType::SUBSCRIPTION, timeval{0, 0}, 1, 3, 34, 0, "vehicle_interface", "c1_cb2", "c1_cb3", 65536);
    auto c1_cb4 = std::make_shared<Callback>(CallbackType::SUBSCRIPTION, timeval{0, 0}, 1, 4, 25, 0, "vehicle_dbw_system", "c1_cb3", "", 8192);
    auto c1_cb5 = std::make_shared<Callback>(CallbackType::SUBSCRIPTION, timeval{0, 0}, 1, 2, 33, 1, "vehicle_interface_input", "c1_cb1", "", 32768);
    c1_cb1->setChain(chain3_4);
    c1_cb2->setChain(chain3_4);
    c1_cb3->setChain(chain3_4);
    c1_cb4->setChain(chain3_4);
    c1_cb5->setChain(chain3_4);
    chain3_4->setLatencyTarget({0, 100000}, 0, true);
    chain3_4->setLatencyTarget({0, 0}, 1, false);
    chain3_4->addCallback(c1_cb1);
    chain3_4->addCallback(c1_cb2);
    chain3_4->addCallback(c1_cb3);
    chain3_4->addCallback(c1_cb4);
    chain3_4->addCallback(c1_cb5);
    chain3_4->setPeriod({0, 100000});
    chain3_4->setDeadline({0, 100000});
    std::vector<int> chain_criticalities2;
    chain_criticalities2.push_back(3);
    chain_criticalities2.push_back(0);
    chain3_4->setPriorities(chain_criticalities2);

    auto chain5_6 = std::make_shared<Chain>(2);
    auto c2_cb1 = std::make_shared<Callback>(CallbackType::TIMER, timeval{0, 60000}, 2, 1, 6, 0, "visualizer", "", "c2_cb1", 1000, true);
    auto c2_cb2 = std::make_shared<Callback>(CallbackType::SUBSCRIPTION, timeval{0, 0}, 2, 2, 8, 0, "lanelet_2_global_planner", "c2_cb1", "c2_cb2", 65536);
    auto c2_cb3 = std::make_shared<Callback>(CallbackType::SUBSCRIPTION, timeval{0, 0}, 2, 3, 14, 0, "behavior_planner_input_1", "c2_cb2", "", 1000);
    auto c2_cb4 = std::make_shared<Callback>(CallbackType::SUBSCRIPTION, timeval{0, 0}, 2, 2, 10, 1, "lanelet_2_map_loader_input", "c2_cb1", "", 32768);
    c2_cb1->setChain(chain5_6);
    c2_cb2->setChain(chain5_6);
    c2_cb3->setChain(chain5_6);
    c2_cb4->setChain(chain5_6);
    chain5_6->setLatencyTarget({0, 0}, 0, false);
    chain5_6->setLatencyTarget({0, 0}, 1, false);
    chain5_6->addCallback(c2_cb1);
    chain5_6->addCallback(c2_cb2);
    chain5_6->addCallback(c2_cb3);
    chain5_6->addCallback(c2_cb4);
    chain5_6->setPeriod({0, 60000});
    chain5_6->setDeadline({0, 60000});
    std::vector<int> chain_criticalities3;
    chain_criticalities3.push_back(0);
    chain_criticalities3.push_back(0);
    chain5_6->setPriorities(chain_criticalities3);

    auto chain7 = std::make_shared<Chain>(3);
    auto c3_cb1 = std::make_shared<Callback>(CallbackType::TIMER, timeval{0, 200000}, 3, 1, 23, "rear_lidar_driver", "", "c3_cb1", 1000);
    auto c3_cb2 = std::make_shared<Callback>(CallbackType::SUBSCRIPTION, timeval{0, 0}, 3, 2, 25, "points_transformer_rear", "c3_cb1", "c3_cb2", 65536);
    auto c3_cb3 = std::make_shared<Callback>(CallbackType::SUBSCRIPTION, timeval{0, 0}, 3, 3, 26, "pointcloud_fusion_input", "c3_cb2", "", 32768);
    c3_cb1->setChain(chain7);
    c3_cb2->setChain(chain7);
    c3_cb3->setChain(chain7);
    chain7->setLatencyTarget({0, 200000}, 0, true);
    chain7->addCallback(c3_cb1);
    chain7->addCallback(c3_cb2);
    chain7->addCallback(c3_cb3);
    chain7->setPeriod({0, 200000});
    chain7->setDeadline({0, 200000});
    std::vector<int> chain_criticalities4;
    chain_criticalities4.push_back(1);
    chain7->setPriorities(chain_criticalities4);

    auto chain8_9_10 = std::make_shared<Chain>(4);
    auto c4_cb1 = std::make_shared<Callback>(CallbackType::TIMER, timeval{0, 100000}, 4, 1, 9, "lanelet_2_map", "", "c4_cb1", 1000);
    auto c4_cb2 = std::make_shared<Callback>(CallbackType::SUBSCRIPTION, timeval{0, 0}, 4, 2, 11, 0, "lanelet_2_map_loader", "c4_cb1", "c4_cb2", 65536, true);
    auto c4_cb3 = std::make_shared<Callback>(CallbackType::SUBSCRIPTION, timeval{0, 0}, 4, 3, 12, 0, "parking_planner", "c4_cb2", "c4_cb3", 65536);
    auto c4_cb4 = std::make_shared<Callback>(CallbackType::SUBSCRIPTION, timeval{0, 0}, 4, 4, 17, 0, "behavior_planner_input_2", "c4_cb3", "", 1000);
    auto c4_cb5 = std::make_shared<Callback>(CallbackType::SUBSCRIPTION, timeval{0, 0}, 4, 3, 13, 1, "lane_planner", "c4_cb2", "c4_cb5", 65536);
    auto c4_cb6 = std::make_shared<Callback>(CallbackType::SUBSCRIPTION, timeval{0, 0}, 4, 4, 16, 1, "behavior_planner_input_3", "c4_cb5", "", 1000);
    auto c4_cb7 = std::make_shared<Callback>(CallbackType::SUBSCRIPTION, timeval{0, 0}, 4, 3, 15, 2, "behavior_planner_input_4", "c4_cb2", "", 1000);
    c4_cb1->setChain(chain8_9_10);
    c4_cb2->setChain(chain8_9_10);
    c4_cb3->setChain(chain8_9_10);
    c4_cb4->setChain(chain8_9_10);
    c4_cb5->setChain(chain8_9_10);
    c4_cb6->setChain(chain8_9_10);
    c4_cb7->setChain(chain8_9_10);
    chain8_9_10->setLatencyTarget({0, 0}, 0, false);
    chain8_9_10->setLatencyTarget({0, 0}, 1, false);
    chain8_9_10->setLatencyTarget({0, 0}, 2, false);
    chain8_9_10->addCallback(c4_cb1);
    chain8_9_10->addCallback(c4_cb2);
    chain8_9_10->addCallback(c4_cb3);
    chain8_9_10->addCallback(c4_cb4);
    chain8_9_10->addCallback(c4_cb5);
    chain8_9_10->addCallback(c4_cb6);
    chain8_9_10->addCallback(c4_cb7);
    chain8_9_10->setPeriod({0, 100000});
    chain8_9_10->setDeadline({0, 100000});
    std::vector<int> chain_criticalities5;
    chain_criticalities5.push_back(0);
    chain_criticalities5.push_back(0);
    chain_criticalities5.push_back(0);
    chain8_9_10->setPriorities(chain_criticalities5);

    auto chain11_12 = std::make_shared<Chain>(5);
    auto c5_cb1 = std::make_shared<Callback>(CallbackType::TIMER, timeval{0, 120000}, 5, 1, 1, "pointcloud_map", "", "c5_cb1", 1000);
    auto c5_cb2 = std::make_shared<Callback>(CallbackType::SUBSCRIPTION, timeval{0, 0}, 5, 2, 3, "pointcloud_map_loader", "c5_cb1", "c5_cb2", 65536);
    auto c5_cb3 = std::make_shared<Callback>(CallbackType::SUBSCRIPTION, timeval{0, 0}, 5, 3, 5, 0, "ndt_localizer", "c5_cb2", "c5_cb3", 65536, true);
    auto c5_cb4 = std::make_shared<Callback>(CallbackType::SUBSCRIPTION, timeval{0, 0}, 5, 4, 7, 0, "lanelet_2_global_planner_input", "c5_cb3", "", 32768);
    auto c5_cb5 = std::make_shared<Callback>(CallbackType::SUBSCRIPTION, timeval{0, 0}, 5, 4, 18, 1, "behavior_planner_input_5", "c5_cb3", "", 1000);
    c5_cb1->setChain(chain11_12);
    c5_cb2->setChain(chain11_12);
    c5_cb3->setChain(chain11_12);
    c5_cb4->setChain(chain11_12);
    c5_cb5->setChain(chain11_12);
    chain11_12->setLatencyTarget({0, 0}, 0, false);
    chain11_12->setLatencyTarget({0, 0}, 1, false);
    chain11_12->addCallback(c5_cb1);
    chain11_12->addCallback(c5_cb2);
    chain11_12->addCallback(c5_cb3);
    chain11_12->addCallback(c5_cb4);
    chain11_12->addCallback(c5_cb5);
    chain11_12->setPeriod({0, 120000});
    chain11_12->setDeadline({0, 120000});
    std::vector<int> chain_criticalities6;
    chain_criticalities6.push_back(0);
    chain_criticalities6.push_back(0);
    chain11_12->setPriorities(chain_criticalities6);

    auto chain13 = std::make_shared<Chain>(6);
    auto c6_cb1 = std::make_shared<Callback>(CallbackType::TIMER, timeval{0, 25000}, 6, 1, 19, "euclidean_cluster_settings", "", "c6_cb1", 1000);
    auto c6_cb2 = std::make_shared<Callback>(CallbackType::SUBSCRIPTION, timeval{0, 0}, 6, 2, 20, "euclidean_cluster_detector", "c6_cb1", "c6_cb2", 65536);
    auto c6_cb3 = std::make_shared<Callback>(CallbackType::SUBSCRIPTION, timeval{0, 0}, 6, 3, 21, "intersection_output", "c6_cb2", "", 16384);
    c6_cb1->setChain(chain13);
    c6_cb2->setChain(chain13);
    c6_cb3->setChain(chain13);
    chain13->setLatencyTarget({0, 0}, 0, false);
    chain13->addCallback(c6_cb1);
    chain13->addCallback(c6_cb2);
    chain13->addCallback(c6_cb3);
    chain13->setPeriod({0, 25000});
    chain13->setDeadline({0, 25000});
    std::vector<int> chain_criticalities7;
    chain_criticalities7.push_back(0);
    chain13->setPriorities(chain_criticalities7);




/*test with adding additional BE chains*/
    // auto chain14_15 = std::make_shared<Chain>(7);
    // auto c7_cb1 = std::make_shared<Callback>(CallbackType::TIMER, timeval{0, 120000}, 5, 1, 1, "pointcloud_map", "", "c7_cb1", 128000);
    // auto c7_cb2 = std::make_shared<Callback>(CallbackType::SUBSCRIPTION, timeval{0, 0}, 5, 2, 3, "pointcloud_map_loader", "c7_cb1", "c7_cb2", 128000);
    // auto c7_cb3 = std::make_shared<Callback>(CallbackType::SUBSCRIPTION, timeval{0, 0}, 5, 3, 5, 0, "ndt_localizer", "c7_cb2", "c7_cb3", 128000, true);
    // auto c7_cb4 = std::make_shared<Callback>(CallbackType::SUBSCRIPTION, timeval{0, 0}, 5, 4, 7, 0, "lanelet_2_global_planner_input", "c7_cb3", "", 128000);
    // auto c7_cb5 = std::make_shared<Callback>(CallbackType::SUBSCRIPTION, timeval{0, 0}, 5, 4, 18, 1, "behavior_planner_input_5", "c7_cb3", "", 128000);
    // c7_cb1->setChain(chain14_15);
    // c7_cb2->setChain(chain14_15);
    // c7_cb3->setChain(chain14_15);
    // c7_cb4->setChain(chain14_15);
    // c7_cb5->setChain(chain14_15);
    // chain14_15->setLatencyTarget({0, 0}, 0, false);
    // chain14_15->setLatencyTarget({0, 0}, 1, false);
    // chain14_15->addCallback(c7_cb1);
    // chain14_15->addCallback(c7_cb2);
    // chain14_15->addCallback(c7_cb3);
    // chain14_15->addCallback(c7_cb4);
    // chain14_15->addCallback(c7_cb5);
    // chain14_15->setPeriod({0, 120000});
    // chain14_15->setDeadline({0, 120000});
    // std::vector<int> chain_criticalities8;
    // chain_criticalities8.push_back(0);
    // chain_criticalities8.push_back(0);
    // chain14_15->setPriorities(chain_criticalities8);


/* end */

    ex.add_chain(chain1_2);
    ex.add_chain(chain3_4);
    ex.add_chain(chain5_6);
    ex.add_chain(chain7);
    ex.add_chain(chain8_9_10);
    ex.add_chain(chain11_12);
    ex.add_chain(chain13);
    //ex.add_chain(chain14_15);
    ex.set_callback_priorities();
    std::cout << std::endl
              << "Printing chains from executor" << std::endl;
    // ex.print_callbacks();
    ex.print_chains_and_callbacks();

    // Profile callback execution time
    ex.add_all_callbacks_to_all_threads();
    ex.disable_callback_priority();
    ex.start();
#ifdef LATENCY_MGMT
    std::this_thread::sleep_for(std::chrono::seconds(15));
    // Pause timer callbacks and wait for a second to finish remaining callbacks
    ex.pause();
    std::this_thread::sleep_for(std::chrono::seconds(1));
    ex.remove_all_callbacks_from_all_threads();

    MPCController mpc;
    mpc.assign_executor(&ex);
    std::thread mpc_thread(&MPCController::run, &mpc);
    struct sched_param param2;
    param2.sched_priority = 98;
    int policy = SCHED_FIFO;
    int ret = pthread_setschedparam(mpc_thread.native_handle(), policy, &param2);

    if (ret != 0)
    {
        std::cerr << "Failed to set mpc controller thread to RT Prio 99: " << strerror(errno) << std::endl;
    }
    else
    {
        std::cout << "Successfully set mpc controller thread to RT Prio 99." << std::endl;
    }

    mpc_thread.join();
#endif // latency_mgmt
    ex.join();

    rclcpp::shutdown();
    return 0;
}
