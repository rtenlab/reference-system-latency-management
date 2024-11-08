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

//#define USE_INTRA_PROCESS_COMMS false
#define USE_INTRA_PROCESS_COMMS true // Note: if this doesn't work with ROS2 Galactic, update Galactic version to 0.9.3 (20221208) or higher

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "PID: %ld run in ROS2.", gettid());

    int n_cpus = 1;
    executor ex(n_cpus);
    // Note: timer callbacks start immediately after callback definitions.
    auto chain1_2 = std::make_shared<Chain>(0);
    auto c0_cb1 = std::make_shared<Callback>(CallbackType::TIMER, timeval{0, 200000}, 0, 1, 22, "front_lidar_driver", "", "cb1", 1000);
    auto c0_cb2 = std::make_shared<Callback>(CallbackType::SUBSCRIPTION, timeval{0, 0}, 0, 2, 24, "points_transformer_front", "cb1", "cb2", 65536);
    auto c0_cb3 = std::make_shared<Callback>(CallbackType::SUBSCRIPTION, timeval{0, 0}, 0, 3, 27, 0, "point_cloud_fusion", "cb2", "cb3", 65536, true);
    auto c0_cb4 = std::make_shared<Callback>(CallbackType::SUBSCRIPTION, timeval{0, 0}, 0, 4, 27, 0, "ray_ground_filter", "cb3", "cb4", 65536);
    auto c0_cb5 = std::make_shared<Callback>(CallbackType::SUBSCRIPTION, timeval{0, 0}, 0, 5, 28, 0, "euclidean_cluster_detector", "cb4", "cb5", 65536);
    auto c0_cb6 = std::make_shared<Callback>(CallbackType::SUBSCRIPTION, timeval{0, 0}, 0, 6, 29, 0, "object_collision_estimator", "cb6", "cb7", 65536);
    auto c0_cb7 = std::make_shared<Callback>(CallbackType::SUBSCRIPTION, timeval{0, 0}, 0, 7, 30, 0, "behavior_planner_input", "cb7", "cb8", 65536);
    auto c0_cb8 = std::make_shared<Callback>(CallbackType::SUBSCRIPTION, timeval{0, 0}, 0, 4, 2, 1, "voxel_grid_downsampler", "cb3", "cb8", 65536);
    auto c0_cb9 = std::make_shared<Callback>(CallbackType::SUBSCRIPTION, timeval{0, 0}, 0, 5, 3, 1, "ndt_localizer_input", "cb8", "cb9", 32768);
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
    chain_criticalities.push_back(1);
    chain_criticalities.push_back(0);
    chain1_2->setPriorities(chain_criticalities);

    ex.add_chain(chain1_2);
    //ex.add_chain(chain3_4);
    //ex.add_chain(chain5_6);
    //ex.add_chain(chain7);
    //ex.add_chain(chain8_9_10);
    //ex.add_chain(chain11_12);
    //ex.add_chain(chain13);

    ex.set_callback_priorities();
    std::cout << std::endl
              << "Printing chains from executor" << std::endl;
    // ex.print_callbacks();
    ex.print_chains_and_callbacks();
    
    // Profile callback execution time
    ex.add_all_callbacks_to_all_threads();
    ex.start();
    std::this_thread::sleep_for(std::chrono::seconds(3));
    // Stop timer callbacks and wait for a second to finish remaining callbacks
    ex.stop();
    std::this_thread::sleep_for(std::chrono::seconds(1));

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
    // Restart timer callbacks
    ex.start();
    ex.join();

    rclcpp::shutdown();
    return 0;
}
