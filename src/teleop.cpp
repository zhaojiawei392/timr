#include <iostream>
#include <timr.hpp>
#include <iomanip>
#include <thread>
#include <mutex>
#include <condition_variable>
#include "driver.hpp"

std::atomic<bool> running{true};

void cleanupAndExit() {
    // First, signal threads to stop
    running = false;
    
    // Give threads a moment to complete their current operations
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    driver::robotdriver0.emergency_stop();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    // Then close the port
    driver::closePorts();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    std::exit(0);
}

int main(int argc, char* argv[])
{
    // Set up signal handlers for multiple signals
    std::signal(SIGINT, [](int signal) { cleanupAndExit(); });   // Ctrl+C
    std::signal(SIGTERM, [](int signal) { cleanupAndExit(); });  // Termination request
    std::signal(SIGSEGV, [](int signal) {                        // Segmentation fault
        std::cerr << "\nSegmentation fault detected" << std::endl;
        cleanupAndExit();
    });

    try {
        using driver::robotdriver0;
        using namespace timr;

        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        using scalar_t = double;
        auto init_joint_positions = robotdriver0.get_joint_positions();
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));

        for (auto x:init_joint_positions) {
            std::cout << x << "\n";
        }

        SerialManipulator<scalar_t, 6> robot("../src/robot.json", init_joint_positions);
        
        auto target_period = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::duration<scalar_t>(robot.config().sampling_time_sec));
        auto runkinematics = [&running, &robot, target_period] {
            while (true)
            {         
                // using namespace std::chrono;
                // auto loop_start = high_resolution_clock::now();

                static size_t i = 0;
                static const Posef x_init = robot.end_pose();
                static const Tranf t_init = Tranf(0.2663,0,0.2985);
                static const Rotf r_init = x_init.rotation();
                constexpr scalar_t radius = 0.05;
                constexpr scalar_t rad_speed = 0.00003;
                Tranf td = t_init + Tranf(0, radius * cos(rad_speed*i), radius * sin(rad_speed*i));
                ++i;
                Posef xd = Posef(r_init, td);
                robot.update(xd);

                // auto loop_end = high_resolution_clock::now();
                // auto loop_duration = duration_cast<microseconds>(loop_end - loop_start);
                // if (loop_duration < target_period) {
                //     std::this_thread::sleep_for(target_period - loop_duration);
                // }
            }
        };

        auto rundrive = [&running, &robotdriver0, &robot, target_period] {
            while (true)
            {   
                // using namespace std::chrono;
                // auto loop_start = high_resolution_clock::now();
                std::array<scalar_t, 6> joint_positions = robot.data().joint_positions;
                std::array<scalar_t, 6> joint_velocities = robot.data().joint_velocities;
                for (auto& v:joint_velocities) {
                    v *= 1000;
                }
                robotdriver0.position_control(joint_positions, joint_velocities, std::array<uint8_t, 6>{0xFF,0xFF,0xFF,0xFF,0xFF,0xFF});
                std::this_thread::sleep_for(std::chrono::microseconds(5));

                // auto loop_end = high_resolution_clock::now();
                // auto loop_duration = duration_cast<microseconds>(loop_end - loop_start);
                // if (loop_duration < target_period) {
                //     std::this_thread::sleep_for(std::chrono::microseconds(target_period - loop_duration));
                // }
            }
        };

        std::thread kinematics_thread(runkinematics);
        std::thread drive_thread(rundrive);

        kinematics_thread.join();
        drive_thread.join();

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    } catch (...) {
        std::cerr << "Unknown error occurred" << std::endl;
        return 1;
    }
    
    return 0;
}
