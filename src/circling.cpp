#include <iostream>
#include <timr.hpp>
#include <iomanip>
#include <thread>
#include <mutex>
#include <condition_variable>
#include "driver.hpp"


extern "C" {
    #define NON_MATLAB_PARSING
    #define MAX_EXT_API_CONNECTIONS 255
    #define NO_NOT_USE_SHARED_MEMORY
    #include "extApi.h"
}

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
        using scalar_t = float;
        
        int clientID = simxStart((simxChar*)"192.168.0.4", 19997, true, true, 100, 10);
        if (clientID != -1)
        {
            printf("Coppeliasim connected!\n");
            simxStartSimulation(clientID, simx_opmode_blocking);

            int xd_handle{0};
            simxGetObjectHandle(clientID, "/xd", &xd_handle, simx_opmode_blocking);
            int x_handle{0};
            simxGetObjectHandle(clientID, "/x", &x_handle, simx_opmode_blocking);

            std::vector<int> joint_handles{6};  
            simxGetObjectHandle(clientID, "/joint1", &joint_handles[0], simx_opmode_blocking);
            simxGetObjectHandle(clientID, "/joint2", &joint_handles[1], simx_opmode_blocking);
            simxGetObjectHandle(clientID, "/joint3", &joint_handles[2], simx_opmode_blocking);
            simxGetObjectHandle(clientID, "/joint4", &joint_handles[3], simx_opmode_blocking);
            simxGetObjectHandle(clientID, "/joint5", &joint_handles[4], simx_opmode_blocking);
            simxGetObjectHandle(clientID, "/joint6", &joint_handles[5], simx_opmode_blocking);

            
            std::vector<int> test_handles{6};  
            simxGetObjectHandle(clientID, "/test1", &test_handles[0], simx_opmode_blocking);
            simxGetObjectHandle(clientID, "/test2", &test_handles[1], simx_opmode_blocking);
            simxGetObjectHandle(clientID, "/test3", &test_handles[2], simx_opmode_blocking);
            simxGetObjectHandle(clientID, "/test4", &test_handles[3], simx_opmode_blocking);
            simxGetObjectHandle(clientID, "/test5", &test_handles[4], simx_opmode_blocking);
            simxGetObjectHandle(clientID, "/test6", &test_handles[5], simx_opmode_blocking);

            scalar_t frame_x_quat[4];
            scalar_t frame_x_posit[3];
            scalar_t frame_xd_quat[4];
            scalar_t frame_xd_posit[3];

            using namespace timr;
            std::array<scalar_t, 6> init_joint_pos;     
            for (int i=0; i<6; ++i){

                simxGetJointPosition(clientID, joint_handles[i], &init_joint_pos[i], simx_opmode_blocking);
            }

            SerialManipulator<scalar_t, 6> robot("../src/robot.json", init_joint_pos);

            auto runkinematics = [&running, &robot, &clientID, &xd_handle, &joint_handles, &test_handles] {
                while (running)
                {     
              
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
                    simxSetObjectPosition(clientID, xd_handle, -1, xd.translation().array3().data(), simx_opmode_oneshot);
                    simxSetObjectQuaternion(clientID, xd_handle, -1, xd.rotation().vrep_array4().data(), simx_opmode_oneshot);
                    std::array<scalar_t, 6> joint_pos = robot.joint_positions();

                    for (int i=0; i<6; ++i){
                        simxSetJointTargetPosition(clientID, joint_handles[i], joint_pos[i], simx_opmode_oneshot);
                    }

                    const auto query = robot.data();

                    for (int i=0; i < 6; ++i){
                        auto t = query.joint_poses[i].translation();
                        auto r = query.joint_poses[i].rotation();
                        simxSetObjectPosition(clientID, test_handles[i], -1, t.array3().data(), simx_opmode_oneshot);
                        simxSetObjectQuaternion(clientID, test_handles[i], -1, r.vrep_array4().data(), simx_opmode_oneshot);

                    }
                }
            };
            
            auto rundrive = [&running, &robotdriver0, &robot] {
                while (running)
                {   
                    std::array<scalar_t, 6> joint_positions = robot.data().joint_positions;
                    std::array<scalar_t, 6> joint_velocities = robot.data().joint_velocities;
                    for (auto& v:joint_velocities) {
                        v *= 1000;
                    }   
                    robotdriver0.position_control(joint_positions, joint_velocities, std::array<uint8_t, 6>{0xFF,0xFF,0xFF,0xFF,0xFF,0xFF});
                    std::this_thread::sleep_for(std::chrono::milliseconds(5));
                }
            };

            std::thread kinematics_thread(runkinematics);
            std::thread drive_thread(rundrive);

            kinematics_thread.join();
            drive_thread.join();

        }
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    } catch (...) {
        std::cerr << "Unknown error occurred" << std::endl;
        return 1;
    }
    
    return 0;
}
