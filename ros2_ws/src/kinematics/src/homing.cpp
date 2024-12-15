#include <chrono>
#include <thread>
#include <timr.hpp>
#include "driver.hpp"

std::atomic<bool> running{true};


int main(int argc, char* argv[])
{
    try {
        using scalar_t = double;
        using driver::robotdriver0;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        robotdriver0.homing();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        driver::closePorts();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    } catch (...) {
        std::cerr << "Unknown error occurred" << std::endl;
        return 1;
    }
    
    return 0;
}