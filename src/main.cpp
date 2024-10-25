#include "../include/VideoProcessor.h"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

int main()
{
    std::vector<std::string> file_options = { "/home/aram/Downloads/yav_test.mp4",
                                              "/home/aram/Videos/Screencasts/output_flight.webm",
                                              "/home/aram/Videos/Screencasts/out_test_1383h_608d.webm" };

    for (std::size_t i = 0; i < file_options.size(); ++i)
    {
        std::cout << i << ": " << std::filesystem::path(file_options[i]).filename() << std::endl;
    }

    std::cout << "Enter the test id: ";
    std::uint16_t id = 0;
    std::cin >> id;

    if (std::cin.fail() || id < 0 || id >= file_options.size())
    {
        id = 0;
        std::cerr << "Invalid input, id will default to: " << id << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(5));
    }

    try
    {
        Air::VideoProcessor processor(file_options[id]);
        processor.processFrames();
    }
    catch (const std::exception& ex)
    {
        std::cerr << ex.what() << std::endl;
    }
}
