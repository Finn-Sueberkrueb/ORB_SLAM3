#include <signal.h>
#include <stdlib.h>
#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <ctime>
#include <sstream>
#include <condition_variable>
#include <opencv2/opencv.hpp>
#include <System.h>

using namespace std;

bool b_continue_session;

void exit_loop_handler(int s){
    cout << "Finishing session" << endl;
    b_continue_session = false;
}

int main(int argc, char **argv) {

    if (argc < 4) {
        cerr << endl
             << "Usage: ./mono_video path_to_vocabulary path_to_settings path_to_video_file (trajectory_file_name)"
             << endl;
        return 1;
    }

    string file_name;
    if (argc == 5) {
        file_name = string(argv[argc - 1]);
    }

    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = exit_loop_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);
    b_continue_session = true;

    // Open the video file
    string video_file = string(argv[3]);
    cv::VideoCapture cap(video_file);
    if (!cap.isOpened()) {
        cerr << "Unable to open video file: " << video_file << endl;
        return 1;
    }

    int width_img = (int) cap.get(cv::CAP_PROP_FRAME_WIDTH);
    int height_img = (int) cap.get(cv::CAP_PROP_FRAME_HEIGHT);
    double fps = cap.get(cv::CAP_PROP_FPS);
    double timestamp = 0;

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::MONOCULAR, false, 0, file_name);
    float imageScale = SLAM.GetImageScale();

    cv::Mat im;
    double t_resize = 0.f;
    double t_track = 0.f;

    while (!SLAM.isShutDown() && b_continue_session) {
        cap >> im;
        if (im.empty()) {
            break; // End of video
        }

        timestamp += 1.0 / fps;

#ifdef REGISTER_TIMES
#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point time_Start_Process = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point time_Start_Process = std::chrono::monotonic_clock::now();
#endif
#endif

        if (imageScale != 1.f) {
#ifdef REGISTER_TIMES
#ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t_Start_Resize = std::chrono::steady_clock::now();
#else
            std::chrono::monotonic_clock::time_point t_Start_Resize = std::chrono::monotonic_clock::now();
#endif
#endif
            int width = im.cols * imageScale;
            int height = im.rows * imageScale;
            cv::resize(im, im, cv::Size(width, height));
#ifdef REGISTER_TIMES
#ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t_End_Resize = std::chrono::steady_clock::now();
#else
            std::chrono::monotonic_clock::time_point t_End_Resize = std::chrono::monotonic_clock::now();
#endif
            t_resize = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(t_End_Resize - t_Start_Resize).count();
            SLAM.InsertResizeTime(t_resize);
#endif
        }

#ifdef REGISTER_TIMES
#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t_Start_Track = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t_Start_Track = std::chrono::monotonic_clock::now();
#endif
#endif
        // Process frame
        SLAM.TrackMonocular(im, timestamp);
#ifdef REGISTER_TIMES
#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t_End_Track = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t_End_Track = std::chrono::monotonic_clock::now();
#endif
        t_track = t_resize + std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(t_End_Track - t_Start_Track).count();
        SLAM.InsertTrackTime(t_track);
#endif
    }

    cout << "System shutdown!\n";
    SLAM.Shutdown();

    return 0;
}