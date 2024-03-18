#include <opencv2/opencv.hpp>
#include <iostream>

int main() {
    cv::VideoCapture cap(0);  // 0 for the default camera
    if (!cap.isOpened()) {
        std::cerr << "Error: Unable to open the camera" << std::endl;
        return -1;
    }

    cv::Mat frame;
    while (true) {
        cap.read(frame);  // Capture a frame from the camera

        if (frame.empty()) {
            std::cerr << "Error: Blank frame grabbed" << std::endl;
            break;
        }

        cv::imshow("Camera Feed", frame);  // Display the captured frame

        if (cv::waitKey(1) == 27) {  // Press 'Esc' to exit
            break;
        }
    }

    cap.release();  // Release the camera
    cv::destroyAllWindows();  // Close all OpenCV windows

    return 0;
}

