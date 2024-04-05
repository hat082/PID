
#include <iostream>
#include <opencv2/opencv.hpp>
#include <wiringPi.h>
#include <wiringSerial.h>

using namespace cv;
using namespace std;

int robot;
int motors[4] = {30, 30, 30, 30};
float kp, ki, kd;

const Mat &inputFrame preprocessFrame(const Mat &inputFrame) {
  // Mat edges;
  // cvtColor(frame, edges, COLOR_BGR2GRAY);  // Convert frame to grayscale
  // Canny(edges, edges, 50, 150);  // Apply Canny edge detection
  //
  Mat filtered;
  // inRange(frame(roi), Scalar(0, 0, 0), Scalar(180, 132, 107), filtered);
  cvtColor(frame(rois[0]), filtered, COLOR_BGR2GRAY);
  inRange(filtered, 0, 70, filtered);
  return filtered
}

bool detectContours(const Mat &inputFrame, vector<Point> &largestContour) {
  vector<vector<Point>> contours;
  findContours(inputFrame, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

  if (contours.empty()) {
    // No contours found, return false
    return false;
  }

  int largest_contour_index = 0;
  double max_area = 0.0;
  for (size_t i = 0; i < contours.size(); i++) {
    double area = contourArea(contours[i]);
    if (area > max_area) {
      max_area = area;
      largest_contour_index = i; 
    }
  }

  largestContour = contours[largest_contour_index];
  return true; // Contours found
}

int clamp(int input) {
  if (input < 0) {
    return 0;
  } else if (input > MAX_SPEED) {
    return MAX_SPEED;
  }
}

int error_sum = 0;
int prev_error = 0;

int pid(int* errors) {
  kp = 0.5; // max speed = kp * max error -> 80 / (roi.width * 0.5)
  ki = 0;
  kd = 1; // kd = kp * 10

  // int offset = (int) (kp * error + ki * error_sum + kd * (error - prev_error));
  // prev_error = error;
  // error_sum += error;
  return offset;
}

int main() {
  wiringPiSetupGpio();
  robot = serialOpen("/dev/ttyAMA0", 57600);
  printf("%d\n", robot);
  char command[50];

  VideoCapture cap(0); // 0 for the default camera
  if (!cap.isOpened()) {
    cerr << "Error: Unable to open the camera" << endl;
    return -1;
  }

  Mat frame;
  cap.read(frame); // Capture a frame from the camera
  cout << "cols: " << frame.cols << " rows: " << frame.rows << endl;

  int x = frame.cols * 0;
  int y = frame.rows * 0.3;
  int width = frame.cols * 1;
  int height = frame.rows * 0.08;

  vector<Rect> rois;
  Rect processed_frame(x, y + (i + 1) * height, width, height);
  int errors[3]; // (error1, error2, existPink)

  for(int i = 0; i < 2; i++){
    Rect roi(x, y + (i + 1) * height, width, height);
    rois.push_back(roi); 
  }
  

  while(true) {
    cap.read(frame); // Capture a frame from the camera
    if (frame.empty()) {
      cerr << "Error: Blank frame grabbed" << endl;
      continue;
    }

    // delay for 10 milliseconds
    delay(100);

    // preprocess both frames and return the offset for both points
    for (int i = 0; i < 2; i++) {
      errors[2] = 0;
      // check if there is pink in the frame
      if (existPink) {
        // stop the robot
        // write the existPink bool in the command array
        errors[0] = 0;
        errors[1] = 0;
        errors[2] = 1;
        break;
      }
      
      // process the frame
      processed_frame = preprocessFrame(rois[i]);

      // find the largest contour in the black line
      // if no contours then offset is set as -10000
      if (detectContours(processed_frame, largestContour) == false) {
        errors[i] = -10000;
      }
      else {
        // calculate the offset
        Point center = moments(largestContour, false).m10 / moments(largestContour, false).m00;
        errors[i] = center.x - (rois[i].x + rois[i].width / 2);
        // cout << "offset: " << errors[i] << endl;
      } 
    }
    
    // using errors calculate offset
    // if the robot sees pink
    if (errors[2] != 1) {
      offset = pid(errors);
    }
    // send commands to the robot
  }
}
