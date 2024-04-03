#include <iostream>
#include <opencv2/opencv.hpp>
#include <wiringPi.h>
#include <wiringSerial.h>

#define MAX_SPEED 80

using namespace cv;
using namespace std;
int robot;
int motors[4] = {30, 30, 30, 30};
float kp, ki, kd;

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

int pid(int error) {
  kp = 0.5; // max speed = kp * max error -> 80 / (roi.width * 0.5)
  ki = 0;
  kd = 1; // kd = kp * 10

  int offset = (int) (kp * error + ki * error_sum + kd * (error - prev_error));
  prev_error = error;
  error_sum += error;
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
  for(int i = 0; i < 2; i++){
    Rect roi(x, y + i * height, width, height);
    rois.push_back(roi); 
  }

  while (true) {

    cap.read(frame); // Capture a frame from the camera
    if (frame.empty()) {
      cerr << "Error: Blank frame grabbed" << endl;
      continue;
    }


    // delay for 10 milliseconds
    delay(100);


    // Mat edges;
    // cvtColor(frame, edges, COLOR_BGR2GRAY);  // Convert frame to grayscale
    // Canny(edges, edges, 50, 150);  // Apply Canny edge detection
    //
    Mat filtered;
    // inRange(frame(roi), Scalar(0, 0, 0), Scalar(180, 132, 107), filtered);
    cvtColor(frame(rois[0]), filtered, COLOR_BGR2GRAY);
    inRange(filtered, 0, 70, filtered);
    

    Mat result = frame.clone();

    // rectangle(result, roi, Scalar(0, 255, 0), 1);

    vector<Point> largestContour;
    int error = 0;
    
    // if no contours were defected 
    if (!detectContours(filtered, largestContour)) {
      error = 0;

      delay(100);

      sprintf(command, "#Barrrf %03d %03d %03d %03d",
              (int)motors[0],  // right front
              (int)motors[1],  // right back
              (int)motors[2],  // left front
              (int)motors[3]); // left back

      serialPrintf(robot, command);

      delay(100);
      continue;
    }

    Moments m = moments(largestContour, false);
    Point2f center(m.m10 / m.m00, m.m01 / m.m00);

    error = (int) center.x - rois[0].width / 2;


    int offset = pid(error);

    circle(result(rois[0]), center, 2, Scalar(255,255,255), -1);
    drawContours(result(rois[0]), vector<vector<Point>>{largestContour}, -1,
      Scalar(0, 255, 0), 1);

    // cout << "error: " << error << " offset: " << offset << endl;

    int right = motors[0] - offset;
    int left = motors[2] + offset;


    
      sprintf(command, "#Bafffr %03d %03d %03d %03d", 
        clamp(right), // right front
        clamp(right), // right back
        clamp(left), // left front
        clamp(left)); // left back
      serialPrintf(robot, command);

    // else if (left < 0 ) {
    //   left *= -2;
    //   sprintf(command, "#Baffrf %03d %03d %03d %03d", 
    //     clamp(right), // right front
    //     clamp(right), // right back
    //     clamp(left), // left front
    //     clamp(left)); // left back
    //   serialPrintf(robot, command);
    // }

    // else {
    //   sprintf(command, "#Bafffr %03d %03d %03d %03d", 
    //     clamp(right), // right front
    //     clamp(right), // right back
    //     clamp(left), // left front
    //     clamp(left)); // left back
    // serialPrintf(robot, command);
    // }


    // sprintf(command, "#Bafffr %03d %03d %03d %03d", 
    //     clamp(right), // right front
    //     clamp(right), // right back
    //     clamp(left), // left front
    //     clamp(left)); // left back
    // serialPrintf(robot, "#Bafffr 020 020 020 020");



    // printf("robot: %d\terror:%d\toffset: %d\t%s\n", robot, error, offset, command);

    // imshow("Camera Feed", result); // Display the captured frame
    // if (waitKey(1) == 27) { // Press 'Esc' to exit
    //   break;
    // }
  }

  // cap.release();       // Release the camera
  // destroyAllWindows(); // Close all OpenCV windows
  return 0;
}
