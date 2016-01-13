#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

using namespace cv;
using namespace std;

int main() {
  VideoCapture stream(0);
  vector<vector<Point> > contours;

  int GREENHUE_LOWERB = 29; //yellow
  int GREENHUE_UPPERB = 64; //green

  namedWindow("output");
  namedWindow("control");
  createTrackbar("Hue Lower", "control", &GREENHUE_LOWERB, 255);
  createTrackbar("Hue Upper", "control", &GREENHUE_UPPERB, 255);

  while (true) {
    Mat cameraFrame;
    Mat processedFrame;
    Mat outputFrame;

    stream.read(cameraFrame);
    cvtColor(cameraFrame, processedFrame, COLOR_BGR2HSV);
    inRange(processedFrame,
            Scalar(GREENHUE_LOWERB, 0, 0), Scalar(GREENHUE_UPPERB, 255, 255),
            processedFrame);
    findContours(processedFrame, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0, 0));

    int maxContourIdx = -1;
    int maxContourArea = -1;
    for (int i=0; i < contours.size(); ++i) {
      Rect boundRect = boundingRect(contours[i]);
      if (boundRect.area() > maxContourArea) {
        maxContourIdx = i;
        maxContourArea = boundRect.area();
      }
    }

    if (maxContourIdx > 0) {
      Rect boundRect = boundingRect(contours[maxContourIdx]);
      outputFrame = cameraFrame;
      rectangle(outputFrame, boundRect.tl(), boundRect.br(), Scalar(GREENHUE_LOWERB, 255, 255));
    }

    imshow("output", outputFrame);

    if (waitKey(30) >= 0)
      break;
  }
  return 0;
}
