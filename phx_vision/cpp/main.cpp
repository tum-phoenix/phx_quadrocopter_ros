#include <QCoreApplication>
#include <opencv2/opencv.hpp>

#include <MainRunning.h>

void displayImage ()
{
    //cv::Mat inputImage = cv::imread("/home/mykytadenysov/Pictures/Orange-Juice-Plain.jpg");
    cv::Mat inputImage = cv::imread("/home/mykyta_denysov/Pictures/DSC_4447.jpg");
    if(!inputImage.empty()) cv::imshow("Display Image", inputImage);
}

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);
    Main_Running();
    return a.exec();
}
