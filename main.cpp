#include <config.h>
#include <csignal>
#include <tdtcamera.h>
#include <thread>
#include "detector.h"
#include <opencv2/videoio.hpp>
#include <signal.h>
#include <thread>
#include "../ksconnect/ksconnect.h"
#include "../usart/usart.h"
#include "../light/light.h"

using namespace tdttoolkit;
using namespace std;
using namespace cv;

void ProgOnImage()
{
    Detector *detector = new Detector;
    ksconnect::ksDetect ksDetect_;
    light::lightDetect lightDetect_;

    tdtusart::Send_Struct_t sendStruct;
    engineer_tool::ReceiveMessage receiveMessage;
    tdtusart::Recv_Struct_t recvStruct;
    tdtusart::RealUsart realUsart;
    tdtusart::Usart &usart = realUsart;
    tdtcamera::Camera *camera;
    camera = new tdtcamera::HikvisionCam("../../config/robot_param.yaml");

    tdtcamera::TImage frame;

    string xml_path = "/home/shelly/TDT2022sorth/video/best.xml";

    detector->init(xml_path, 0.25, 0.55);

    VideoCapture capture(2);
    VideoWriter vw;
    capture.set(CAP_PROP_AUTO_EXPOSURE, 1);
    capture.set(CAP_PROP_BRIGHTNESS, 0); // CAP_PROP_EXPOSURE166
    capture.set(CAP_PROP_FRAME_WIDTH, 1280);
    capture.set(CAP_PROP_FRAME_HEIGHT, 720);
    capture.set(CAP_PROP_EXPOSURE, 0);
    int fourcc = vw.fourcc('M', 'J', 'P', 'G'); //设置摄像头编码方式
    capture.set(CAP_PROP_FOURCC, fourcc);

    while (true)
    {
        Mat image;
        capture >> image;
        //   resize(image, image, Size(1280, 720));

        engineer_tool::TransformRecv(recvStruct, receiveMessage);
        //int usart_ret = usart.UsartRecv(recvStruct);
        camera->GetImage(frame);
        double time = (double)getTickCount();
        int usart_send = usart.UsartSend(sendStruct);
        if (frame.cvimage_.empty())
        {
            continue;
        }
        
        if (image.empty())
        {
            continue;
        }
        
        Mat osrc = frame.cvimage_.clone();

        // static int tool = 0;
        // cv::imwrite("tool.jpg",osrc);
        // tool++;

        //  cv::imwrite("/home/tdt/桌面/TDT2022/video", osrc);

        // imshow("aa", osrc);
        resize(osrc, osrc, Size(480, 480));
        vector<Detector::Object> detected_objects;
        detector->process_frame(frame.cvimage_, detected_objects);
        for (int i = 0; i < detected_objects.size(); ++i)
        {
            int xmin = detected_objects[i].rect.x;
            int ymin = detected_objects[i].rect.y;
            int width = detected_objects[i].rect.width;
            int height = detected_objects[i].rect.height;
            Rect rect(xmin, ymin, width, height); //左上坐标（x,y）和矩形的长(x)宽(y)
            cv::rectangle(osrc, rect, Scalar(255, 0, 0), 1, LINE_8, 0);
            cv::putText(osrc, detected_objects[i].name,
                        cv::Point(detected_objects[i].rect.x,
                                  detected_objects[i].rect.y),
                        5,
                        5,
                        Scalar(255, 0, 255));
        }

        ksDetect_.Get(osrc, detected_objects, sendStruct, recvStruct);
        lightDetect_.Getlight(image, sendStruct, recvStruct);
        cout << "face" << sendStruct.judgeface << endl;
        cout << "flag" << sendStruct.judgeFlag << endl;
        time = ((double)cv::getTickCount() - time) / cv::getTickFrequency();

        cout << "fps:" << 1 / time << "fps" << endl;
        cv::imshow("a", osrc);
        cv::waitKey(1);
    }
}

int main(int argc, char **argv)
{
    tdtconfig::Init();
    LoadParam::Init("../../config/robot_param.yaml");
    ProgOnImage();
    return 0;
}
