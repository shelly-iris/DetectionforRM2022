#ifndef LIGHT_H
#define LIGHT_H

#include <algorithm>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include "../tool/tool.h"
using namespace cv;
using namespace std;

namespace light
{
    class lightDetect
    {
    public:
        lightDetect()
        {
        }
        ~lightDetect() {}
        void start(tdtusart::Send_Struct_t &sendMessage);
        void imagedeal(cv::Mat &src);
        void modelJudge(std::vector<engineer_tool::modelL> &L_LU,
                        std::vector<engineer_tool::modelL> &L_LD, std::vector<engineer_tool::modelL> &L_RD,
                        std::vector<engineer_tool::modelL> &L_RU);
        void Getlight(cv::Mat &frame, tdtusart::Send_Struct_t &sendMessage, tdtusart::Recv_Struct_t &recvMessage);
        vector<vector<Point>> light_contours;
        vector<vector<Point>> shine_contours;
        vector<string> texts;
        vector<RotatedRect> light_infos;
        vector<vector<Point>> shine_infos;
        std::vector<engineer_tool::modelL> L_LU;
        // L型左上
        std::vector<engineer_tool::modelL> L_LD;
        // L型左下
        std::vector<engineer_tool::modelL> L_RU;
        // L型右上
        std::vector<engineer_tool::modelL> L_RD;
        // L型右下
        cv::Mat lightoperate;
        cv::Mat framecopy;
    };

} // namespace light

#endif