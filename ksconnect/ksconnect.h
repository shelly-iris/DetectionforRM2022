#ifndef KSCONECT_H
#define KSCONECT_H

#include <algorithm>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include "../tool/tool.h"
#include "detector.h"

namespace ksconnect
{

  class ksDetect
  {
  public:
    ksDetect()
    {
    }
    ~ksDetect() {}
    void getksBar(cv::Mat &src);
    void sortSquare(std::vector<engineer_tool::modelL> &square); //从左至右

    // void Get(cv::Mat &img, tdtusart::Send_Struct_t &sendMessage, tdtusart::Recv_Struct_t &recvMessage);
    void Get(cv::Mat &img,std::vector<Detector::Object> &detected_objects, tdtusart::Send_Struct_t &sendMessage, tdtusart::Recv_Struct_t &recvMessage);
    bool aroundJudge(cv::Mat &img, cv::Point &center);
    void start(tdtusart::Send_Struct_t &sendMessage);

    void modelJudge(cv::Mat &img, std::vector<engineer_tool::modelL> &L_LU,
                    std::vector<engineer_tool::modelL> &L_LD, std::vector<engineer_tool::modelL> &L_RD,
                    std::vector<engineer_tool::modelL> &L_RU, std::vector<engineer_tool::modelL> &square);
    // void modelJudge(std::vector<engineer_tool::CustomRect> &modelks);
    cv::Mat ksoperate;
    cv::Mat target;
    int judgeFlag = 0;
    int flag_zhuan = 0;
    int judgeface = 0;
    float sengAngle2 = 0;
    int visionzf = 0;
    std::vector<Detector::Object> squares;
    //深度学习得到的正方形
    std::vector<Detector::Object> Ls;
    //深度学习得到的L型
    std::vector<engineer_tool::modelL> allKsbars;
    //粗筛选轮廓
    std::vector<engineer_tool::modelL> L_LU;
    // L型左上
    std::vector<engineer_tool::modelL> L_LD;
    // L型左下
    std::vector<engineer_tool::modelL> L_RU;
    // L型右上
    std::vector<engineer_tool::modelL> L_RD;
    // L型右下
    std::vector<engineer_tool::modelL> square;
    //正方形
    std::vector<engineer_tool::modelL> resultks;
    //筛选出来的结果
    std::vector<engineer_tool::modelL> tiaoma;
    //条形码uesless
    std::vector<cv::Point2f> Points2D;
    //二维平面上的像素点
    std::vector<cv::Point3f> Points3D;
    //三维坐标系下的点，单位mm
  };

}

#endif
