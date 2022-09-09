#include <opencv2/opencv.hpp>
#include "iostream"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "light.h"
#include <eigen3/Eigen/Dense>     //for cv2eigen()
#include <opencv2/core/eigen.hpp> //for cv2eigen()

using namespace cv;
using namespace std;

#define judgesquarecos 0.3
#define HALF_LENGTH 125 //兑换站实际值
#define pi 3.14
#define thresholdimage 60
#define color 0

namespace light
{
    double distance(cv::Point a, cv::Point b)
    {
        return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
    }
    static double angle(Point pt1, Point pt2, Point pt0) //三点形成的角度
    {
        double dx1 = pt1.x - pt0.x;
        double dy1 = pt1.y - pt0.y;
        double dx2 = pt0.x - pt2.x;
        double dy2 = pt0.y - pt2.y;
        return (dx1 * dx2 + dy1 * dy2) / sqrt((dx1 * dx1 + dy1 * dy1) * (dx2 * dx2 + dy2 * dy2) + 1e-10);
        // cos<a,b>=(ab的内积)/(|a||b|)
    }
    void lightDetect::start(tdtusart::Send_Struct_t &sendMessage) //串口
    {
        // sendMessage.center_dx = this->center_dx;
        // cout << "qq" << sendMessage.center_dx << endl;
    }
    void lightDetect::imagedeal(cv::Mat &src) //图像处理
    {
        Mat img, img_thresholded, pyr;
        std::vector<cv::Mat> bgr;
        pyrDown(src, pyr, Size(src.cols / 2, src.rows / 2)); //高斯降噪，并只取奇数行列缩小图片    // 缩小和放大图像以滤除噪音
        pyrUp(pyr, src, src.size());                         //插入偶数行列，再次高斯降噪
        split(src, bgr);
        img = bgr[color] - bgr[2 - color]; // blue
        threshold(img, img_thresholded, thresholdimage, 255, THRESH_BINARY);
        Canny(img_thresholded, img_thresholded, 100, 200, 3);                                                                //加上效果更好
        this->lightoperate = img_thresholded.clone();
        imshow("operate", img_thresholded);
    }
    void lightDetect::Getlight(cv::Mat &frame, tdtusart::Send_Struct_t &sendMessage, tdtusart::Recv_Struct_t &recvMessage)
    {
        frame.copyTo(this->framecopy);
        vector<vector<Point>> all_contours;
        imagedeal(frame);
        findContours(this->lightoperate, all_contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
        vector<vector<Point>> contours_poly(all_contours.size()); //用于存放折线点集
        for (int i = 0; i < (int)all_contours.size(); i++)
        {
            approxPolyDP(Mat(all_contours[i]), contours_poly[i], arcLength(all_contours[i], true) * 0.04, true);
        }
        for (const auto &contours : contours_poly)
        {
            engineer_tool::modelL contour(contours);
            std::vector<cv::Point2f> Points_;
            Points_ = contour.GetVertices2f();
            // const int Area_difference = 7000; //350
            bool rec_condition1 = false;
            bool l_condition1 = false;
            bool l_condition2 = false;
            float area = contour.GetArea();
            if (area < 300 || area > 20000)
                continue;

            if ((contour.GetNum() == 6 && !isContourConvex(Mat(contours))))
            {
                int a = pow((Points_[0].x - Points_[4].x), 2) + pow((Points_[0].y - Points_[4].y), 2);
                int b = pow((Points_[0].x - Points_[2].x), 2) + pow((Points_[0].y - Points_[2].y), 2);
                double length1 = sqrt(pow(Points_[0].x - Points_[1].x, 2) + pow(Points_[0].y - Points_[1].y, 2));
                double length2 = sqrt(pow(Points_[0].x - Points_[5].x, 2) + pow(Points_[0].y - Points_[5].y, 2));
                double area = pow(std::max(length1, length2), 2);
                cv::Point l_center;
                l_center.x = (Points_[0].x + Points_[4].x + Points_[2].x) / 3.0;
                l_center.y = (Points_[0].y + Points_[4].y + Points_[2].y) / 3.0;

                // if (abs(contour.GetArea() - area * 0.56) < Area_difference)
                if (abs(1 - contour.GetArea() / area * 0.56) < 0.6)
                {
                    l_condition2 = true;
                }

                if (a >= b) // 0与4为对角点
                {

                    if (l_center.x < Points_[2].x && l_center.y < Points_[2].y)
                    {
                        // if ((Points_[4].y + Points_[5].y > Points_[3].y + Points_[2].y) && (Points_[3].y + Points_[2].y > Points_[1].y + Points_[0].y))
                        //     if (abs((Points_[5].x - Points_[4].x) - (Points_[5].y - Points_[0].y)) < 70)
                        {
                            l_condition1 = true;
                            if (l_condition1 && l_condition2)
                                L_RD.emplace_back(contour);
                        }
                    }

                    else if (l_center.x < Points_[5].x && l_center.y > Points_[5].y)
                    {
                        // if (Points_[3].y + Points_[4].y > Points_[1].y + Points_[2].y && (Points_[1].y + Points_[2].y > Points_[0].y + Points_[5].y))
                        //     if (abs((Points_[5].x - Points_[0].x) - (Points_[4].y - Points_[5].y)) < 70)
                        {
                            l_condition1 = true;
                            if (l_condition1 && l_condition2)
                            {
                                L_RU.emplace_back(contour);
                            }
                        }
                    }
                    else if (l_center.x > Points_[5].x && l_center.y > Points_[5].y)
                    {
                        // if (Points_[0].y + Points_[1].y > Points_[2].y + Points_[3].y && (Points_[2].y + Points_[3].y > Points_[5].y + Points_[4].y))
                        //     if (abs((Points_[0].y - Points_[5].y) - (Points_[4].x - Points_[5].x)) < 70)
                        // if (abs(Points_[5].x - Points_[0].x) < 10)
                        //     if (abs((Points_[0].y - Points_[5].y) / (Points_[1].x - Points_[0].x)) > 2)
                        //         if (abs((Points_[0].y - Points_[5].y) / (Points_[1].x - Points_[0].x)) < 3)
                        {
                            // cout << "beta" << (Points_[0].y - Points_[5].y) / (Points_[1].x - Points_[0].x) << endl;
                            l_condition1 = true;
                            if (l_condition1 && l_condition2)
                            {
                                L_LU.emplace_back(contour);

                                // cout << "area" << contour.GetArea() << endl;
                            }
                        }
                    }
                }
                else
                {

                    if (l_center.x > Points_[4].x && l_center.y < Points_[4].y)
                    {
                        // if (Points_[1].y + Points_[2].y > Points_[4].y + Points_[3].y && Points_[4].y + Points_[3].y > Points_[0].y + Points_[5].y)
                        //     if (abs((Points_[2].x - Points_[1].x) - (Points_[1].y - Points_[0].y)) < 70)
                        {
                            l_condition1 = true;
                            if (l_condition1 && l_condition2)
                                L_LD.emplace_back(contour);
                        }
                    }
                    else if (l_center.x > Points_[1].x && l_center.y > Points_[1].y)
                    {
                        // if (Points_[2].y + Points_[3].y > Points_[4].y + Points_[5].y && Points_[4].y + Points_[5].y > Points_[0].y + Points_[1].y)
                        //     if (abs((Points_[2].y - Points_[1].y) - (Points_[0].x - Points_[1].x)) < 70)
                        // if (abs(Points_[1].x - Points_[2].x) < 10)
                        //     if (abs((Points_[2].y - Points_[1].y) / (Points_[3].x - Points_[2].x)) > 2)
                        //         if (abs((Points_[2].y - Points_[1].y) / (Points_[3].x - Points_[2].x)) < 3)
                        {
                            // cout << "beta" << (Points_[2].y - Points_[1].y) / (Points_[3].x - Points_[2].x) << endl;
                            l_condition1 = true;
                            if (l_condition1 && l_condition2)
                                L_LU.emplace_back(contour);
                        }
                    }
                    else if ((l_center.x) < Points_[1].x && (l_center.y) > Points_[1].y)
                    {
                        // drawPoint(contour, img);
                        //    if (Points_[3].y + Points_[4].y > Points_[1].y + Points_[2].y && (Points_[1].y + Points_[2].y > Points_[0].y + Points_[5].y))
                        //       if (abs((Points_[5].x - Points_[0].x) - (Points_[4].y - Points_[5].y)) < 70)
                        {
                            l_condition1 = true;
                            if (l_condition1 && l_condition2)
                            {

                                L_RU.emplace_back(contour);
                            }
                        }
                    }
                } // 0，2位角点
            }
        }

        //可视化
        std::vector<std::vector<Point>> L1;
        std::vector<std::vector<Point>> L2;
        std::vector<std::vector<Point>> L3;
        std::vector<std::vector<Point>> L4;
        for (auto &it : L_RU)
        {
            L1.push_back(L_RU.back().getCount());
        }
        for (auto &it : L_LD)
        {
            L2.push_back(L_LD.back().getCount());
        }
        for (auto &it : L_LU)
        {
            L3.push_back(L_LU.back().getCount());
        }
        for (auto &it : L_RD)
        {
            L4.push_back(L_RD.back().getCount());
        }
        modelJudge(L_LU, L_LD, L_RD, L_RU);

        drawContours(this->framecopy, L1, -1, cv::Scalar(0, 0, 255), 2, 8);
        drawContours(this->framecopy, L2, -1, cv::Scalar(0, 0, 255), 2, 8);
        drawContours(this->framecopy, L3, -1, cv::Scalar(0, 120, 255), 2, 8);
        drawContours(this->framecopy, L4, -1, cv::Scalar(0, 0, 255), 2, 8);
        start(sendMessage);
        imshow("origin", this->framecopy);

        L_RU.clear();
        L_LD.clear();
        L_LU.clear();
        L_RD.clear();
    }
    void lightDetect::modelJudge(std::vector<engineer_tool::modelL> &L_LU,
                                 std::vector<engineer_tool::modelL> &L_LD, std::vector<engineer_tool::modelL> &L_RD,
                                 std::vector<engineer_tool::modelL> &L_RU)
    {

        const int Range = 140; // 30
        double diagonal,
            standardLen,
            value;
        for (const auto &ldSingle : L_LD) //左下右上匹配
        {
            for (const auto &ruSingle : L_RU)
            {
                diagonal = distance(ldSingle.center_, ruSingle.center_);
                standardLen = ldSingle.cross_;
                value = diagonal / standardLen;
                //        cout << "value" << value << endl;
                if (value < 12 && value > 4) // 75  115
                    if (ldSingle.center_.y > ruSingle.center_.y)
                        if (ruSingle.center_.x > ldSingle.center_.x)
                        {
                            cv::line(this->framecopy, ldSingle.center_, ruSingle.center_, cv::Scalar(0, 0, 255), 2);
                            cv::Point center; //当前 匹配的中心点
                            center.x = (ruSingle.center_.x + ldSingle.center_.x) * 0.5;
                            center.y = (ldSingle.center_.y + ruSingle.center_.y) * 0.5;

                            if (L_RD.size()) //左下，右下，右上匹配
                            {
                                for (const auto &rdSingle : L_RD)
                                {
                                    // cout << "ah" << diagonal * 0.5 - distance(center, rdSingle.center_) << endl;
                                    if (distance(center, rdSingle.center_) < diagonal * 0.5 + Range && distance(center, rdSingle.center_) > diagonal * 0.5 - Range)
                                        if (rdSingle.center_.x > center.x && rdSingle.center_.y > center.y)
                                        {
                                            cv::line(this->framecopy, center, rdSingle.center_, cv::Scalar(0, 0, 255), 2);
                                            for (const auto &lusingle : L_LU)
                                            { //匹配左上
                                                //

                                                if (distance(lusingle.center_, center) > diagonal * 0.5 - Range && distance(lusingle.center_, center) < diagonal * 0.5 + Range)
                                                {
                                                    if (lusingle.center_.x < center.x && lusingle.center_.y < center.y)
                                                    { // rec在左上
                                                        //自定义的物体世界坐标，单位为mm
                                                        vector<Point3f> obj = vector<Point3f>{
                                                            cv::Point3f(-HALF_LENGTH, -HALF_LENGTH, 0), // tl
                                                            cv::Point3f(HALF_LENGTH, -HALF_LENGTH, 0),  // tr
                                                            cv::Point3f(HALF_LENGTH, HALF_LENGTH, 0),   // br
                                                            cv::Point3f(-HALF_LENGTH, HALF_LENGTH, 0)   // bl
                                                        };

                                                        vector<Point2f> pnts = vector<Point2f>{
                                                            cv::Point2f(lusingle.center_.x, lusingle.center_.y), // tl
                                                            cv::Point2f(ruSingle.center_.x, ruSingle.center_.y), // tr
                                                            cv::Point2f(rdSingle.center_.x, rdSingle.center_.y), // br
                                                            cv::Point2f(ldSingle.center_.x, ldSingle.center_.y)  // bl
                                                        };
                                                        // circle(this->framecopy, cv::Point2f(lusingle.min_x, lusingle.min_y), 2, Scalar(0, 255, 0), 5);
                                                        // circle(this->framecopy, cv::Point2f(ruSingle.max_x, ruSingle.min_y), 2, Scalar(0, 255, 0), 5);
                                                        // circle(this->framecopy, cv::Point2f(rdSingle.max_x, rdSingle.max_y), 2, Scalar(0, 255, 0), 5);
                                                        // circle(this->framecopy, cv::Point2f(ldSingle.min_x, ldSingle.max_y), 2, Scalar(0, 255, 0), 5);
                                                        circle(this->framecopy, cv::Point2f(lusingle.center_.x, lusingle.center_.y), 2, Scalar(0, 255, 0), 5);
                                                        circle(this->framecopy, cv::Point2f(ruSingle.center_.x, ruSingle.center_.y), 2, Scalar(0, 255, 0), 5);
                                                        circle(this->framecopy, cv::Point2f(rdSingle.center_.x, rdSingle.center_.y), 2, Scalar(0, 255, 0), 5);
                                                        circle(this->framecopy, cv::Point2f(ldSingle.center_.x, ldSingle.center_.y), 2, Scalar(0, 255, 0), 5);

                                                        cv::Mat rVec = cv::Mat::zeros(3, 1, CV_64FC1); // init rvec
                                                        cv::Mat tVec = cv::Mat::zeros(3, 1, CV_64FC1); // init tvec
                                                        //进行位置解算

                                                        cv::Mat cam = (cv::Mat_<double>(3, 3) << 5.1413482510598203e+02, 0., 6.5338317730207848e+02, 0.,
                                                                       5.1451236692368548e+02, 3.9390298159050002e+02, 0., 0., 1.);
                                                        cv::Mat dis = (cv::Mat_<double>(4, 1) << 3.1910200097062152e-01, 1.9528971940819218e-01,
                                                                       -3.1002062794832780e-01, 2.2718917443659598e-01);
                                                        solvePnP(obj, pnts, cam, dis, rVec, tVec, false, SOLVEPNP_ITERATIVE);
                                                        //输出平移向量
                                                        cout << "tvec: " << tVec << endl;
                                                        // cout << "rVec: " << rVec << endl;
                                                        // calculate rotation angles
                                                        double theta = cv::norm(rVec);

                                                        // transformed to quaterniond
                                                        Eigen::Quaterniond q;
                                                        q.w() = cos(theta / 2);
                                                        q.x() = sin(theta / 2) * rVec.at<double>(0, 0) / theta;
                                                        q.y() = sin(theta / 2) * rVec.at<double>(0, 1) / theta;
                                                        q.z() = sin(theta / 2) * rVec.at<double>(0, 2) / theta;
                                                        double ysqr = q.y() * q.y();

                                                        // pitch (x-axis rotation)
                                                        double t0 = +2.0 * (q.w() * q.x() + q.y() * q.z());
                                                        double t1 = +1.0 - 2.0 * (q.x() * q.x() + ysqr);
                                                        double pitch = std::atan2(t0, t1);

                                                        // yaw (y-axis rotation)
                                                        double t2 = +2.0 * (q.w() * q.y() - q.z() * q.x());
                                                        t2 = t2 > 1.0 ? 1.0 : t2;
                                                        t2 = t2 < -1.0 ? -1.0 : t2;
                                                        double yaw = std::asin(t2);

                                                        // roll (z-axis rotation)
                                                        double t3 = +2.0 * (q.w() * q.z() + q.x() * q.y());
                                                        double t4 = +1.0 - 2.0 * (ysqr + q.z() * q.z());
                                                        double roll = std::atan2(t3, t4);
                                                        cout << "pitch" << pitch / pi * 180 << endl;
                                                        cout << "yaw" << yaw / pi * 180 << endl;
                                                        cout << "roll" << roll / pi * 180 << endl;
                                                        double location_x = tVec.at<double>(0, 0);
                                                        double location_y = tVec.at<double>(0, 1);
                                                        double location_z = tVec.at<double>(0, 2);
                                                        cout << "x:" << location_x << endl; //实际的y
                                                        cout << "y:" << location_y << endl; //实际的z
                                                        cout << "z:" << location_z << endl; //实际的x

                                                        // test2:
                                                        cv::Mat rotation_matrix;
                                                        cv::Mat rotation_vec;                       // 3 x 1
                                                        cv::Mat pose_mat = cv::Mat(3, 4, CV_64FC1); // 3 x 4 R | T
                                                        cv::Mat euler_angle = cv::Mat(3, 1, CV_64FC1);
                                                        cv::Mat out_intrinsics = cv::Mat(3, 3, CV_64FC1);
                                                        cv::Mat out_rotation = cv::Mat(3, 3, CV_64FC1);
                                                        cv::Mat out_translation = cv::Mat(4, 1, CV_64FC1);
                                                        cv::Rodrigues(rVec, rotation_matrix);
                                                        cv::hconcat(rotation_matrix, tVec, pose_mat);
                                                        cv::decomposeProjectionMatrix(pose_mat, out_intrinsics, out_rotation, out_translation, cv::noArray(), cv::noArray(), cv::noArray(), euler_angle);
                                                        cout << "euler_angle" << euler_angle << endl;
                                                        cv::line(this->framecopy, center, lusingle.center_, cv::Scalar(0, 0, 255), 2);
                                                        cv::putText(this->framecopy, "find", center, cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0, 0, 255), 5, 8, 0);
                                                        // cout << "findme" << endl;
                                                    }
                                                }
                                            }
                                        }
                                }
                            }
                        }
            }
        }
    }

}
