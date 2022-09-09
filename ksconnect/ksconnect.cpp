#include "ksconnect.h"
#include "tool.h"
#include <cmath>
using namespace std;
using namespace cv;
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#define minarea 500
#define maxarea 20000

#define thresholdValue 140
#define target_y1 176 //white190
#define target_y2 420 //white
#define target_y3 176 //tiaoma
#define target_y4 420 //tiaoma
#define target_y5 176 //R
#define target_y6 416 //R
//免驱110

namespace ksconnect
{
    void ksDetect::getksBar(cv::Mat &src)
    {
        Mat pyr, img, img_thresholded;
        pyrDown(src, pyr, Size(src.cols / 2, src.rows / 2)); //高斯降噪，并只取奇数行列缩小图片    // 缩小和放大图像以滤除噪音
        pyrUp(pyr, src, src.size());                         //插入偶数行列，再次高斯降噪
        cvtColor(src, img, COLOR_BGR2GRAY);
        threshold(img, img_thresholded, thresholdValue, 255, THRESH_BINARY); // 190//110
        imshow("operate", img_thresholded);
        Canny(img_thresholded, img_thresholded, 100, 200, 3);
        this->ksoperate = img_thresholded.clone();
    }
    bool comp(engineer_tool::modelL a, engineer_tool::modelL b)
    {
        return a.center_.x < b.center_.x;
    }
    void ksDetect::sortSquare(vector<engineer_tool::modelL> &square)
    {
        sort(square.begin(), square.end(), comp);
    }
    void drawPoint(engineer_tool::modelL &contour, cv::Mat &img)
    {
        cv::putText(img, "p0", contour.p0, cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0, 0, 255), 5, 8, 0);
        cv::putText(img, "p1", contour.p1, cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0, 0, 255), 5, 8, 0);
        cv::putText(img, "p2", contour.p2, cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0, 0, 255), 5, 8, 0);
        cv::putText(img, "p3", contour.p3, cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0, 0, 255), 5, 8, 0);
        cv::putText(img, "p4", contour.p4, cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0, 0, 255), 5, 8, 0);
        cv::putText(img, "p5", contour.p5, cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0, 0, 255), 5, 8, 0);
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
    static double angle22(Point pt1, Point pt2, Point pt0) //三点形成的角度
    {
        double theta = atan2(pt1.x - pt0.x, pt1.y - pt0.y) - atan2(pt2.x - pt0.x, pt2.y - pt0.y);
        if (theta > M_PI)
            theta -= 2 * M_PI;
        if (theta < -M_PI)
            theta += 2 * M_PI;

        return theta;
        // cos<a,b>=(ab的内积)/(|a||b|)
    }
    double distance(cv::Point a, cv::Point b)
    {
        return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
    }
    static double getSendangle(Point centerPoint, Point squarePoint)
    {
        double sendAngle22 = 0;
        cv::Point2f hypothetical = cv::Point2f(0, 0); //假设点
        hypothetical.x = centerPoint.x + 30;
        hypothetical.y = centerPoint.y;

        sendAngle22 = angle22(hypothetical, squarePoint, centerPoint) * 57.3f;

        return sendAngle22; //+45?
    }
    void ksDetect::start(tdtusart::Send_Struct_t &sendMessage)
    {
        sendMessage.judgeFlag = this->judgeFlag;
        sendMessage.judgeface = this->judgeface;
        // cout << "qq" << sendMessage.judgeFlag << endl;
        // cout << "face" << sendMessage.judgeface << endl;
    }
    void ksDetect::Get(cv::Mat &img, std::vector<Detector::Object> &detected_objects, tdtusart::Send_Struct_t &sendMessage, tdtusart::Recv_Struct_t &recvMessage)
    {
        this->judgeFlag = 0;
        this->judgeface = 0;

        int square_flag1 = 0;
        int square_flag2 = 0;
        int square_flag3 = 0;
        int square_flag4 = 0;
        int square_flag5 = 0;  //L
        int square_flag6 = 0;  //L
        int square_flag0 = 0;  //L
        int square_flag01 = 0; //L
        int square_flag02 = 0; //L
        int square_flag03 = 0; //L
        int square_flag04 = 0; //L
        int square_flag00 = 0; //L
        Point2d squarescenter;
        Point2d L_center;
        squarescenter.x = 0;
        squarescenter.y = 0;
        L_center.x = 0;
        L_center.y = 0;

        for (int i = 0; i < detected_objects.size(); ++i)
        {
            int xmin = detected_objects[i].rect.x;
            int ymin = detected_objects[i].rect.y;
            int width = detected_objects[i].rect.width;
            int height = detected_objects[i].rect.height;
            Rect rect(xmin, ymin, width, height); //左上坐标（x,y）和矩形的长(x)宽(y)
            if (detected_objects[i].name == "0")  // tiaoma
            {
                this->judgeface = 1;
            }
            if (detected_objects[i].name == "1") // white
            {
                this->judgeface = 2;
            }
            if (detected_objects[i].name == "2") // R
            {
                this->judgeface = 3;
            }
            if (detected_objects[i].name == "3") // square
            {
                this->squares.push_back(detected_objects[i]);
            }
            if (detected_objects[i].name == "4") // L
            {
                this->Ls.push_back(detected_objects[i]);
            }
        }
        if (this->judgeface == 2) //white
        {
            for (int i = 0; i < squares.size(); i++)
            {
                squarescenter.x = squares[i].rect.x + squares[i].rect.width / 2.0;
                squarescenter.y = squares[i].rect.y + squares[i].rect.height / 2.0;
                circle(img, squarescenter, 5, Scalar(0, 0, 255));
                if (abs(squarescenter.y - target_y1) / squares[i].rect.height < 0.3)
                {
                    square_flag1 = 1;
                }
                if (abs(squarescenter.y - target_y2) / squares[i].rect.height < 0.35) //0.3
                {
                    square_flag2 = 1;
                }
            }
            for (int i = 0; i < Ls.size(); i++)
            {
                L_center.x = Ls[i].rect.x + Ls[i].rect.width / 2.0;
                L_center.y = Ls[i].rect.y + Ls[i].rect.height / 2.0;
                circle(img, L_center, 5, Scalar(0, 0, 255));
                if (abs(L_center.y - target_y1) / Ls[i].rect.height < 0.3) //0.4
                {
                    square_flag01 = 1;
                }
                if (abs(L_center.y - target_y2) / Ls[i].rect.height < 0.30)
                {
                    square_flag02 = 1;
                }
            }
            cout << square_flag1 << square_flag2 << square_flag01 << square_flag02 << endl;
            // if (square_flag1 == 1 && square_flag2 == 1 && (square_flag01 == 1 && square_flag02 == 1))
            if ((square_flag1 + square_flag2 + square_flag01 + square_flag02) >= 3)
            {
                this->judgeFlag = 1; //转平啦
            }
        }
        if (this->judgeface == 1) //tiaoma
        {
            for (int i = 0; i < squares.size(); i++)
            {
                squarescenter.x = squares[i].rect.x + squares[i].rect.width / 2.0;
                squarescenter.y = squares[i].rect.y + squares[i].rect.height / 2.0;
                circle(img, squarescenter, 5, Scalar(0, 0, 255));

                if (abs(squarescenter.y - target_y3) / squares[i].rect.height < 0.3)
                {
                    square_flag3 = 1;
                }
                if (abs(squarescenter.y - target_y4) / squares[i].rect.height < 0.35)
                {
                    square_flag4 = 1;
                }
            }
            for (int i = 0; i < Ls.size(); i++)
            {
                L_center.x = Ls[i].rect.x + Ls[i].rect.width / 2.0;
                L_center.y = Ls[i].rect.y + Ls[i].rect.height / 2.0;
                circle(img, L_center, 5, Scalar(0, 0, 255));
                if (abs(L_center.y - target_y3) / Ls[i].rect.height < 0.3)
                {
                    square_flag03 = 1;
                }
                if (abs(L_center.y - target_y4) / Ls[i].rect.height < 0.35)
                {
                    square_flag04 = 1;
                }
                if (L_center.y < (target_y4 - 50) && L_center.y > (target_y3 + 50))
                {
                    square_flag00 = 1;
                }
            }
            //if (square_flag3 == 1 && square_flag4 == 1 && square_flag00 != 1 && square_flag03 == 1 && square_flag04 == 1)
            if ((square_flag3 + square_flag4 + square_flag03 + square_flag04) >= 3 && square_flag00 != 1)
            {
                this->judgeFlag = 1; //转平啦
            }
        }

        if (this->judgeface == 3) //R
        {
            for (int i = 0; i < Ls.size(); i++)
            {
                L_center.x = Ls[i].rect.x + Ls[i].rect.width / 2.0;
                L_center.y = Ls[i].rect.y + Ls[i].rect.height / 2.0;
                circle(img, L_center, 5, Scalar(0, 255, 0));
                if (abs(L_center.y - target_y5) / Ls[i].rect.height < 0.25)
                {
                    square_flag5++;
                }
                if (abs(L_center.y - target_y6) / Ls[i].rect.height < 0.4)
                {
                    square_flag6++;
                }
                if (L_center.y < (target_y6 - 50) && L_center.y > (target_y5 + 50))
                {
                    square_flag0 = 1;
                }
            }
            for (int i = 0; i < squares.size(); i++)
            {
                squarescenter.x = squares[i].rect.x + squares[i].rect.width / 2.0;
                squarescenter.y = squares[i].rect.y + squares[i].rect.height / 2.0;
                circle(img, squarescenter, 5, Scalar(0, 255, 0));
                if (abs(squarescenter.y - target_y5) / squares[i].rect.height < 0.25)
                {
                    square_flag5++;
                }
                if (squarescenter.y < (target_y6 - 50) && squarescenter.y > (target_y5 + 50))
                {
                    square_flag0 = 1;
                }
            }
            cout << "as" << square_flag0 << endl;
            cout << "as2" << square_flag6 << endl;
            if (((square_flag5 >= 2 && square_flag6 == 1) || (square_flag5 >= 2 && square_flag6 == 2)) && square_flag0 != 1)
            {
                this->judgeFlag = 1; //转平啦
            }
        }
        if (this->judgeface == 0)
        {
            this->judgeFlag = 0; //SD
        }
        cout << "square_flag1" << square_flag1 << endl;
        cout << "square_flag2" << square_flag2 << endl;
        cv::line(img, Point(10, target_y5), Point(600, target_y5), cv::Scalar(0, 0, 255), 2);
        cv::line(img, Point(10, target_y6), Point(600, target_y6), cv::Scalar(0, 0, 255), 2);
        start(sendMessage);
        squares.clear();
        Ls.clear();
    }
    bool ksDetect::aroundJudge(Mat &imgRoi, Point &center)
    {
        float blackcount = 0.0;
        float whitecount = 0.0;
        float rate = 0.0;
        bool flagblack;
        for (int i = 0; i < imgRoi.rows; i++)
        {
            for (int j = 0; j < imgRoi.cols; j++)
            {
                // int iii=imgRoi.at<uchar>(i, j);
                // cout<<"as"<<iii<<endl;
                if (imgRoi.at<uchar>(i, j) == 255)
                {
                    blackcount++;
                }
                else
                    whitecount++;
            }
            rate = blackcount * 1.0 / (blackcount + whitecount) * 1.0;
            // cout << "rate" << rate << endl;
            cout << "rate" << rate << endl;
            if (rate > 0.01) //代测试的参数
                flagblack = true;
            else
                flagblack = false;
            return flagblack;
        }
    }
    void ksDetect::modelJudge(Mat &img, std::vector<engineer_tool::modelL> &L_LU,
                              std::vector<engineer_tool::modelL> &L_LD, std::vector<engineer_tool::modelL> &L_RD,
                              std::vector<engineer_tool::modelL> &L_RU, std::vector<engineer_tool::modelL> &square)
    {
        //特征点世界坐标
        //初始化相机参数Opencv
        double camD[9] = {
            600.7, 0, 665.8,
            0, 798.1, 667.6,
            0, 0, 1};
        cv::Mat camera_matrix = cv::Mat(3, 3, CV_64FC1, camD);

        //畸变参数
        double distCoeffD[5] = {-0.189314, 0.444657, 0, 0, -1.57547};
        cv::Mat distortion_coefficients = cv::Mat(5, 1, CV_64FC1, distCoeffD);

        Points3D.push_back(cv::Point3f(-60, 60, 0));  // P1 120mm以矿石中心为坐标原点从左上顺时针
        Points3D.push_back(cv::Point3f(60, 60, 0));   // P2
        Points3D.push_back(cv::Point3f(60, -60, 0));  // P3
        Points3D.push_back(cv::Point3f(-60, -60, 0)); // P4

        // diagonal=对角线，standardLen=标志块基础长度（rec边长，L型最长的边）
        // judgeface++;
        this->flag_zhuan = 0;
        this->judgeface = 0;
        Mat imgthershold22;
        img.copyTo(imgthershold22);
        const int Range = 150;    // 30
        double targetangley = 44; // pnp结算出来的angle
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

                if (value < 8 && value > 4) // 75
                    if (ldSingle.center_.y >= ruSingle.center_.y || ruSingle.center_.x >= ldSingle.center_.x)
                    {
                        // cv::line(img, ldSingle.center_, ruSingle.center_, cv::Scalar(0, 0, 255), 2);
                        cv::Point center; //当前 匹配的中心点
                        center.x = (ruSingle.center_.x + ldSingle.center_.x) * 0.5;
                        center.y = (ldSingle.center_.y + ruSingle.center_.y) * 0.5;

                        if (L_LU.size() == 0 && L_RD.size() == 0)
                        {
                            if (square.size() > 1) //跟两个正方形进行匹配
                            {
                                int flag3d = 0;
                                for (const auto &squareSingle : square)
                                {
                                    if (distance(squareSingle.center_, center) > diagonal * 0.5 - Range && distance(squareSingle.center_, center) < diagonal * 0.5 + Range)
                                    {
                                        flag3d++;
                                        //给像素点赋值
                                        if (flag3d == 1)
                                        {
                                            Points2D.push_back(cv::Point2f(squareSingle.center_.x, squareSingle.center_.y)); // P1
                                            Points2D.push_back(cv::Point2f(ruSingle.center_.x, ruSingle.center_.y));         // P2
                                        }
                                        if (flag3d == 2)
                                        {
                                            Points2D.push_back(cv::Point2f(squareSingle.center_.x, squareSingle.center_.y)); // P3
                                            Points2D.push_back(cv::Point2f(ldSingle.center_.x, ldSingle.center_.y));         // P4
                                        }

                                        Mat roimg = imgthershold22(Rect(center.x, center.y, 60, 60));
                                        if (aroundJudge(roimg, center) == true)
                                        {
                                            this->judgeface = 1;
                                            cv::putText(this->target, "me", center, cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0, 0, 255), 5, 8, 0);
                                            //  circle(this->target, Point(center.x, center.y), 30, Scalar(0, 0, 255));
                                        }
                                        else
                                        {
                                            this->judgeface = 2;
                                            cv::putText(this->target, "opposite", center, cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0, 0, 255), 5, 8, 0);
                                        }
                                    }
                                }
                                //初始化输出矩阵
                                cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64FC1);
                                cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64FC1);
                                cout << "2d" << Points2D.size() << endl;
                                cout << "3d" << Points3D.size() << endl;
                                if (Points2D.size() == 4 && Points3D.size() == 4)
                                    //三种方法求解
                                    // solvePnP(Points3D, Points2D, camera_matrix, distortion_coefficients, rvec, tvec, false, CV_ITERATIVE);    //实测迭代法似乎只能用4个共面特征点求解，5个点或非共面4点解不出正确的解
                                    cv::solvePnP(Points3D, Points2D, camera_matrix, distortion_coefficients, rvec, tvec, false, SOLVEPNP_P3P); // Gao的方法可以使用任意四个特征点，特征点数量不能少于4也不能多于4
                                // solvePnP(Points3D, Points2D, camera_matrix, distortion_coefficients, rvec, tvec, false, CV_EPNP);         //该方法可以用于N点位姿估计;与前两种有偏差
                                //旋转向量变旋转矩阵
                                //提取旋转矩阵
                                double rm[9];
                                cv::Mat rotM(3, 3, CV_64FC1, rm);
                                Rodrigues(rvec, rotM);
                                cout << "use CV_P3P:the rotation matrix is :\n"
                                     << rotM << endl;
                                cout << "use CV_P3P:the tran matrix is :\n"
                                     << tvec << endl;
                                double r11 = rotM.at<double>(0, 0);
                                double r21 = rotM.at<double>(1, 0);
                                double r31 = rotM.at<double>(2, 0);
                                double r32 = rotM.at<double>(2, 1);
                                double r33 = rotM.at<double>(2, 2);

                                const double PI = 3.141592653;
                                double thetaz = atan2(r21, r11) / PI * 180;
                                double thetay = atan2(-1 * r31, sqrt(r32 * r32 + r33 * r33)) / PI * 180;
                                double thetax = atan2(r32, r33) / PI * 180;
                                cout << "thetay" << thetay << endl;
                                cout << "thetaz" << thetaz << endl;
                                cout << "thetax" << thetax << endl;

                                Points2D.clear();
                                Points3D.clear();
                            }
                        }
                        if (L_RD.size()) //左下，右下，右上匹配
                        {
                            for (const auto &rdSingle : L_RD)
                            {
                                if (distance(center, rdSingle.center_) < diagonal * 0.5 + Range && distance(center, rdSingle.center_) > diagonal * 0.5 - Range)
                                    if (rdSingle.center_.x > center.x && rdSingle.center_.y > center.y)
                                    {
                                        // cv::line(img, center, rdSingle.center_, cv::Scalar(0, 0, 255), 2);
                                        for (const auto &squareSingle : square)
                                        { //匹配正方形
                                            //
                                            if (distance(squareSingle.center_, center) > diagonal * 0.5 - Range && distance(squareSingle.center_, center) < diagonal * 0.5 + Range)
                                            {
                                                if (squareSingle.center_.x < center.x && squareSingle.center_.y < center.y)
                                                { // rec在左上

                                                    this->judgeface = 4;
                                                    // cv::line(img, center, squareSingle.center_, cv::Scalar(0, 0, 255), 2);
                                                    cv::putText(this->target, "dao4", center, cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0, 0, 255), 5, 8, 0);
                                                }
                                            }
                                        }
                                    }
                            }
                        }
                        if (L_LU.size()) //左上，右下，右上匹配
                        {
                            for (const auto &luSingle : L_LU)
                            {
                                if (distance(center, luSingle.center_) < diagonal * 0.5 + Range && distance(center, luSingle.center_) > diagonal * 0.5 - Range)
                                    if (luSingle.center_.x < center.x && luSingle.center_.y < center.y)
                                    {
                                        // cv::line(img, center, luSingle.center_, cv::Scalar(0, 0, 255), 2);
                                        for (const auto &squareSingle : square)
                                        { //匹配正方形

                                            if (distance(squareSingle.center_, center) > diagonal * 0.5 - Range && distance(squareSingle.center_, center) < diagonal * 0.5 + Range)
                                            {
                                                if (squareSingle.center_.x > center.x && squareSingle.center_.y > center.y)
                                                { // rec在右下
                                                    //  this->judgeface = 45 - getSendangle(center, squareSingle.center_);
                                                    this->judgeface = 3;
                                                    // cv::line(img, center, squareSingle.center_, cv::Scalar(0, 0, 255), 2);
                                                    cv::putText(this->target, "zhe3", center, cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0, 0, 255), 5, 8, 0);
                                                }
                                            }
                                        }
                                    }
                            }
                        }
                    }
            }
        }
        for (const auto &luSingle : L_LU) //左上右下匹配
        {
            for (const auto &rdSingle : L_RD)
            {
                diagonal = distance(luSingle.center_, rdSingle.center_);
                standardLen = luSingle.cross_;
                value = diagonal / standardLen;
                // cout << "standardLen" << standardLen << endl;
                // cout << "diagonal" << diagonal << endl;
                // cout << "value" << value << endl;
                if (value < 8 && value > 4) // 75
                    if (luSingle.center_.y <= rdSingle.center_.y || rdSingle.center_.x >= luSingle.center_.x)
                    //  if (rdSingle.center_.x > luSingle.center_.x)
                    {
                        //  cv::line(img, luSingle.center_, rdSingle.center_, cv::Scalar(0, 0, 255), 2);
                        cv::Point center; //当前 匹配的中心点
                        center.x = (rdSingle.center_.x + luSingle.center_.x) * 0.5;
                        center.y = (luSingle.center_.y + rdSingle.center_.y) * 0.5;

                        if (L_RU.size() == 0 && L_LD.size() == 0)
                        {
                            if (square.size() > 1)
                            {
                                for (const auto &squareSingle : square)
                                {
                                    if (distance(squareSingle.center_, center) > diagonal * 0.5 - Range && distance(squareSingle.center_, center) < diagonal * 0.5 + Range)
                                    {
                                        Mat roimg = imgthershold22(Rect(center.x, center.y, 60, 60));
                                        if (aroundJudge(roimg, center) == true)
                                        {
                                            this->judgeface = 1;
                                            cv::putText(this->target, "me", center, cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0, 0, 255), 5, 8, 0);
                                        }
                                        else
                                        {
                                            this->judgeface = 2;
                                            cv::putText(this->target, "opposite", center, cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0, 0, 255), 5, 8, 0);
                                        }
                                        // to do哪一个面
                                    }
                                }
                            }
                        }
                        if (L_RU.size()) //左上，右下，右上匹配
                        {
                            for (const auto &ruSingle : L_RU)
                            {
                                if (distance(center, ruSingle.center_) < diagonal * 0.5 + Range && distance(center, ruSingle.center_) > diagonal * 0.5 - Range)
                                    if (ruSingle.center_.x > center.x && ruSingle.center_.y < center.y)
                                    {
                                        //   cv::line(img, center, ruSingle.center_, cv::Scalar(0, 0, 255), 2);
                                        for (const auto &squareSingle : square)
                                        { //匹配正方形
                                            //
                                            if (distance(squareSingle.center_, center) > diagonal * 0.5 - Range && distance(squareSingle.center_, center) < diagonal * 0.5 + Range)
                                            {
                                                if (squareSingle.center_.x < center.x && squareSingle.center_.y > center.y)
                                                { // rec在左下
                                                    // this->judgeface = -getSendangle(center, squareSingle.center_) + 45;
                                                    this->judgeface = 6;
                                                    //        this->judgeFlag = 3;
                                                    //  cv::line(img, center, squareSingle.center_, cv::Scalar(0, 0, 255), 2);
                                                    cv::putText(this->target, "you6", center, cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0, 0, 255), 5, 8, 0);
                                                }
                                            }
                                        }
                                    }
                            }
                        }
                        if (L_LD.size()) //左上，左下，右下匹配
                        {
                            for (const auto &ldSingle : L_LD)
                            {
                                if (distance(center, ldSingle.center_) < diagonal * 0.5 + Range && distance(center, ldSingle.center_) > diagonal * 0.5 - Range)
                                    if (ldSingle.center_.x < center.x && ldSingle.center_.y > center.y)
                                    {
                                        // cv::line(img, center, ldSingle.center_, cv::Scalar(0, 0, 255), 2);
                                        for (const auto &squareSingle : square)
                                        { //匹配正方形

                                            if (distance(squareSingle.center_, center) > diagonal * 0.5 - Range && distance(squareSingle.center_, center) < diagonal * 0.5 + Range)
                                            {
                                                if (squareSingle.center_.x > center.x && squareSingle.center_.y < center.y)
                                                { // rec在右上
                                                    //   this->judgeface = -getSendangle(center, squareSingle.center_) + 45; //+45
                                                    this->judgeface = 5;
                                                    //  this->judgeFlag = 3;
                                                    // cv::line(img, center, squareSingle.center_, cv::Scalar(0, 0, 255), 2);
                                                    cv::putText(this->target, "zuo5", center, cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0, 0, 255), 5, 8, 0);
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
//考虑状态机，
//相机角度，能看全四个，逻辑上至少看见的是下面两个？
// 3.9  空接：可见左正方形和右下L型，从右看逆时针转一个角度转平刚好条形码朝上
// 3.9 从地上取： 同理可以调整
// 3.9  除此之外的状态 ：电控先转平，我给出是哪个面从而解算
//状态切换：转正进入面的判断模式
// s rd
// ld s
// ld rd
//加入面的判断
//四种面：1：条形码；2：空白面；3：正R面；4：倒R面；5：左R面；6：右R面
//转平考虑上面的角点
// s ru