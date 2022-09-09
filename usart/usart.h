//
// Created by li on 2019/12/22.
//
#ifndef TDTENGINEER_USART_H
#define TDTENGINEER_USART_H

#include <errno.h>
#include <fcntl.h> //文件控制定义
#include <mutex>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <termios.h> //终端控制定义
#include <unistd.h>
#include <vector>

//#include <tdtcommon.h>

//串口名称
const char kDevice[] = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0";
/******************非特殊禁止修改！！！********************/
/**
 * @brief    传口通信协议
 *
 * @author   何武军
 * @qq       2522548651
 *
 */
namespace tdtusart
{
#pragma pack(1)
    struct Recv_Struct_t
    {
        uint8_t frameHeader;       // 0xA5
        uint16_t vision = 0;       //使用视觉，vision为1是代表第一次取矿，2代表第二次，3代表nuc关机  1 2
        // uint8_t positionJudge = 0; //电控已升起抬升，吸盘已就位，收到后进行闪烁灯判断，避免吸盘移动对判断造成影响
        // uint8_t chassisLock = 0;   //电控底盘已对位成功并锁死，收到后停止发送位置信息
        uint16_t CRC16CheckSum;
    };
    struct Send_Struct_t
    {
        uint8_t frameHeader; // 0xA5s
        int judgeFlag = 0;//判断是否转正
        int judgeface = 0;//判断是哪个面
        double center_dx=0;
        uint16_t CRC16CheckSum;
    };
#pragma pack()

    class Usart
    {
    public:
        virtual int UsartSend(Send_Struct_t &send_message) = 0;
        virtual int UsartRecv(Recv_Struct_t &receive_message) = 0;
    };

    class RealUsart : public tdtusart::Usart
    {
    public:
        int UsartSend(Send_Struct_t &send_message);
        /**
         * @brief    串口接收数据
         *            要求每次接受最多25字节数据
         */
        int UsartRecv(Recv_Struct_t &receive_message);

        static int serial_fd_; //设备名称
        RealUsart();           //串口初始化

        /**
         * @brief    串口接收数据
         *            要求启动后，在pc端发送ascii文件
         */
        // template <class T> int UsartPreRecv(T receive_message);
        int UsartPreRecv(volatile int *receive_message);
        uint16_t GetCRC16CheckSum(uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC);
        uint32_t VerifyCRC16CheckSum(uint8_t *pchMessage, uint32_t dwLength);
        void AppendCRC16CheckSum(uint8_t *pchMessage, uint32_t dwLength);

        bool Wrong_CRC_Check = false;
        bool Wrong_Recvived_Len = false;
        bool Wrong_Usart_Offline = false;
    };

} // namespace tdtusart
#endif // TDTVision_RM2021_USART_H