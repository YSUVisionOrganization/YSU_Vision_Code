#include "Main/headfiles.h"
#include "Communication/communication.h"
#define USE21
typedef signed char        int8_t;
typedef short              int16_t;
typedef int                int32_t;
typedef unsigned char      uint8_t;
typedef unsigned short     uint16_t;
typedef unsigned int       uint32_t;

/***********************************    ￄ1�7?    DJI提供的CRC� �检函数   ￄ1�7?  ***********************************/
/*
** Descriptions: CRC8 checksum function
** Input: Data to check,Stream length, initialized checksum
** Output: CRC checksum
*/
uint8_t Communication::Get_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength, unsigned char ucCRC8)
{
    unsigned char ucIndex;
    while (dwLength--)
    {
        ucIndex = ucCRC8 ^ (*pchMessage++);//^按位异或
        ucCRC8 = CRC8_TAB[ucIndex];
    }
    return(ucCRC8);
}

/*
** Descriptions: CRC8 Verify function
** Input: Data to Verify,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/
uint32_t Communication::Verify_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength)
{
    unsigned char ucExpected = 0;
    if ((pchMessage == 0) || (dwLength <= 2))
        return 0;
    ucExpected = Get_CRC8_Check_Sum(pchMessage, dwLength - 1, CRC8_INIT);
    return (ucExpected == pchMessage[dwLength - 1]);
}


/*
** Descriptions: append CRC8 to the end of data
** Input: Data to CRC and append,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/
void Communication::Append_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength)
{
    unsigned char ucCRC = 0;
    if ((pchMessage == 0) || (dwLength <= 2))
        return;
    ucCRC = Get_CRC8_Check_Sum((unsigned char *)pchMessage, dwLength - 1, CRC8_INIT);
    pchMessage[dwLength - 1] = ucCRC;
}



/*
** Descriptions: CRC16 checksum function
** Input: Data to check,Stream length, initialized checksum
** Output: CRC checksum
*/
uint16_t Communication::Get_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC)
{
    uint8_t chData;
    if (pchMessage == 0)
    {
        return 0xFFFF;
    }
    while (dwLength--)
    {
        chData = *pchMessage++;
        (wCRC) = ((uint16_t)(wCRC) >> 8) ^ wCRC_Table[((uint16_t)(wCRC) ^
                                                       (uint16_t)(chData)) & 0x00ff];
    }
    return wCRC;
}


/*
** Descriptions: CRC16 Verify function
** Input: Data to Verify,Streammax_p == 0.000347357
 length = Data + checksum
** Output: True or False (CRC Verify Result)
*/
uint32_t Communication::Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength)
{
    uint16_t wExpected = 0;
    if ((pchMessage == 0) || (dwLength <= 2))
    {
        return 0;
    }
    wExpected = Get_CRC16_Check_Sum(pchMessage, dwLength - 2, CRC_INIT);
    return ((wExpected & 0xff) == pchMessage[dwLength - 2] && ((wExpected >> 8) & 0xff) == pchMessage[dwLength - 1]);
}


/*
** Descriptions: append CRC16 to the end of data
** Input: Data to CRC and append,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/
void Communication::Append_CRC16_Check_Sum(uint8_t * pchMessage, uint32_t dwLength)
{
    uint16_t wCRC = 0;
    if ((pchMessage == 0) || (dwLength <= 2))
    {
        return;
    }
    wCRC = Get_CRC16_Check_Sum((uint8_t*)pchMessage, dwLength - 2, CRC_INIT);
    pchMessage[dwLength - 2] = (uint8_t)(wCRC & 0x00ff);
    pchMessage[dwLength - 1] = (uint8_t)((wCRC >> 8) & 0x00ff);
}
/***********************************    ￄ1�7?    DJI提供的CRC� �检函数   ￄ1�7?  ***********************************/


/**********************************由Kirs使用CppLinuxSerial重写************************************/
Communication::Communication()
:serialport("/dev/ttyUSB0", mn::CppLinuxSerial::BaudRate::B_115200, mn::CppLinuxSerial::NumDataBits::EIGHT, mn::CppLinuxSerial::Parity::NONE, mn::CppLinuxSerial::NumStopBits::ONE)
{

}

void Communication::InitCom(const std::string &device, mn::CppLinuxSerial::BaudRate baudRate, mn::CppLinuxSerial::NumDataBits numDataBits, mn::CppLinuxSerial::Parity parity, mn::CppLinuxSerial::NumStopBits numStopBits)
{
    this->serialport.SetDevice(device);
    this->serialport.SetBaudRate(baudRate);
    this->serialport.SetNumDataBits(numDataBits);
    this->serialport.SetParity(parity);
    this->serialport.SetNumStopBits(numStopBits);

    this->serialport.SetTimeout(-1);
}

bool Communication::open()
{

    this->serialport.Open();

    if(this->serialport.getState() == mn::CppLinuxSerial::State::OPEN)
        return true;
    else
        return false;
}

bool Communication::close()
{
    this->serialport.Close();
    if(this->serialport.getState() == mn::CppLinuxSerial::State::CLOSED)
        return true;
    else
        return false;
}

#ifndef USE21
void Communication::sendMsg()
{


    // buyao kan xiamian de nahang zhushi nashi bubin de xieyi
    // 数据ￄ1�7?                               固定ￄ1�7?                数据段长ￄ1�7? 包序ￄ1�7? CRC8  帧类ￄ1�7?                             yerr                perr               dis                  shoot    aim_mode                                    延迟时间      CRC16
    //uint8_t writeBuf[19] = {FrameHeader, 0x00, 0x0A, 0x00,   0x00,  FrameTypePC2MCU, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,    (uint8_t)Infantry.aim_mode, 0x00, 0x00, 0x00, 0x00};
    uint8_t writeBuf[19] = {0xA1, 14, 0, 0x00, 0x00,   \
                            0x00, 0x00, 0x00, 0x00, \
                            0x00, 0x00, 0x00, 0x00,   \
                            0x00, 0x00, 0x00, 0x00,   \
                            0x00, 0x00};
    Append_CRC8_Check_Sum(writeBuf, 5);

    short_to_byte temp_data_y;
    temp_data_y.s_data = (short)(Infantry.amorAttackmsg.yawErr * 10);
    writeBuf[7] = temp_data_y.data[0];
    writeBuf[8] = temp_data_y.data[1];

    short_to_byte temp_data_p;
    temp_data_p.s_data = (short)(Infantry.amorAttackmsg.pitchErr * 10);
    writeBuf[9] = temp_data_p.data[0];
    writeBuf[10] = temp_data_p.data[1];

    short_to_byte temp_data_d;
    temp_data_d.s_data = (short)(Infantry.amorAttackmsg.distance * 10);
    writeBuf[11] = temp_data_d.data[0];
    writeBuf[12] = temp_data_d.data[1];

    writeBuf[13] = Infantry.amorAttackmsg.shootFlag;
    writeBuf[14] = Infantry.amorAttackmsg.holderFlag;
    //cout<<"                     Infa shootFlag"<<Infantry.amorAttackmsg.shootFlag<<endl;
    Append_CRC16_Check_Sum(writeBuf, 19);

    std::vector<uint8_t> writeVec(writeBuf, writeBuf + 19);
    serialport.WriteBinary(writeVec);
}
#else

void Communication::sendMsg()
{
    uint8_t writeBuf[8] = {0x50,0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    short_to_byte temp_data_y;
    temp_data_y.s_data = (short)(Infantry.amorAttackmsg.yawErr * 10);
    temp_data_y.s_data=114;
    writeBuf[1] = temp_data_y.data[0];
    writeBuf[2] = temp_data_y.data[1];


    short_to_byte temp_data_p;
    temp_data_p.s_data  = (short)(Infantry.amorAttackmsg.pitchErr * 10);
    temp_data_p.s_data=514;
    writeBuf[3] = temp_data_p.data[0];
    writeBuf[4] = temp_data_p.data[1];

    writeBuf[5] = Infantry.amorAttackmsg.shootFlag;
    Append_CRC16_Check_Sum(writeBuf, 8);

    std::vector<uint8_t> writeVec(writeBuf, writeBuf + 8);
    serialport.WriteBinary(writeVec);
}
#endif


#define RECV_MSG_LEN 25 //25个数据

int Communication::recvMsg()
{
    std::vector<uint8_t> readVec;
    serialport.ReadBinary(readVec);

    //out<<"size="<<readVec.size()<<endl;//- RECV_MSG_LEN
    //for(int i =0;i<readVec.size();i++)
      //  printf("i=%d,0x%02x\n",i,readVec[i]);
    for(int i=0; i<=readVec.size(); i++){
        if(readVec[i] == FrameHeader){
            uint8_t readbuf[RECV_MSG_LEN];
            for(int j=0; j< RECV_MSG_LEN; j++){
                readbuf[j] = readVec[i+j];
            }
            uint8_t verifybuf[ RECV_MSG_LEN];
            memcpy(verifybuf, readbuf,  RECV_MSG_LEN);
            Append_CRC16_Check_Sum(verifybuf,  RECV_MSG_LEN);
            //最后两位为校验位
            if(readbuf[ RECV_MSG_LEN-2] == verifybuf[ RECV_MSG_LEN-2] && readbuf[ RECV_MSG_LEN-1] == verifybuf[ RECV_MSG_LEN-1]){
                float_to_byte temp_data_s;
                temp_data_s.data[0] = readbuf[7];
                temp_data_s.data[1] = readbuf[8];
                temp_data_s.data[2] = readbuf[9];
                temp_data_s.data[3] = readbuf[10];
                mcu_data.pit_angle =  temp_data_s.f_data;

                temp_data_s.data[0] = readbuf[11];
                temp_data_s.data[1] = readbuf[12];
                temp_data_s.data[2] = readbuf[13];
                temp_data_s.data[3] = readbuf[14];
                mcu_data.yaw_angle =  temp_data_s.f_data;

                temp_data_s.data[0] = readbuf[15];
                temp_data_s.data[1] = readbuf[16];
                temp_data_s.data[2] = readbuf[17];
                temp_data_s.data[3] = readbuf[18];
                mcu_data.pit_speed =  temp_data_s.f_data;

                temp_data_s.data[0] = readbuf[19];
                temp_data_s.data[1] = readbuf[20];
                temp_data_s.data[2] = readbuf[21];
                temp_data_s.data[3] = readbuf[22];
                mcu_data.yaw_speed =  temp_data_s.f_data;
                cout<<"                         mcu_data:"<<mcu_data.yaw_angle<<" "<<mcu_data.pit_angle<<" "<<mcu_data.yaw_speed
                   <<" "<<mcu_data.pit_speed<<endl;

                Mat canvas(260,800, CV_8UC3, Scalar(0, 0, 0));
                String s1=to_string((int)(mcu_data.yaw_angle*100)/100.0)+" ";
                String s2=to_string((int)(mcu_data.pit_angle*100)/100.0);
                String s3=to_string((int)(mcu_data.yaw_speed*100)/100.0);

                String s4=to_string((int)(mcu_data.pit_speed*100)/100.0);
                String s5=to_string(readbuf[18])+" "+to_string(readbuf[19])+" "+to_string(readbuf[20]);
                String s6=to_string(readbuf[21])+" "+to_string(readbuf[22])+" "+to_string(readbuf[23]);

                putText(canvas,s1,Point(10,40),FONT_HERSHEY_SIMPLEX,0.8,Scalar(255,255,255),2);
                putText(canvas,s2,Point(10,80),FONT_HERSHEY_SIMPLEX,0.8,Scalar(255,255,255),2);
                putText(canvas,s3,Point(10,120),FONT_HERSHEY_SIMPLEX,0.8,Scalar(255,255,255),2);
                putText(canvas,s4,Point(10,160),FONT_HERSHEY_SIMPLEX,0.8,Scalar(255,255,255),2);
                putText(canvas,s5,Point(10,200),FONT_HERSHEY_SIMPLEX,0.8,Scalar(255,255,255),2);
                putText(canvas,s6,Point(10,240),FONT_HERSHEY_SIMPLEX,0.8,Scalar(255,255,255),2);

                imshow("recv",canvas);


            }
        }
    }
    return readVec.size();
}


void Communication::UpdateData(double *p_y_err)
{
    Infantry.amorAttackmsg.yawErr=-(float)p_y_err[0];
    Infantry.amorAttackmsg.pitchErr=-(float)p_y_err[1];
}

void Communication::shoot_err(int shoot)
{
    Infantry.amorAttackmsg.shootFlag=shoot;
}

void Communication::upflag()
{
    Infantry.amorAttackmsg.holderFlag=0;
    //cout<<"holderflag:"<<Infantry.amorAttackmsg.holderFlag<<endl;
}
void Communication::downflag()
{
    Infantry.amorAttackmsg.holderFlag=1;
    //cout<<"holderflag:"<<Infantry.amorAttackmsg.holderFlag<<endl;
}
void Communication::RecvMcuData(double out_mcu_data[4],bool& IsRecv)
{
    int readVec_size = recvMsg();
    if(readVec_size == 0){IsRecv = false;}
    else
    {
        out_mcu_data[0] = mcu_data.yaw_angle;
        out_mcu_data[1] = mcu_data.yaw_speed;
        out_mcu_data[2] = mcu_data.pit_angle;
        out_mcu_data[3] = mcu_data.pit_speed;
        IsRecv = true;
    }

//    int timedelay = 5;
//    auto t1 = std::chrono::high_resolution_clock::now();
//    while (1) {//电控使用闲时中断，需要有延迟。不知道为何，usleep()没用
//        auto t2 = std::chrono::high_resolution_clock::now();
//        std::chrono::duration<double, std::milli> elapsed = t2 - t1;
//        if(elapsed.count() >= timedelay) break;
//    }

}

void Communication::communication(Infantry_Struct Infantry)
{

    sendMsg();

    int timedelay = 5;
    auto t1 = std::chrono::high_resolution_clock::now();
    while (1) {//电控使用闲时中断，需要有延迟。不知道为何，usleep()没用
        auto t2 = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> elapsed = t2 - t1;
        if(elapsed.count() >= timedelay) break;
    }
}
