#ifndef LMZUART__HPP
#define LMZUART__HPP

#include <wiringPi.h>
#include <wiringSerial.h>
#include <string>
#include <iostream>
#include <unistd.h>
// #include <cstdlib>

using namespace std;

class LMZUart
{
private:
    int fd;
    string Device;
    int bauds;
public:
    LMZUart(string Device, int bauds){
        this->Device = Device;
        this->bauds = bauds;
    }
    void setBaud(int bauds){
        if(this->bauds == bauds)
            return ;
        this->bauds = bauds;
        this->openUart();
    }
    void closeUart(){
        serialClose(this->fd);
    }
    void setDevice(string Dev){
        if(this->Device == Dev)
            return ;
        this->Device = Dev;
        this->openUart();
    }
    void sendMessage(string str){
        for (int i = 0; i < (int)str.length(); i++)
        {
            serialPutchar(this->fd,(unsigned char)str[i]);
        }
    }
    
    string receiveMessage(int waitSec = 5){

        string D ;
        for(int i = 0 ; i < waitSec * 10 ; i++){
            int length  = serialDataAvail(this->fd) ;
            if(length > 0){
                string Data;
                for(int i = 0 ; i < length ; i++){
                    Data += (char)serialGetchar(this->fd);
                }
                serialFlush(this->fd);
                return Data;
            }
            usleep(100*1000);  // 延时100ms
        }
        return D;
    }

    // 打开串口
    bool openUart(){
        this->fd = serialOpen((char *)(this->Device.c_str()), this->bauds);
        if( 0 > this->fd){
                cout << "Device: " << this->Device.c_str() << ", Baud = " << this->bauds << ", fd =" << this->fd << endl;
                cerr << "Uart open failed!!! Open again. Times: 3" <<  endl ;
            for(int i = 0 ; i < 3 ; i++){
                this->fd = serialOpen((char *)(this->Device.c_str()),this->bauds) ;
                if(this->fd > 0){
                    cout << "Uart open successed!!!" <<  endl ;
                    return true;
                }
                cerr << "Uart open failed!!! Open again. Times: " <<  3-i-1 <<  endl;
            }
            return false;
        }else{
            cout << "Uart open successed!!!" <<  endl ;
            return true;
        }
    }

    // 初始化wiringPi的GPIO
    static bool GPIOInitAll(){
        if(-1==wiringPiSetup()){
            cerr << "WiringPi setup error!!! Try again! Times: 3" <<  endl;
            for(int i = 3 ; i > 0 ; i--){
                if(-1 != wiringPiSetup()){
                    break ;
                }
                cerr<< "WiringPi setup error!!! Try again! Time: " <<  i-1 <<  endl;
            }
            cerr << "error--->Can not setup wiringPi!!! Will quit!!!" <<  endl ;
            return false;
        }
        cout << "WiringPi setup successed!!!" <<  endl ;
        return true;
    }
};


#endif
