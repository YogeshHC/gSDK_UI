
#include <iostream>
#include <stdio.h>
#include <cstdlib>
#include <unistd.h>
#include <cmath>
#include <string.h>
#include <inttypes.h>
#include <fstream>
#include <signal.h>
#include <time.h>
#include <sys/time.h>
#include <ardupilotmega/mavlink.h>
#include "gimbal_interface.h"
#include "serial_port.h"

using std::string;
using namespace std;

typedef struct {
    uint64_t last_time_send;
    uint64_t timeout;
} sdk_process_t;

typedef struct{
    int pitch;
    int yaw;
    bool reset;
    bool stop;
    bool reboot;
} controls;

typedef struct{
    bool sweep;
    float speed;
} sweep;

typedef struct{
    int ready;
    int errorCode;
} status;

typedef struct{
    float roll;
    float pitch;
    float yaw;
} att;

Gimbal_Interface *gimbal_interface_quit;
Serial_Port *serial_port_quit;
static sdk_process_t sdk;
static controls control;
static sweep swp;
static status stat;
static att pos;
float angularVel = 5.0;

void quit_handler( int sig );
void parse_commandline(int argc, char **argv, char *&uart_name, int &baudrate);
void readJSON();

class gimbalControl{
    public:
        gimbalControl(int argc, char **argv);

    private:
        void gimbalControlLoop(Gimbal_Interface &gimbal_interface);
        void Gimbal_setProperties(Gimbal_Interface &gimbal_interface);
        void Gimbal_setMsgRate(Gimbal_Interface &gimbal_interface);
        void Gimbal_displays(Gimbal_Interface &gimbal_interface);
        void getGimbalPos(Gimbal_Interface &gimbal_interface);
        
        int Gimbal_pitchUp(Gimbal_Interface &gimbal_interface);
        int Gimbal_pitchDown(Gimbal_Interface &gimbal_interface);
        int Gimbal_yawRight(Gimbal_Interface &gimbal_interface);
        int Gimbal_yawLeft(Gimbal_Interface &gimbal_interface);
        int Gimbal_stop(Gimbal_Interface &gimbal_interface);
        int Gimbal_goToZero(Gimbal_Interface &gimbal_interface);
        int Gimbal_startSweep(Gimbal_Interface &gimbal_interface);
};







