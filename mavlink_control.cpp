/* Includes ------------------------------------------------------------------*/
#include "mavlink_control.h"
#include "joystick.h"
#include <thread>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <jsoncpp/json/json.h>
#include <jsoncpp/json/value.h>
/* Private define-------------------------------------------------------------*/
/* Private Typedef------------------------------------------------------------*/


typedef enum _sdk_process_state
{
    STATE_IDLE,

    STATE_CHECK_FIRMWARE_VERSION,
    STATE_SETTING_GIMBAL,
    STATE_SETTING_MESSAGE_RATE,

    STATE_SET_GIMBAL_OFF,
    STATE_SET_GIMBAL_ON,
    
    STATE_SET_GIMBAL_FOLLOW_MODE,

    STATE_SET_GIMBAL_ROTATION_CW_1,
    STATE_SET_GIMBAL_ROTATION_CCW_1,

    STATE_SET_GIMBAL_LOCK_MODE,

    STATE_SET_GIMBAL_ROTATION_CW_2,
    STATE_SET_GIMBAL_ROTATION_CCW_2,

    STATE_SET_GIMBAL_ROTATION_SPEED_CW,
    STATE_SET_GIMBAL_ROTATION_SPEED_CCW,
    
    STATE_MOVE_TO_ZERO,
    STATE_SWITCH_TO_RC,

    STATE_SET_RESET_MODE,

    STATE_SET_GIMBAL_REBOOT,

    STATE_DONE
}sdk_process_state_t; 


typedef struct 
{
    sdk_process_state_t state;
    uint64_t            last_time_send;
    uint64_t            timeout;
} sdk_process_t;

/* Private variable- ---------------------------------------------------------*/
static sdk_process_t sdk;
int joyRoll = 0, joyPitch = 0, joyYaw = 0, stopMoving = 0, resetGimbal = 0, sweep = 0, reboot = 0; 
int socket_desc;
bool stop = false;
float yawSpeed = 30;
float angularVel = 5.0;
int scale = 5;


// ------------------------------------------------------------------------------
//   Gimbal sample control and get data 
// ------------------------------------------------------------------------------
int Gimbal_initialize(int argc, char **argv)
{
	// --------------------------------------------------------------------------
	//   PARSE THE COMMANDS
	// --------------------------------------------------------------------------

	// Default input arguments
#ifdef __APPLE__
	char *uart_name = (char*)"/dev/tty.usbmodem1";
#else
	char *uart_name = (char*)"/dev/ttyUSB0";
#endif
	int baudrate = 115200;

    char *ip_address = "192.168.1.1";

	// do the parse, will throw an int if it fails
	parse_commandline(argc, argv, ip_address,uart_name, baudrate);

	// --------------------------------------------------------------------------
	//   PORT and THREAD STARTUP
	// --------------------------------------------------------------------------

	/*
	 * Instantiate a serial port object
	 *
	 * This object handles the opening and closing of the offboard computer's
	 * serial port over which it will communicate to an autopilot.  It has
	 * methods to read and write a mavlink_message_t object.  To help with read
	 * and write in the context of pthreading, it gaurds port operations with a
	 * pthread mutex lock.
	 *
	 */
	Serial_Port serial_port(uart_name, baudrate);


	/*
	 * Instantiate an autopilot interface object
	 *
	 * This starts two threads for read and write over MAVlink. The read thread
	 * listens for any MAVlink message and pushes it to the current_messages
	 * attribute.  The write thread at the moment only streams a heartbeat 1hz It's
	 * important that one way or another this program signals offboard mode exit,
	 * otherwise the vehicle will go into failsafe.
	 *
	 */
	Gimbal_Interface gimbal_interface(&serial_port);

	/*
	 * Setup interrupt signal handler
	 *
	 * Responds to early exits signaled with Ctrl-C.  The handler will command
	 * to exit offboard mode if required, and close threads and the port.
	 * The handler in this example needs references to the above objects.
	 *
	 */
	serial_port_quit        = &serial_port;
	gimbal_interface_quit 	= &gimbal_interface;
	signal(SIGINT, quit_handler);

	/*
	 * Start the port and Gimbal_interface
	 * This is where the port is opened, and read and write threads are started.
	 */
	serial_port.start();
    gimbal_interface.set_gimbal_reboot();
	gimbal_interface.start();
    const char *device;
    int js;

    size_t axis;
    thread th1(readJSON);

    Gimbal_setProperties(gimbal_interface);
    Gimbal_setMsgRate(gimbal_interface);
    Gimbal_goToZero(gimbal_interface);
	/// Process data 
    // cout << joyPitch << joyRoll << joyYaw << "\n";
	while (!gimbal_interface.get_flag_exit())
	{
		uint64_t time_display_ms = get_time_msec();
        uint64_t last_time_send_attitude_ms = 0;

		if (gimbal_interface.present())
		{
            // Reset time             
            sdk.timeout = get_time_usec();

            if(joyYaw == 0 && joyPitch == 0 && joyRoll == 0 && sweep == 0){
                Gimbal_stop(gimbal_interface);
            }
            if(resetGimbal == 1){
                Gimbal_goToZero(gimbal_interface);
                resetGimbal = 0;
            }
             if(reboot == 1){
                gimbal_interface.set_gimbal_reboot();
                reboot = 0;
             }
            // cout << joyPitch << joyRoll << joyYaw << stopMoving << "\n";

            if(joyYaw == -1){
                // cout << "yaw left";
                Gimbal_yawLeft(gimbal_interface);
            }
            else if(joyYaw == 1){
                // cout << "yaw right";
                Gimbal_yawRight(gimbal_interface);
            }
            
            if(joyRoll == -1){
                // cout << "roll left";
                Gimbal_rollLeft(gimbal_interface);
            }
            else if(joyRoll == 1){
                // cout << "roll right";
                Gimbal_rollRight(gimbal_interface);
            }

            if(joyPitch == -1){
                // cout << "pitch left";
                Gimbal_pitchDown(gimbal_interface);
            }
            else if(joyPitch == 1){
                // cout << "pitch right";
                Gimbal_pitchUp(gimbal_interface);
            }

            if(sweep == 1){
                Gimbal_startSweep(gimbal_interface);
            }
		}
        /* Update autopilot attitude for gimbal to reduce pan drift*/
        if (time_display_ms - last_time_send_attitude_ms > 20) // 50Hz
        {
            attitude3f_t autopilot_attitude = { 0.f, 0.f, 0.f};

            gimbal_interface.set_autopilot_attitude(autopilot_attitude);
        }

        usleep(1000);   // 1ms
	}

	// --------------------------------------------------------------------------
	//   THREAD and PORT SHUTDOWN
	// --------------------------------------------------------------------------

	/*
	 * Now that we are done we can stop the threads and close the port
	 */
	gimbal_interface.stop();
	serial_port.stop();

	// --------------------------------------------------------------------------
	//   DONE
	// --------------------------------------------------------------------------

	// woot!
	return 0;
}

// --------------------------------------------------------------------------
//   Paser gimbal info
// --------------------------------------------------------------------------

void Gimbal_displays(Gimbal_Interface &api)
{
	/*--------------------------------------------------------------------------
	  GET A MESSAGE
	--------------------------------------------------------------------------*/
	printf("READ SOME MESSAGES \n");
	printf("\n");

    gimbal_status_t gimbal_status = api.get_gimbal_status();
    printf("Got message gimbal status \n");

    if(gimbal_status.state == GIMBAL_STATE_OFF)
    {
        printf("Gimbal's status is OFF!\n");
    }
    else if(gimbal_status.state == GIMBAL_STATE_ON)
    {
          printf("Gimbal is operating\n");
    }
    else if(gimbal_status.state == GIMBAL_STATE_INIT)
    {
        printf("Gimbal is busy!\n");
    }
    else if(gimbal_status.state == GIMBAL_STATE_ERROR)
    {
        printf("Gimbal's status is error!\n");
    }

    imu_t imu;
    imu = api.get_gimbal_raw_imu();

	printf("Got message RAW IMU.\n");
	printf("\traw imu: xacc:%d, yacc:%d, zacc:%d, xgyro:%d, xgyro:%d, xgyro:%d(raw)\n", 
                                                    imu.accel.x, 
                                                    imu.accel.y, 
                                                    imu.accel.z,
                                                    imu.gyro.x,
                                                    imu.gyro.y,
                                                    imu.gyro.z);

	attitude3f_t mnt_orien;
    mnt_orien = api.get_gimbal_mount_orientation();

	printf("Got message Mount orientation.\n");
	printf("\torientation: pitch:%f, roll:%f, yaw:%f (degree)\n", 
                                                        mnt_orien.pitch, 
                                                        mnt_orien.roll, 
                                                        mnt_orien.yaw);

    attitude3d_t encoder;
    encoder = api.get_gimbal_encoder();

	printf("Got message Mount status \n");

    if(api.get_gimbal_config_mavlink_msg().enc_type_send)
    {
        printf("\tEncoder Count: pitch:%d, roll:%d, yaw:%d (Resolution 2^16)\n",
                                                            encoder.pitch, 
                                                            encoder.roll, 
                                                            encoder.yaw);
    }
    else
    {
        printf("\tEncoder Angle: pitch:%d, roll:%d, yaw:%d (Degree)\n",
                                                            encoder.pitch, 
                                                            encoder.roll, 
                                                            encoder.yaw);
    }



    gimbal_config_axis_t setting = api.get_gimbal_config_tilt_axis();

    printf("\tSETTING TILT: dir %d, speed_follow: %d, speed_control: %d\n", 
                                                            setting.dir,
                                                            setting.speed_follow,
                                                            setting.speed_control);

    gimbal_motor_control_t tilt;
    gimbal_motor_control_t roll;
    gimbal_motor_control_t pan;

    uint8_t output_filter, gyro_filter, gain;


    api.get_gimbal_motor_control(tilt, roll, pan, gyro_filter, output_filter, gain);
    printf("\tMOTOR_CONTROL: GYRO: %d, OUT %d, GAIN %d\n", gyro_filter, output_filter, gain);
    printf("\tTILT  stiff %d, hold: %d\n" , tilt.stiffness, tilt.holdstrength);
    printf("\tROLL  stiff %d, hold: %d\n" , roll.stiffness, roll.holdstrength);
    printf("\tPAN   stiff %d, hold: %d\n" , pan.stiffness, pan.holdstrength);

    /*Get gimbal limit information*/
    limit_angle_t limit_angle_pitch;
    limit_angle_t limit_angle_roll;
    limit_angle_t limit_angle_yaw;

    api.get_limit_angle_pitch(limit_angle_pitch);
    api.get_limit_angle_roll(limit_angle_roll);
    api.get_limit_angle_yaw(limit_angle_yaw);


    printf("Limit angle [Pitch]: upper_limit %d lower_limit %d\n", limit_angle_pitch.angle_max, limit_angle_pitch.angle_min);
    printf("Limit angle [Roll]: upper_limit %d lower_limit %d\n", limit_angle_roll.angle_max, limit_angle_roll.angle_min);
    printf("Limit angle [Yaw]: upper_limit %d lower_limit %d\n", limit_angle_yaw.angle_max, limit_angle_yaw.angle_min);


	printf("\n");
}

// ------------------------------------------------------------------------------
//   This example will demonstrate how to set gimbal mode 
//      and control gimbal in angle and speed mode
// ------------------------------------------------------------------------------

void Gimbal_checkVersion(Gimbal_Interface &onboard){
    fw_version_t fw = onboard.get_gimbal_version();
    printf("FW Version: %d.%d.%d.%s\n", fw.x, fw.y, fw.z, fw.type);
    // This firmware only apply for the firmware version from v7.x.x or above
    if(fw.x >= 7)
    {
        sdk.state = STATE_SETTING_GIMBAL;
    }
    else
    {
        printf("DO NOT SUPPORT FUNCTIONS. Please check the firmware version\n");
        printf("1. MOTOR CONTROL\n");
        printf("2. AXIS CONFIGURATION\n");
        printf("3. MAVLINK MSG RATE CONFIGURATION\n");
    }
}

void Gimbal_setProperties(Gimbal_Interface &onboard){
    gimbal_config_axis_t config = {0};

    config = {DIR_CW, 50, 10, 50, 10, 1};
    onboard.set_gimbal_config_tilt_axis(config);

    config = {DIR_CW, 50, 10, 50, 10, 1};
    onboard.set_gimbal_config_roll_axis(config);

    config = {DIR_CW, 50, 10, 50, 10, 1};
    onboard.set_gimbal_config_pan_axis(config);

    /*Delay 1ms*/
    usleep(1000);

    // Motor control likes: Stiffness, holdstrength, gyro filter, output filter and gain
    gimbal_motor_control_t tilt = {10, 40};
    gimbal_motor_control_t roll = {10, 40};
    gimbal_motor_control_t pan = {10, 40};
    onboard.set_gimbal_motor_control(tilt, roll, pan, 2, 4, 120);

    /*Delay 1ms*/
    usleep(1000);

    /*Set limit */
    limit_angle_t limit_angle_pitch = {-90, 45};
    limit_angle_t limit_angle_roll = {-45, 45};
    limit_angle_t limit_angle_yaw = {-320, 320};
    onboard.set_limit_angle_pitch(limit_angle_pitch);
    onboard.set_limit_angle_roll(limit_angle_roll);
    onboard.set_limit_angle_yaw(limit_angle_yaw);

    /*Delay 1ms*/
    usleep(1000);

    /*Set enable combine attitude from the aircraft*/
    /* Uncomment the line below to enable gimbal reduce pan drift */
    onboard.set_gimbal_combine_attitude(true);
}

void Gimbal_setMsgRate(Gimbal_Interface &onboard){
    uint8_t emit_heatbeat = 1;
    uint8_t status_rate = 10;
    uint8_t enc_value_rate = 10; 
    uint8_t enc_type_send = 0;  // Set type of encoder is angle
    uint8_t orien_rate = 50;
    uint8_t imu_rate = 10;
    
    printf("Set msg rate!\n");

    // configuration message. Note emit_heartbeat need to emit when using this gSDK. If not, the gSDK will waiting forever.
    onboard.set_gimbal_config_mavlink_msg(  emit_heatbeat, 
                                            status_rate, 
                                            enc_value_rate, 
                                            enc_type_send, 
                                            orien_rate,
                                            imu_rate);
}

void Gimbal_turnOff(Gimbal_Interface &onboard){
    if(onboard.get_gimbal_status().state == GIMBAL_STATE_ON)
    {
        // Turn off
        onboard.set_gimbal_motor_mode(TURN_OFF);
        printf("Set TURN_OFF! Current - mode %d & state: %d\n", onboard.get_gimbal_status().mode, onboard.get_gimbal_status().state);
        sdk.last_time_send = get_time_usec();
    }
}

void Gimbal_turnOn(Gimbal_Interface &onboard){
    // Check gimbal is on
    if(onboard.get_gimbal_status().mode == GIMBAL_STATE_OFF)
    {
        // Turn on gimbal
        onboard.set_gimbal_motor_mode(TURN_ON);
        printf("Set TURN_ON! Current - mode %d & state: %d\n", onboard.get_gimbal_status().mode, onboard.get_gimbal_status().state);
        sdk.last_time_send = get_time_usec();
    }
}

void Gimbal_lockMode(Gimbal_Interface &onboard){
    if(onboard.set_gimbal_lock_mode_sync() == MAV_RESULT_ACCEPTED) 
    {
        //Wait a moment about 5 seconds. Just see the effect
        if((get_time_usec() - sdk.last_time_send) > 10000000)
        {
            sdk.last_time_send = get_time_usec();
            printf("Set gimbal yaw LOCK mode accepted !\n");
        }
    } 
    else {
        printf("Set gimbal yaw LOCK mode!\n");
    }
}

void Gimbal_pitchUp(Gimbal_Interface &onboard){
    float speed_pitch  = angularVel;
    float speed_roll  = 0.0;
    float speed_yaw    = 0.0;
    int res = onboard.set_gimbal_rotation_sync(speed_pitch, speed_roll, speed_yaw, GIMBAL_ROTATION_MODE_SPEED);
    // printf("Pitch Up [%3.2f - %3.2f - %3.2f] [Result: %d]\n",speed_roll,speed_pitch, speed_yaw,res);
}                                                            
                                                                                                                                                       
void Gimbal_pitchDown(Gimbal_Interface &onboard){
    float speed_pitch  = -angularVel;
    float speed_roll  = 0.0;
    float speed_yaw    = 0.0;
    int res = onboard.set_gimbal_rotation_sync(speed_pitch, speed_roll, speed_yaw, GIMBAL_ROTATION_MODE_SPEED);
    // printf("Pitch Down [%3.2f - %3.2f - %3.2f] [Result: %d]\n",speed_roll,speed_pitch, speed_yaw,res);
}

void Gimbal_yawRight(Gimbal_Interface &onboard){
    float speed_pitch  = 0.0;
    float speed_roll  = 0.0;
    float speed_yaw    = angularVel;
    int res = onboard.set_gimbal_rotation_sync(speed_pitch, speed_roll, speed_yaw, GIMBAL_ROTATION_MODE_SPEED);
    // printf("Yaw Right [%3.2f - %3.2f - %3.2f] [Result: %d]\n",speed_roll,speed_pitch, speed_yaw,res);
}

void Gimbal_yawLeft(Gimbal_Interface &onboard){
    float speed_pitch  = 0.0;
    float speed_roll  = 0.0;
    float speed_yaw    = -angularVel;
    int res = onboard.set_gimbal_rotation_sync(speed_pitch, speed_roll, speed_yaw, GIMBAL_ROTATION_MODE_SPEED);
    // printf("Yaw Left [%3.2f - %3.2f - %3.2f] [Result: %d]\n",speed_roll,speed_pitch, speed_yaw,res);
}

void Gimbal_rollRight(Gimbal_Interface &onboard){
    float speed_pitch  = 0.0;
    float speed_roll  = angularVel;
    float speed_yaw    = 0.0;
    int res = onboard.set_gimbal_rotation_sync(speed_pitch, speed_roll, speed_yaw, GIMBAL_ROTATION_MODE_SPEED);
    // printf("Roll Right [%3.2f - %3.2f - %3.2f] [Result: %d]\n",speed_roll,speed_pitch, speed_yaw,res);
}

void Gimbal_rollLeft(Gimbal_Interface &onboard){
    float speed_pitch  = 0.0;
    float speed_roll  = -angularVel;
    float speed_yaw    = 0.0;
    int res = onboard.set_gimbal_rotation_sync(speed_pitch, speed_roll, speed_yaw, GIMBAL_ROTATION_MODE_SPEED);
    // printf("Roll Left [%3.2f - %3.2f - %3.2f] [Result: %d]\n",speed_roll,speed_pitch, speed_yaw,res);
}

void Gimbal_stop(Gimbal_Interface &onboard){
    // if(stop == false){
        // stop = true;
        float speed_pitch  = 0.0;
        float speed_roll  = 0.0;
        float speed_yaw    = 0.0;
        int res = onboard.set_gimbal_rotation_sync(speed_pitch, speed_roll, speed_yaw, GIMBAL_ROTATION_MODE_SPEED);
        // printf("Stop [%3.2f - %3.2f - %3.2f] [Result: %d]\n",speed_roll,speed_pitch, speed_yaw,res);
    // }
}

void Gimbal_goToZero(Gimbal_Interface &onboard){
    float setpoint_pitch  = 0.f;
    float setpoint_roll   = 0.f;
    float setpoint_yaw    = 0.f;
    
    /// Set command gimbal move
    int res = onboard.set_gimbal_rotation_sync(setpoint_pitch, setpoint_roll, setpoint_yaw, GIMBAL_ROTATION_MODE_ABSOLUTE_ANGLE);

    /* Check delta to make sure gimbal has move complete. */
    float delta_pitch_angle = fabsf(onboard.get_gimbal_mount_orientation().pitch - setpoint_pitch);
    float delta_roll_angle  = fabsf(onboard.get_gimbal_mount_orientation().roll - setpoint_roll);
    float delta_yaw_angle   = fabsf(onboard.get_gimbal_mount_orientation().yaw - setpoint_yaw);

    // Json::Value value;
    // std::ifstream paramsFile("params.json");
    // paramsFile >> value;
    // value["reset"] = 0;


    printf("Moving zero gimbal RYP [%3.2f - %3.2f - %3.2f] [Result: %d]\n",delta_pitch_angle,
                                                                                delta_roll_angle,
                                                                                delta_yaw_angle,
                                                                                res);

    // Check gimbal feedback COMMAND_ACK when sending MAV_CMD_DO_MOUNT_CONTROL
    if(res == MAV_RESULT_ACCEPTED) {
        //Wait a moment about 5 seconds. Just see the effect
        if((get_time_usec() - sdk.last_time_send) > 10000000)
        {
            // Reset time for the next step
            sdk.last_time_send = get_time_usec();            
        }
    }
}

void Gimbal_startSweep(Gimbal_Interface &onboard){
    float speed_pitch  = 0.f;
    float speed_roll  = 0.f;
    float speed_yaw    = yawSpeed;

    // Set command gimbal move
    int res = onboard.set_gimbal_rotation_sync(speed_pitch, speed_roll, speed_yaw, GIMBAL_ROTATION_MODE_SPEED);
    if(onboard.get_gimbal_mount_orientation().yaw > 45){
        yawSpeed = -10.0;
    }
    else if(onboard.get_gimbal_mount_orientation().yaw < -45){
        yawSpeed = 10.0;
    }
}

// ------------------------------------------------------------------------------
//   Parse Command Line
// ------------------------------------------------------------------------------
// throws EXIT_FAILURE if could not open the port
void
parse_commandline(int argc, char **argv, char *&ip_address, char *&uart_name, int &baudrate)
{
	// string for command line usage
	const char *commandline_usage = "usage: ./gSDK -i <ip_address> -d <devicename> -b <baudrate>";

	// Read input arguments
	for (int i = 1; i < argc; i++) { // argv[0] is "mavlink"

		// Help
		if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
			printf("%s\n",commandline_usage);
			throw EXIT_FAILURE;
		}

		// UART device ID
		if (strcmp(argv[i], "-d") == 0 || strcmp(argv[i], "--device") == 0) {
			if (argc > i + 1) {
				uart_name = argv[i + 1];

			} else {
				printf("%s\n",commandline_usage);
				throw EXIT_FAILURE;
			}
		}

        if(strcmp(argv[i], "-i") == 0){
            if(argc > i+1){
                ip_address = argv[i+1];
            } else {
				printf("%s\n",commandline_usage);
				throw EXIT_FAILURE;
			}
        }

		// Baud rate
		if (strcmp(argv[i], "-b") == 0 || strcmp(argv[i], "--baud") == 0) {
			if (argc > i + 1) {
				baudrate = atoi(argv[i + 1]);

			} else {
				printf("%s\n",commandline_usage);
				throw EXIT_FAILURE;
			}
		}
	}
	// end: for each input argument

	// Done!
	return;
}


// ------------------------------------------------------------------------------
//   Quit Signal Handler
// ------------------------------------------------------------------------------
// this function is called when you press Ctrl-C
void
quit_handler( int sig )
{
	printf("\n");
	printf("TERMINATING AT USER REQUEST\n");
	printf("\n");

	// autopilot interface
	try {
		gimbal_interface_quit->handle_quit(sig);
	}
	catch (int error){}

	// serial port
	try {
		serial_port_quit->handle_quit(sig);
	}
	catch (int error){}

	// end program here
	exit(0);
}


void readJSON(){
    Json::Reader readParams;
    Json::Value value;
    while (true)
    {
        try{
            std::ifstream paramsFile("params.json");
            paramsFile >> value;
            int startSweep = value["Sweep"].asInt();
            int stopGimbal = value["Stop"].asInt();
            int pitchUp = value["pitchUP"].asInt();
            int reset = value["reset"].asInt();
            int pitchDown = value["pitchDown"].asInt();
            int yawRight = value["yawRight"].asInt();
            int yawLeft = value["yawLeft"].asInt();
            int rebootGimbal = value["Reboot"].asInt();

            if(startSweep == 1){
                sweep = 1;
            }
            else{
                sweep = 0;
            }

            if(pitchUp == 1){
                joyPitch = 1;
            }
            else if(pitchDown == -1){
                joyPitch = -1;
            }
            else{
                joyPitch = 0;
            }
            
            if(yawLeft == -1){
                joyYaw = -1;
            }
            else if(yawRight == 1){
                joyYaw= 1;
            }
            else{
                joyYaw = 0;
            }

            if(stopGimbal == 1){
                joyPitch = 0;
                joyRoll = 0;
                joyYaw = 0;
            }
            if(reset == 1){
                resetGimbal = 1;
            }
            if(rebootGimbal == 1){
                reboot = 1;
            }
            paramsFile.close();
        }
        catch (exception e){
            cout << e.what() << "\n";
        }
    }
}

// ------------------------------------------------------------------------------
//   Main
// ------------------------------------------------------------------------------
int
main(int argc, char **argv)
{
	// This program uses throw, wrap one big try/catch here
	try
	{
		int result = Gimbal_initialize(argc,argv);
		return result;
	}

	catch ( int error )
	{
		fprintf(stderr,"mavlink_control threw exception %i \n" , error);
		return error;
	}

}

