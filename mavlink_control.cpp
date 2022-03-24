#include "mavlink_control.h"
#include "joystick.h"
#include <thread>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <jsoncpp/json/json.h>
#include <jsoncpp/json/value.h>

// ------------------------------------------------------------------------------
//   Initializing Gimbal
// ------------------------------------------------------------------------------
gimbalControl::gimbalControl(int argc, char **argv)
{
	char *uart_name = (char*)"/dev/ttyUSB0";
	int baudrate = 115200;

	parse_commandline(argc, argv, uart_name, baudrate);

	Serial_Port serial_port(uart_name, baudrate);

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

    // gimbal_interface.set_gimbal_reboot();
	gimbal_interface.start();
    const char *device;
    // int js;
    thread th1(readJSON);

    Gimbal_setProperties(gimbal_interface);
    Gimbal_setMsgRate(gimbal_interface);
    stat.errorCode = Gimbal_goToZero(gimbal_interface);
	/// Process data 
    stat.ready = 1;
    cout << "Gimbal Initialized" << "\n";
    cout << "Starting Control loop" << "\n";
    gimbalControlLoop(gimbal_interface);
    gimbal_interface.stop();
	serial_port.stop();
}


void gimbalControl::gimbalControlLoop(Gimbal_Interface &gimbal_interface){
	while (!gimbal_interface.get_flag_exit())
	{
		uint64_t time_display_ms = get_time_msec();
        uint64_t last_time_send_attitude_ms = 0;

		if (gimbal_interface.present())
		{
            // Reset time             
            sdk.timeout = get_time_usec();

            if(control.pitch == 0 && control.yaw == 0 && swp.sweep == false){
                stat.errorCode = Gimbal_stop(gimbal_interface);
            }
            
            if(control.reset){
                stat.errorCode = Gimbal_goToZero(gimbal_interface);
                control.reset = false;
            }
            
            if(control.reboot){
                gimbal_interface.set_gimbal_reboot();
                control.reboot = false;
             }

            if(control.yaw == -1){
                // cout << "yaw left";
                stat.errorCode = Gimbal_yawLeft(gimbal_interface);
            }
            else if(control.yaw == 1){
                // cout << "yaw right";
                stat.errorCode = Gimbal_yawRight(gimbal_interface);
            }

            if(control.pitch == -1){
                // cout << "pitch left";
                stat.errorCode = Gimbal_pitchDown(gimbal_interface);
            }
            else if(control.pitch == 1){
                // cout << "pitch right";
                stat.errorCode = Gimbal_pitchUp(gimbal_interface);
            }

            if(swp.sweep){
                stat.errorCode = Gimbal_startSweep(gimbal_interface);
            }
            getGimbalPos(gimbal_interface);
		}
        /* Update autopilot attitude for gimbal to reduce pan drift*/
        if (time_display_ms - last_time_send_attitude_ms > 20) // 50Hz
        {
            attitude3f_t autopilot_attitude = { 0.f, 0.f, 0.f};

            gimbal_interface.set_autopilot_attitude(autopilot_attitude);
        }

        usleep(1000);   // 1ms
	}
}

// --------------------------------------------------------------------------
//   Paser gimbal info
// --------------------------------------------------------------------------

void gimbalControl::Gimbal_displays(Gimbal_Interface &api)
{
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

    // printf("\tSETTING TILT: dir %d, speed_follow: %d, speed_control: %d\n", 
    //                                                         setting.dir,
    //                                                         setting.speed_follow,
    //                                                         setting.speed_control);

    gimbal_motor_control_t tilt;
    gimbal_motor_control_t roll;
    gimbal_motor_control_t pan;

    uint8_t output_filter, gyro_filter, gain;


    api.get_gimbal_motor_control(tilt, roll, pan, gyro_filter, output_filter, gain);
    // printf("\tMOTOR_CONTROL: GYRO: %d, OUT %d, GAIN %d\n", gyro_filter, output_filter, gain);
    // printf("\tTILT  stiff %d, hold: %d\n" , tilt.stiffness, tilt.holdstrength);
    // printf("\tROLL  stiff %d, hold: %d\n" , roll.stiffness, roll.holdstrength);
    // printf("\tPAN   stiff %d, hold: %d\n" , pan.stiffness, pan.holdstrength);

    /*Get gimbal limit information*/
    limit_angle_t limit_angle_pitch;
    limit_angle_t limit_angle_roll;
    limit_angle_t limit_angle_yaw;

    api.get_limit_angle_pitch(limit_angle_pitch);
    api.get_limit_angle_roll(limit_angle_roll);
    api.get_limit_angle_yaw(limit_angle_yaw);


    // printf("Limit angle [Pitch]: upper_limit %d lower_limit %d\n", limit_angle_pitch.angle_max, limit_angle_pitch.angle_min);
    // printf("Limit angle [Roll]: upper_limit %d lower_limit %d\n", limit_angle_roll.angle_max, limit_angle_roll.angle_min);
    // printf("Limit angle [Yaw]: upper_limit %d lower_limit %d\n", limit_angle_yaw.angle_max, limit_angle_yaw.angle_min);


	printf("\n");
}

void gimbalControl::Gimbal_setProperties(Gimbal_Interface &onboard){
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

void gimbalControl::Gimbal_setMsgRate(Gimbal_Interface &onboard){
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

int gimbalControl::Gimbal_pitchUp(Gimbal_Interface &onboard){
    float speed_pitch  = angularVel;
    float speed_roll  = 0.0;
    float speed_yaw    = 0.0;
    int res = onboard.set_gimbal_rotation_sync(speed_pitch, speed_roll, speed_yaw, GIMBAL_ROTATION_MODE_SPEED);
    return res;
    // printf("Pitch Up [%3.2f - %3.2f - %3.2f] [Result: %d]\n",speed_roll,speed_pitch, speed_yaw,res);
}                                                            
                                                                                                                                                       
int gimbalControl::Gimbal_pitchDown(Gimbal_Interface &onboard){
    float speed_pitch  = -angularVel;
    float speed_roll  = 0.0;
    float speed_yaw    = 0.0;
    int res = onboard.set_gimbal_rotation_sync(speed_pitch, speed_roll, speed_yaw, GIMBAL_ROTATION_MODE_SPEED);
    // printf("Pitch Down [%3.2f - %3.2f - %3.2f] [Result: %d]\n",speed_roll,speed_pitch, speed_yaw,res);
    return res;
}

int gimbalControl::Gimbal_yawRight(Gimbal_Interface &onboard){
    float speed_pitch  = 0.0;
    float speed_roll  = 0.0;
    float speed_yaw    = angularVel;
    int res = onboard.set_gimbal_rotation_sync(speed_pitch, speed_roll, speed_yaw, GIMBAL_ROTATION_MODE_SPEED);
    // printf("Yaw Right [%3.2f - %3.2f - %3.2f] [Result: %d]\n",speed_roll,speed_pitch, speed_yaw,res);
    return res;
}

int gimbalControl::Gimbal_yawLeft(Gimbal_Interface &onboard){
    float speed_pitch  = 0.0;
    float speed_roll  = 0.0;
    float speed_yaw    = -angularVel;
    int res = onboard.set_gimbal_rotation_sync(speed_pitch, speed_roll, speed_yaw, GIMBAL_ROTATION_MODE_SPEED);
    // printf("Yaw Left [%3.2f - %3.2f - %3.2f] [Result: %d]\n",speed_roll,speed_pitch, speed_yaw,res);
    return res;
}

int gimbalControl::Gimbal_stop(Gimbal_Interface &onboard){
        float speed_pitch  = 0.0;
        float speed_roll  = 0.0;
        float speed_yaw    = 0.0;
        int res = onboard.set_gimbal_rotation_sync(speed_pitch, speed_roll, speed_yaw, GIMBAL_ROTATION_MODE_SPEED);
        return res;
}

void gimbalControl::getGimbalPos(Gimbal_Interface &gimbal_interface){
    pos.pitch = gimbal_interface.get_gimbal_mount_orientation().pitch;
    pos.roll  = gimbal_interface.get_gimbal_mount_orientation().roll;
    pos.yaw   = gimbal_interface.get_gimbal_mount_orientation().yaw;
}

int gimbalControl::Gimbal_goToZero(Gimbal_Interface &onboard){
    float setpoint_pitch  = 0.f;
    float setpoint_roll   = 0.f;
    float setpoint_yaw    = 0.f;
    
    /// Set command gimbal move
    int res = onboard.set_gimbal_rotation_sync(setpoint_pitch, setpoint_roll, setpoint_yaw, GIMBAL_ROTATION_MODE_ABSOLUTE_ANGLE);

    /* Check delta to make sure gimbal has move complete. */
    float delta_pitch_angle = fabsf(onboard.get_gimbal_mount_orientation().pitch - setpoint_pitch);
    float delta_roll_angle  = fabsf(onboard.get_gimbal_mount_orientation().roll - setpoint_roll);
    float delta_yaw_angle   = fabsf(onboard.get_gimbal_mount_orientation().yaw - setpoint_yaw);

    // printf("Moving zero gimbal RYP [%3.2f - %3.2f - %3.2f] [Result: %d]\n",delta_pitch_angle,
    //                                                                             delta_roll_angle,
    //                                                                             delta_yaw_angle,
    //                                                                             res);

    // Check gimbal feedback COMMAND_ACK when sending MAV_CMD_DO_MOUNT_CONTROL
    if(res == MAV_RESULT_ACCEPTED) {
        //Wait a moment about 5 seconds. Just see the effect
        if((get_time_usec() - sdk.last_time_send) > 10000000)
        {
            // Reset time for the next step
            sdk.last_time_send = get_time_usec();            
        }
    }
    return res;
}

int gimbalControl::Gimbal_startSweep(Gimbal_Interface &onboard){
    float speed_pitch  = 0.f;
    float speed_roll  = 0.f;
    // float speed_yaw    = swp.speed;

    if(onboard.get_gimbal_mount_orientation().yaw > 45){
        swp.speed = -1 * swp.speed;
    }
    else if(onboard.get_gimbal_mount_orientation().yaw < -45){
        swp.speed = swp.speed;
    }
    // Set command gimbal move
    int res = onboard.set_gimbal_rotation_sync(speed_pitch, speed_roll, swp.speed, GIMBAL_ROTATION_MODE_SPEED);
    return res;
    
}

// ------------------------------------------------------------------------------
//   Parse Command Line
// ------------------------------------------------------------------------------
// throws EXIT_FAILURE if could not open the port
void parse_commandline(int argc, char **argv, char *&uart_name, int &baudrate)
{
	// string for command line usage
	const char *commandline_usage = "usage: ./gSDK -d <devicename> -b <baudrate>";
	// Read input arguments
	for (int i = 1; i < argc; i++) { 

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
	return;
}


// ------------------------------------------------------------------------------
//   Quit Signal Handler
// ------------------------------------------------------------------------------
// this function is called when you press Ctrl-C
void quit_handler( int sig )
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

            int startSweep = value["SweepControls"]["Sweep"].asInt();
            swp.speed = value["SweepControls"]["Speed"].asFloat();

            int stopGimbal = value["GimbalControls"]["Stop"].asInt();
            int pitchUp = value["GimbalControls"]["pitchUP"].asInt();
            
            int pitchDown = value["GimbalControls"]["pitchDown"].asInt();
            int yawRight = value["GimbalControls"]["yawRight"].asInt();
            int yawLeft = value["GimbalControls"]["yawLeft"].asInt();
            
            int rebootGimbal = value["Reboot"].asInt();
            int reset = value["reset"].asInt();
            
            if(startSweep == 1){
                swp.sweep = true;
            }
            else{
                swp.sweep = false;
            }

            if(pitchUp == 1){
                control.pitch = 1;
            }
            else if(pitchDown == -1){
                control.pitch = -1;
            }
            else{
                control.pitch = 0;
            }
            
            if(yawLeft == -1){
                control.yaw = -1;
            }
            else if(yawRight == 1){
                control.yaw= 1;
            }
            else{
                control.yaw = 0;
            }

            if(stopGimbal == 1){
                control.yaw = 0;
                control.pitch = 0;
                control.stop = true;
            }
            if(reset == 1){
                control.reset = true;
            }
            if(rebootGimbal == 1){
                control.reboot = true;
            }
            paramsFile.close();
            // update JSON 
            value["GimbalStatus"]["error"] = stat.errorCode;
            value["GimbalStatus"]["roll"] = pos.roll;
            value["GimbalStatus"]["pitch"] = pos.pitch;
            value["GimbalStatus"]["yaw"] = pos.yaw;
            value["GimbalStatus"]["ready"] = stat.ready;
            Json::FastWriter writer;
            const std::string json_file = writer.write(value);
            ofstream out("params.json");  
            out << json_file;
            out.close();
        }
        catch (exception e){
            cout << e.what() << "\n";
        }
    }
}

// ------------------------------------------------------------------------------
//   Main
// ------------------------------------------------------------------------------
int main(int argc, char **argv)
{
	try
	{
		gimbalControl gimbalControl(argc,argv);
	}

	catch ( int error )
	{
		fprintf(stderr,"Gimbal Exception %i: Gimbal disconnected \n" , error);
		return error;
	}

}

