#include <phantom_teleop_joystick.h>
#include <std_msgs/ByteMultiArray.h>
#include "LinuxDARwIn.h"

#define	c1DEC		10
#define _M_PI           3141
#define ToRad(x) ((x)*0.01745329252)  // *pi/180
#define ToDeg(x) ((x)*57.2957795131)  // *180/pi


using namespace Robot;
LinuxArbotixPro linux_arbotixpro("/dev/ttyUSB0");
ArbotixPro arbotixpro(&linux_arbotixpro);

//==============================================================================
// Constructor
//==============================================================================

PhantomTeleopJoystick::PhantomTeleopJoystick( void )
{
    ros::param::get( "NON_TELEOP", NON_TELEOP );
    ros::param::get( "PS3_DEABAND", DEADBAND );
    ros::param::get( "PS3_OFFSET", OFFSET );
    ros::param::get( "PS3_SCALE", SCALE );


    joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("/joy", 10, &PhantomTeleopJoystick::joyCallback,this);
    
    //if (NON_TELEOP == false){
            //arbotixpro.DEBUG_JOINTS = true;
	    if (arbotixpro.Connect() == true){
                timer_Write = nh_.createTimer(ros::Duration(0.04), boost::bind(&PhantomTeleopJoystick::Write_Read_CM530, this));
	    }
    //}
   

}

//==============================================================================
// Joystick call reading joystick topics
//==============================================================================

void PhantomTeleopJoystick::joyCallback( const sensor_msgs::Joy::ConstPtr &joy )
{
    ros::Time current_time = ros::Time::now();

    buttons = 0;
    leftX = 128;
    leftY = 128;
    rightX = 128;
    rightY = 128;

    if( joy->buttons[BIT8DO_BUTTON_ACTION_X] == 1 )
    {
     	 buttons += 8; 
    }
    if( joy->buttons[BIT8DO_BUTTON_ACTION_A] == 1 )
    {
     	 buttons += 32; 
    }
    if( joy->buttons[BIT8DO_BUTTON_ACTION_B] == 1 )
    {
     	 buttons += 1; 
    }
    if( joy->buttons[BIT8DO_BUTTON_ACTION_Y] == 1 )
    {
     	 buttons += 16; 
    }   
    if( joy->buttons[BIT8DO_BUTTON_REAR_LEFT_1] == 1 )
    {
     	 buttons += 128; 
    }
    if( joy->buttons[BIT8DO_BUTTON_REAR_RIGHT_1] == 1 )
    {
     	 buttons += 64; 
    }
    if( joy->buttons[BIT8DO_BUTTON_STICK_RIGHT] == 1 )
    {
     	 buttons += 2; 
    }

    
    if( joy->axes[PS3_AXIS_STICK_LEFT_LEFTWARDS] >  DEADBAND || joy->axes[PS3_AXIS_STICK_LEFT_LEFTWARDS] < -DEADBAND)
    {
	 //ROS_DEBUG("%.6f" ,joy->axes[PS3_AXIS_STICK_LEFT_LEFTWARDS]);
	
         leftX = mapa(joy->axes[PS3_AXIS_STICK_LEFT_LEFTWARDS],-1.0,1.0,0.0,253.0);
         //ROS_DEBUG("%.6f" ,joy->axes[PS3_AXIS_STICK_LEFT_LEFTWARDS] );
         //ROS_DEBUG("leftX %i", leftX);
    }
    if( joy->axes[PS3_AXIS_STICK_LEFT_UPWARDS] >  DEADBAND || joy->axes[PS3_AXIS_STICK_LEFT_UPWARDS] < -DEADBAND)
   {
         leftY = mapa(joy->axes[PS3_AXIS_STICK_LEFT_UPWARDS],-1.0,1.0,0.0,253.0);
    }
    if( joy->axes[PS3_AXIS_STICK_RIGHT_LEFTWARDS] >  DEADBAND || joy->axes[PS3_AXIS_STICK_RIGHT_LEFTWARDS] < -DEADBAND)
    {
         rightX = mapa(joy->axes[PS3_AXIS_STICK_RIGHT_LEFTWARDS],-1.0,1.0,0.0,253.0);
    }
    if( joy->axes[PS3_AXIS_STICK_RIGHT_UPWARDS] >  DEADBAND || joy->axes[PS3_AXIS_STICK_RIGHT_UPWARDS] < -DEADBAND)
    {
         rightY = mapa(joy->axes[PS3_AXIS_STICK_RIGHT_UPWARDS],-1.0,1.0,0.0,253.0);
    }

    checksum = (255 - (rightY+rightX+leftY+leftX+buttons)%256);

}
int PhantomTeleopJoystick::mapa(double x, double in_min, double in_max, double out_min, double out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int PhantomTeleopJoystick::makeword(int lowbyte, int highbyte,bool feed,int index) {
	unsigned short word;
	word = highbyte;
	word = word << 8;
	word = word + lowbyte;
        
	if(feed && (index == 0)){
           printf("%i\n",word );
	} 
	
	return (int)word;
}

void PhantomTeleopJoystick::Write_Read_CM530(){
	arbotixpro.TxRx_CM530(rightY,rightX,leftY,leftX,buttons,checksum,rxpacket);
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "phantom_joystick");
    PhantomTeleopJoystick phantomTeleopJoystick;

    //ros::spin();

/**    // our loop will publish at 10Hz
    ros::Rate loop_rate(5);
 
    ros::spin();
*/    

    ros::AsyncSpinner spinner(1); // Using 1 threads
    spinner.start();

    ros::Rate loop_rate(50); // 100 hz
    while ( ros::ok() )
    {  
        loop_rate.sleep();
    }
}
