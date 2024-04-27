#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/NavSatFix.h"
#include <cstdlib>
#include <cmath>

//we use these global variables so that we can access them in every function
double lat;
double lon;
double alt;
double const a = 6378137;
double const b = 6356752;
double const e_square = 1- (b*b)/(a*a);

void fixCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    //The subscriber's callback updates our global variables to the new values recieved. This way we can see the new values in the main function, where we process the data and publish it.
    lat = msg->latitude;
    lon = msg->longitude;
    alt = msg->altitude;
}
double Ntheta(){
    return a/( sqrt( 1- (e_square * sin(lat*M_PI/180) * sin(lat*M_PI/180)) ) );
}
double Ntheta_r(double lat_r){
    return a/( sqrt( 1- (e_square * sin(lat_r*M_PI/180) * sin(lat_r*M_PI/180)) ) );
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "gps_to_odom");
    ros::NodeHandle nodeHandle;

    //we define the publisher of the topic /gps_odom and subscribe to the /fix topic.
    // The subscriber calls the callback method when it recieves data.
    ros::Publisher pub = nodeHandle.advertise<nav_msgs::Odometry>("/gps_odom", 1);      //publisher
    ros::Subscriber sub = nodeHandle.subscribe("/fix", 1, fixCallback);                 //subscriber

    ros::Rate loop_rate(1);

    //we define all the parameters we will need throughout the node
    double lat_r,lon_r,alt_r;       //INPUT parameters
    double lat_r_lon_r_rad[2];      //transformation in radiants (parameters)
    double lat_lon_rad[2];          //trasformation in radiants
    double reference_ECEF[3];       //parameters transformed in ECEF
    double ECEF[3];                 //ECEF
    double ENU[3];                  //ENU
    double angleShift = 130.0 * M_PI / 180.0;
    double ENU_shifted[3];          //ENU shifted
    double ENU_prec[3];             //ENU_prec
    double roll_pitch_yaw[3];       //roll pitch yaw
    double yaw_prec;
    double cr_sr_cp_sp_cy_sy[6];    //angles for the quaternion
    double quaternion[4];           //quaternion (x,y,z,w)

    //we get the static parameters set in the launchfile, these are the lat, lon and alt of the first value from the gps
    nodeHandle.param("lat_r", lat_r, 0.0); 
    nodeHandle.param("lon_r", lon_r, 0.0);
    nodeHandle.param("alt_r", alt_r, 0.0);

    //we set the global variables to the values we got from the parameters
    lat = lat_r; 
    lon = lon_r;
    alt = alt_r;

    //we convert these values from degrees to radiants, since the <cmat> library needs radiants for sin() and cos() functions
    lat_r_lon_r_rad[0] = lat_r*M_PI/180;
    lat_r_lon_r_rad[1] = lon_r*M_PI/180;

    //we get the reference values in ECEF
    reference_ECEF[0] = ( Ntheta_r(lat_r) + alt_r ) * cos(lat_r_lon_r_rad[0]) * cos(lat_r_lon_r_rad[1]);
    reference_ECEF[1] = ( Ntheta_r(lat_r) + alt_r ) * cos(lat_r_lon_r_rad[0]) * sin(lat_r_lon_r_rad[1]);
    reference_ECEF[2] = ( Ntheta_r(lat_r) * (1-e_square) + alt_r ) * sin(lat_r_lon_r_rad[0]);

    ROS_INFO("lat_r: %f", reference_ECEF[0]);
    ROS_INFO("lon_r: %f", reference_ECEF[1]);
    ROS_INFO("alt_r: %f", reference_ECEF[2]);

    //we set the starting values of ENU and ECEF
    ENU[0] =0;
    ENU[1] =0;
    ENU[2] =0;
    ENU_shifted[0] =0;
    ENU_shifted[1] =0;
    ENU_shifted[2] =0;
    ECEF[0] = reference_ECEF[0];
    ECEF[1] = reference_ECEF[1];
    ECEF[2] = reference_ECEF[2];
    yaw_prec=0;


    while(ros::ok()) {
        nav_msgs::Odometry private_message;

        //we convert these values from degrees to radiants, just like we did for the reference values.
        lat_lon_rad[0] = M_PI * lat / 180;
        lat_lon_rad[1] = M_PI * lon / 180;

        //we use the formulas to calculate ECEF. The Ntheta() functions has the global variables and directly calculates the value needed for each one.
        ECEF[0] = ( Ntheta() + alt ) * cos(lat_lon_rad[0]) * cos(lat_lon_rad[1]);
        ECEF[1] = ( Ntheta() + alt ) * cos(lat_lon_rad[0]) * sin(lat_lon_rad[1]);
        ECEF[2] = ( Ntheta() * (1-e_square) + alt ) * sin(lat_lon_rad[0]);

        ENU_prec[0] = ENU_shifted[0];
        ENU_prec[1] = ENU_shifted[1];
        ENU_prec[2] = ENU_shifted[2];

        //we use the formula to calculate ENU, we commented ENU[2] because ??????
        ENU[0] = (-sin(lat_r_lon_r_rad[1]))*(ECEF[0] - reference_ECEF[0]) 
            + (cos(lat_r_lon_r_rad[1]))*(ECEF[1] - reference_ECEF[1]) 
            + 0;
        ENU[1] = (-sin(lat_r_lon_r_rad[0])*cos(lat_r_lon_r_rad[1]))*(ECEF[0] - reference_ECEF[0]) 
            + (-sin(lat_r_lon_r_rad[0])*sin(lat_r_lon_r_rad[1]))*(ECEF[1] - reference_ECEF[1]) 
            + (cos(lat_r_lon_r_rad[0]))*(ECEF[2] - reference_ECEF[2]);

        /*ENU[2] = (cos(lat_r_lon_r_rad[0])*cos(lat_r_lon_r_rad[1]))*(ECEF[0] - reference_ECEF[0]) 
            + (cos(lat_r_lon_r_rad[0])*sin(lat_r_lon_r_rad[1]))*(ECEF[1] - reference_ECEF[1]) 
            + (sin(lat_r_lon_r_rad[0]))*(ECEF[2] - reference_ECEF[2]);*/

        ENU_shifted[0] = cos(angleShift) * ENU[0] - sin(angleShift) * ENU[1];
        ENU_shifted[1] = sin(angleShift) * ENU[0] + cos(angleShift) * ENU[1];
        ENU_shifted[2] = ENU[2];

        private_message.pose.pose.position.x = ENU_shifted[0];
        private_message.pose.pose.position.y = ENU_shifted[1];
        private_message.pose.pose.position.z = 0;
        //private_message.pose.pose.position.z = ENU[2];

        //roll_pitch_yaw[0] = atan((ENU[0]-ENU_prec[0]) / (ENU[2]-ENU_prec[2]));
        //roll_pitch_yaw[1] = atan((ENU[2]-ENU_prec[2]) / (ENU[1]-ENU_prec[1]));
        roll_pitch_yaw[0] = 0;
        roll_pitch_yaw[1] = 0;
        if(ENU_shifted[0]-ENU_prec[0] ==0){
            roll_pitch_yaw[2] = yaw_prec;
        }
        else{
            roll_pitch_yaw[2] = atan2((ENU_shifted[1]-ENU_prec[1]) , (ENU_shifted[0]-ENU_prec[0]));
            yaw_prec = roll_pitch_yaw[2];
        }

        cr_sr_cp_sp_cy_sy[0] = cos(roll_pitch_yaw[0]/2);
        cr_sr_cp_sp_cy_sy[1] = sin(roll_pitch_yaw[0]/2);
        cr_sr_cp_sp_cy_sy[2] = cos(roll_pitch_yaw[1]/2);
        cr_sr_cp_sp_cy_sy[3] = sin(roll_pitch_yaw[1]/2);
        cr_sr_cp_sp_cy_sy[4] = cos(roll_pitch_yaw[2]/2);
        cr_sr_cp_sp_cy_sy[5] = sin(roll_pitch_yaw[2]/2);

        /*quaternion[0] = cr_sr_cp_sp_cy_sy[1]*cr_sr_cp_sp_cy_sy[2]*cr_sr_cp_sp_cy_sy[4]   
                        - cr_sr_cp_sp_cy_sy[0]*cr_sr_cp_sp_cy_sy[3]*cr_sr_cp_sp_cy_sy[5] ;
        quaternion[1] = cr_sr_cp_sp_cy_sy[0]*cr_sr_cp_sp_cy_sy[3]*cr_sr_cp_sp_cy_sy[4]   
                        + cr_sr_cp_sp_cy_sy[1]*cr_sr_cp_sp_cy_sy[2]*cr_sr_cp_sp_cy_sy[5] ;*/
        quaternion[2] = cr_sr_cp_sp_cy_sy[0]*cr_sr_cp_sp_cy_sy[2]*cr_sr_cp_sp_cy_sy[5]   
                        - cr_sr_cp_sp_cy_sy[1]*cr_sr_cp_sp_cy_sy[3]*cr_sr_cp_sp_cy_sy[4] ;
        quaternion[3] = cr_sr_cp_sp_cy_sy[0]*cr_sr_cp_sp_cy_sy[2]*cr_sr_cp_sp_cy_sy[4]   
                        + cr_sr_cp_sp_cy_sy[1]*cr_sr_cp_sp_cy_sy[3]*cr_sr_cp_sp_cy_sy[5] ;

        //private_message.pose.pose.orientation.x = quaternion[0];
        //private_message.pose.pose.orientation.y = quaternion[1];
        private_message.pose.pose.orientation.x = 0;
        private_message.pose.pose.orientation.y = 0;
        private_message.pose.pose.orientation.z = quaternion[2];
        private_message.pose.pose.orientation.w = quaternion[3];

        //we publish the new message
        pub.publish(private_message);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
