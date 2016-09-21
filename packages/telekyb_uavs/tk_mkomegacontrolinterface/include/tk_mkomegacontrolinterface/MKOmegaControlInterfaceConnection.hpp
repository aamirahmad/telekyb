/*
 * MKOmegaControlInterfaceConnection.hpp
 *
 *  Created on: Nov 23, 2011
 *      Author: mriedel
 */

#ifndef MKOMEGACONTROLINTERFACECONNECTION_HPP_
#define MKOMEGACONTROLINTERFACECONNECTION_HPP_

#include <telekyb_defines/telekyb_defines.hpp>
#include <telekyb_defines/MKDefines.hpp>

// Options
#include <tk_mkomegacontrolinterface/MKOmegaControlInterfaceConnectionOptions.hpp>


#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdarg.h>
#include <stdbool.h>

#include <sys/stat.h>
#include <sys/types.h>

#include <errno.h>
#include <fcntl.h>

#include <string.h>
#include <termios.h>
#include <pthread.h>
#include <unistd.h>




namespace TELEKYB_NAMESPACE
{
  
struct mikrokopter_channel_s {
  int fd;
  bool avail;
  uint8_t buf[64], r, w; /* read ring buffer */
  bool start;
  bool escape;
  uint8_t msg[64], len; /* last message */
};

struct Myfoo_IMU {
    double acc_x, acc_y,acc_z;
    double gyro_roll, gyro_pitch, gyro_yaw;
    uint32_t stamp;
    
    Myfoo_IMU(){
      acc_x=0;
      acc_y=0;
      acc_z=0;
      gyro_roll=0;
      gyro_pitch=0;
      gyro_yaw=0;
      stamp=0;
    }
};

struct Myfoo_MOTOR {
    double velocity;
    double pwm;
    double current;
};

struct Myfoo_VEL {
    uint16_t v1, v2, v3, v4;
};
  
void* mikroRec(void* arg);  

class MKOmegaControlInterfaceConnection {
  
  private:
    
    ros::NodeHandle mainNodeHandle; // Programs main Nodehandle
    
    ros::Publisher imuPublisher;
    ros::Publisher motorPublisher;
    ros::Publisher batteryPublisher;
    
    ros::Time lastImuTime;
    
    double estRoll;
    double estPitch;
    
    //Options
    MKOmegaControlInterfaceConnectionOptions options;
    
    // output: IMU;
    Myfoo_IMU IMU;
    bool _newIMUAvailable;
    
    // output: Battery level;
    double BATTERY;
    bool _newBATTERYAvailable;
    
    // output: motor angular velocities;
    Myfoo_MOTOR MOTOR[4];
    bool _newMOTORAvailable;
    
    // input: motor command;
    Myfoo_VEL VEL;
    
    bool _driftEstimationDone;

    
    bool terminateThread;
    
    pthread_mutex_t biglock;
    
    struct mikrokopter_channel_s chan;
    
    // full path of the serial device
    std::string devicePathName;
    
    // string expected as output
    std::string idVersionString;
    
    // last command sent
    uint16_t lastuPtrs[4];

    // private methods and routines
    // open a tty channel
    int mk_open_tty(const char *device, speed_t baud);
    
    // receives a message on an open channel
    int mk_recv_msg(bool complete);
    
    // sends one message passed as a string
    bool mk_send_msg(const char *fmt, ...);
    
    // subroutine of mk_send_msg
    static void mk_encode(char x, char **buf);
    
    // this function features an infinite loop that receive messages on the serial
    // and tries to interpret them as imu, battery level or motor data messages
    void receiveLoop();
    
    // this function establishes the connection with the MK and checks if the 
    bool initializeMK();
    
        
  public:
    
    MKInt motorstate;

    
    /// \brief Constructor. Don't use! use \b findConnection(...) instead
    /// \param[in] devicePath full path to the serial device
    /// \param[in] id ID of the MK
    /// \param[in] version software version flashed on the MK
    MKOmegaControlInterfaceConnection(const std::string& devicePath, int id, int version);
    
    /// \brief Standard destructor.
    virtual ~MKOmegaControlInterfaceConnection();
    
    /// This function starts a connection with the mikrokopter and create
    /// an infinite loop to read imu motor and battery data
    void* startReceiveLoop();
    
    /// \brief Asks ID and flashed software version to the MK and waits for the answer.
    /// \param[out] output in this string the answer is written.
    /// \return 0 if it fails, != 0 otherwise
    bool idVersion(std::string& output);
   
    /// \brief Turns motor on.
    /// \return 0 if it fails, != 0 otherwise
    bool turnMotorOn();
    
    /// \brief Turns motor off.
    /// \return 0 if it fails, != 0 otherwise
    bool turnMotorOff();
    
    /// \brief Turns motor init.
    /// \return 0 if it fails, != 0 otherwise
    bool turnMotorInit();
    
    /// \brief Returns motor state.
    MKInt getMotorState();
    
    /// \brief Sends rotational velocities for the motors.
    /// \param[in] uPtrs an array of four uint16_t* representing the desired rotational velocities of the propellers in rad/s
    void sendCommand(double uPtrs[4]);

    /// \brief Requires the MK to perform an onboard gyro drift estimation and waits until finished.
    /// \param[in] period a uint8_t representing the length of the caslibration in s
    /// \return \b false if it fails, \b true otherwise
    /// \todo put a timer and return error if the gyro drift is not performed within it.
    bool performOnboardGyroDriftEstimation(uint8_t period);
    
    /// \brief Requires the MK to perform an onboard linear acceleration drift estimation and waits until finished.
    /// \param[in] period a uint8_t representing the length of the caslibration in s
    /// \return \b false if it fails, \b true otherwise
    /// \todo put a timer and return error if the linear acceleration drift is not performed within it.
    bool performOnboardAccDriftEstimation(uint8_t period);
    
    /// \brief Requires the activation of a constant stream of IMU data.
    /// \param[in] period a uint32_t representing the time period between two transmissions in us
    /// \return 0 if it fails, != 0 otherwise
    bool imuQuery(uint32_t period);
    
    /// \brief Requires the activation of a constant stream of battery level data.
    /// \param[in] period a uint32_t representing the time period between two transmissions in us
    /// \return 0 if it fails, != 0 otherwise
    bool batteryLevelQuery(uint32_t period);
    
    /// \brief Requires the activation of a constant stream of motor data.
    /// \param[in] period a uint32_t representing the time period between two transmissions in us
    /// \return 0 if it fails, != 0 otherwise
    bool motorDataQuery(uint32_t period);
    
    /// \brief Requires the MK to beep at given frequency.
    /// \param[in] freqHz a uint16_t representing the frequency of the beep
    /// \return 0 if it fails, != 0 otherwise
    bool beep(uint16_t freqHz);
    
    /// \brief Looks for a serial device that behaves like a MK.
    /// \param[in] serialDeviceDirectory folder in which the method look for the serial connection
    /// \param[in] serialDeviceNameRegEx name of the serial connection
    /// \param[in] id ID of the MK
    /// \param[in] version software version flashed on the MK
    /// \return a pointer to the MKOmegaControlInterfaceConnection (NULL if does not find a valid connection)
    static MKOmegaControlInterfaceConnection* findConnection(
		    const std::string& serialDeviceDirectory,
		    const std::string& serialDeviceNameRegEx,
		    int id,
		    int version);
    
    /// return weather new battery level has been received since last time that it has been read
    bool newBATTERYAvailable(){
      return _newBATTERYAvailable;
    }
    
    /// return weather new imu data has been received since last time that it has been read
    bool newIMUAvailable(){
      return _newIMUAvailable;
    }
    
    /// return weather new imu data has been received since last time that it has been read
    bool newMOTORAvailable(){
      return _newMOTORAvailable;
    }
    
    /// return weather new battery level has been received since last time that it has been read
    double getBATTERY(){
      _newBATTERYAvailable = false;
      return BATTERY;
    }
    
    /// gets last IMU received
    void getIMU(Myfoo_IMU &out){
      pthread_mutex_lock(&biglock);
      out = IMU;
      _newIMUAvailable = false;
      pthread_mutex_unlock(&biglock);
    }
    
    /// gets last motor data received
    void getMOTOR(Myfoo_MOTOR* out){
      pthread_mutex_lock(&biglock);
      out[0] = MOTOR[0];
      out[1] = MOTOR[1];
      out[2] = MOTOR[2];
      out[3] = MOTOR[3];
      _newMOTORAvailable = false;
      pthread_mutex_unlock(&biglock);
    }

};

} // namespace

#endif /* MKOMEGACONTROLINTERFACECONNECTION_HPP_ */
