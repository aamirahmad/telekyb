/*
 * MKOmegaControlInterfaceConnection.cpp
 *
 *  Created on: Nov 23, 2011
 *  Updated: Dec, 2015
 *      Author: mriedel
 *      Coauthors: pstegagno, byueksel
 */

#include <telekyb_base/ROS/ROSModule.hpp>

#include <tk_mkomegacontrolinterface/MKOmegaControlInterfaceConnection.hpp>

#include <telekyb_serial/SerialHelper.hpp>

#include <tk_draft_msgs/TKSmallImu.h>

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float32.h"

// Boost Filesystem and Regex to Find Serial
#include <boost/regex.hpp>
#include <boost/filesystem.hpp>

namespace fs = boost::filesystem;

// ADDR
#define ALLADDR 'a'
#define FCADDR 'b'
#define NCADDR 'c'
#define MK3MAGADDR 'd'

// Commands OUT
#define SETVALUE_CMD_OUT 's'
#define UPDATEVALUE_CMD_OUT 'u'
#define ONLY_CMD_OUT 'b'
#define ACTIVEDATAIDS_OUT 'l'

// Commands IN
#define VALUE_IN 'S'
#define ACTIVEDATA_IN 'D'
#define ACTIVEDATAIDS_IN 'L'


// Performance Parameters in Options?
#define SYNC_SLEEP_USEC 25000 // 25ms every 4th command at 120hz
#define MKVALUE_RESENDS 40

#define BUFFER_SIZE 256

#define SERIALDEVICE_MAXRETRIES 25

namespace TELEKYB_NAMESPACE
{
  

/* Open a serial port, configure to the given baud rate */
int MKOmegaControlInterfaceConnection::mk_open_tty(const char *device, speed_t baud)
{
    struct termios t;
    int f, fd;
    
    /* open non-blocking before configuration */
    fd = open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) return fd;
    if (!isatty(fd)) {
        errno = ENOTTY;
        return -1;
    }
    
    /* configure line discipline */
    if (tcgetattr(fd, &t)) return -1;
    
    t.c_iflag = IGNBRK;
    t.c_oflag = 0;
    t.c_lflag = 0;
    t.c_cflag = CS8 | CREAD | CLOCAL;
    t.c_cc[VMIN] = 0;
    t.c_cc[VTIME] = 15; /* 500ms read timeout */
    
    if (cfsetospeed(&t, baud)) return -1;
    if (cfsetispeed(&t, baud)) return -1;
    
    if (tcsetattr(fd, TCSANOW, &t)) return -1;
    
    /* set back to blocking reads */
    f = fcntl(fd, F_GETFL, 0);
    if (f != -1)
        fcntl(fd, F_SETFL, f & ~O_NONBLOCK);
    
    /* discard any pending data */
    tcflush(fd, TCIFLUSH);
    
    return fd;
}


/* returns: 0: timeout/incomplete, -1: error, 1: complete msg */
int MKOmegaControlInterfaceConnection::mk_recv_msg(bool complete)
{
    ssize_t s;
    uint8_t c;

    if (chan.fd < 0) return 0;
    
    do {
        /* read up to the buffer limit or the read pointer. One cannot use readv()
         * to fill the full buffer because of a Linux bug where readv() is a read()
         * loop, and this interacts badly with VMIN and VTIME ... */
        if (chan.r > chan.w)
            s = chan.r - 1 - chan.w;
        else
            s = sizeof(chan.buf) - chan.w;
        
        s = read(chan.fd, chan.buf + chan.w, s);
        if (s < 0 && errno != EINTR && errno != EAGAIN)
            return -1;
        else if (s == 0)
            return 0;
        else if (s > 0)
            chan.w = (chan.w + s) % sizeof(chan.buf);
        chan.avail = false;
        
        while(chan.r != chan.w) {
            c = chan.buf[chan.r];
            chan.r = (chan.r + 1) % sizeof(chan.buf);
            
            switch(c) {
                case '^':
                    chan.start = true;
                    chan.escape = false;
                    chan.len = 0;
                    break;
                    
                case '$':
                    if (!chan.start) break;
                    
                    chan.start = false;
                    return 1;
                    
                case '!':
                    chan.start = false;
                    break;
                    
                case '\\':
                    chan.escape = true;
                    break;
                    
                default:
                    if (!chan.start) break;
                    if (chan.len >= sizeof(chan.msg)) {
                        chan.start = false; break;
                    }
                    
                    if (chan.escape) {
                        c = ~c; chan.escape = false;
                    }
                    chan.msg[chan.len++] = c;
                    break;
            }
        }
    } while(complete);
    
    return 0;
}



/* --- mk_send_msg --------------------------------------------------------- */
bool MKOmegaControlInterfaceConnection::mk_send_msg(const char *fmt, ...)
{
  
    va_list ap;
    ssize_t s;
    char buf[64], *w, *r;
    char c;
    
    if (chan.fd < 0) return true;
    
    w = buf;
    
    va_start(ap, fmt);
    *w++ = '^';
    while((c = *fmt++)) {

        while (w - buf > sizeof(buf)-8 /* 8 = worst case (4 bytes escaped) */) {
            do {
                s = write(chan.fd, buf, w - buf);
            } while (s < 0 && errno == EINTR);
            if (s < 0) return false;
            
            if (s > 0 && s < w - buf) memmove(buf, buf + s, w - buf - s);
            w -= s;
        }
        
        switch(c) {
            case '%': {
                switch(*fmt++) {
                    case '1':
			mk_encode(va_arg(ap, int/*promotion*/), &w);
                        break;
                        
                    case '2': {
                        uint16_t x = va_arg(ap, int /*promotion*/);
                        mk_encode((x >> 8) & 0xff, &w);
                        mk_encode(x & 0xff, &w);
                        break;
                    }
                    
                    case '4': {
                        uint32_t x = va_arg(ap, uint32_t);
                        mk_encode((x >> 24) & 0xff, &w);
                        mk_encode((x >> 16) & 0xff, &w);
                        mk_encode((x >> 8) & 0xff, &w);
                        mk_encode(x & 0xff, &w);
                        break;
                    }
                }
                break;
            }
            
            default:
                mk_encode(c, &w);
        }
    }
    *w++ = '$';
    va_end(ap);
    
    r = buf;
    while (w > r) {
        do {
            s = write(chan.fd, r, w - r);
        } while (s < 0 && errno == EINTR);
        if (s < 0) return false;
        
        r += s;
    }
    
    return true;
}

// subroutine of mk_send_msg
void MKOmegaControlInterfaceConnection::mk_encode(char x, char** buf)
{
  switch (x) {
        case '^': case '$': case '\\': case '!':
            *(*buf)++ = '\\';
            x = ~x;
    }
    *(*buf)++ = x;
}	

MKOmegaControlInterfaceConnection::MKOmegaControlInterfaceConnection(const std::string& devicePath, int id, int version)
	    : mainNodeHandle(ROSModule::Instance().getMainNodeHandle())
{
  motorstate=MotorState::Off;
  _newBATTERYAvailable = false;
  _newIMUAvailable = false;
  _newMOTORAvailable = false;
  devicePathName = devicePath;
  std::stringstream ss;
  ss << "mkfl" << id<< "." << version;
  idVersionString = ss.str();
  ROS_INFO("Requested MK         %s on serial connection %s.", idVersionString.c_str(), devicePathName.c_str());
  pthread_t p;
  pthread_mutex_init(&biglock, NULL);
  terminateThread = false;
  if (!initializeMK()){
    ROS_ERROR("MKOmegaControlInterfaceConnection::MKOmegaControlInterfaceConnection: initialization of MK failed. Shutting down.");
    ros::shutdown();
  }
  
//   imuPublisher = mainNodeHandle.advertise<tk_draft_msgs::TKSmallImu>("imu",10);
  imuPublisher = mainNodeHandle.advertise<tk_draft_msgs::TKSmallImu>(options.tImuStatePublisher->getValue(),10);
  motorPublisher = mainNodeHandle.advertise<std_msgs::Float32MultiArray>("motor", 10);
  batteryPublisher = mainNodeHandle.advertise<std_msgs::Float32>("battery", 10);
    
  pthread_create(&p,NULL, mikroRec,this);
}

MKOmegaControlInterfaceConnection::~MKOmegaControlInterfaceConnection()
{
    terminateThread = true;
    this->turnMotorOff();
    this->imuQuery((uint32_t){0});
    this->motorDataQuery((uint32_t){0});
    this->batteryLevelQuery((uint32_t){0});
    close(chan.fd);
}


bool MKOmegaControlInterfaceConnection::initializeMK(){
  chan.r=0;
  chan.w=0;
    
//     chan.fd=mk_open_tty("/dev/ttyUSB0", B115200);
//   chan.fd=mk_open_tty(devicePathName.c_str(), B115200);
  
  
  if (options.tBaudRate->getValue()=="B115200"){
    chan.fd=mk_open_tty(devicePathName.c_str(), B115200);
    ROS_INFO("Opening the serial channel at B115200");
  }
  else if (options.tBaudRate->getValue()=="B230400") { 
    chan.fd=mk_open_tty(devicePathName.c_str(), B230400);
    ROS_INFO("Opening the serial channel at B230400");
  }
  else if (options.tBaudRate->getValue()=="B500000") {
    chan.fd=mk_open_tty(devicePathName.c_str(), B500000);
    ROS_INFO("Opening the serial channel at B500000");
  }
  else{
    ROS_ERROR("Wrong baudrate is assigned. Choose the baudrate matching to the one on board of your flight controller. Current options are B115200, B230400, B500000");
    return false;
  }

  if (chan.fd<0){
    ROS_ERROR("USB is not recognized, check open_tty parameters.");
    return false;
  }
  
  std::string receivedIDVersion;
  idVersion(receivedIDVersion); 
  if (receivedIDVersion.compare(0, idVersionString.length(), idVersionString) != 0){
    ROS_ERROR("ID VERSION values does not match.\n Specified in the launch file: %s\n Received on serial connection: %s", idVersionString.c_str(), receivedIDVersion.c_str());
    return false;
  }
  else {
    receivedIDVersion = receivedIDVersion.substr(0, idVersionString.length());
    ROS_INFO("Succesfully found MK %s on serial connection %s", receivedIDVersion.c_str(), devicePathName.c_str());
  }
  
  turnMotorOff();
  
  if (imuQuery((uint32_t)options.tImuDataPeriodus->getValue())){
    ROS_INFO("imu requested at %d ms", options.tImuDataPeriodus->getValue()/1000);
  }
  if (motorDataQuery((uint32_t)options.tMotorDataPeriodus->getValue())){
    ROS_INFO("motor data requested at %d ms", options.tMotorDataPeriodus->getValue()/1000);
  }
  if (batteryLevelQuery((uint32_t)options.tBatteryDataPeriodus->getValue())){
    ROS_INFO("battery data requested at %d ms", options.tBatteryDataPeriodus->getValue()/1000);
  }
  ROS_INFO("MK initialized correctly.");
  
  lastImuTime = ros::Time(0);
  estRoll = 0.0;
  estPitch = 0.0;

  return true;
}
  
  
void MKOmegaControlInterfaceConnection::receiveLoop()
{
    while(!terminateThread){
        uint8_t* msg;
        uint8_t len;
        int16_t v16;
        uint16_t u16;
        
        while (mk_recv_msg(true)==1){
            msg = chan.msg;
            len = chan.len;
            switch(*msg++) {
                case 'I': /* IMU data - µs */
                    if (len == 14) {
		        uint8_t seq;
			tk_draft_msgs::TKSmallImu rosmsg;
                        pthread_mutex_lock(&biglock);
			
			seq = (*msg++);
                        
                        v16 = ((int16_t)(*msg++) << 8);
                        v16 |= ((uint16_t)(*msg++) << 0);
                        IMU.acc_x = v16/1000.;
                        
                        v16 = ((int16_t)(*msg++) << 8);
                        v16 |= ((uint16_t)(*msg++) << 0);
                        IMU.acc_y = v16/1000.;
                        
                        v16 = ((int16_t)(*msg++) << 8);
                        v16 |= ((uint16_t)(*msg++) << 0);
                        IMU.acc_z = v16/1000.;
                        
                        v16 = ((int16_t)(*msg++) << 8);
                        v16 |= ((uint16_t)(*msg++) << 0);
                        IMU.gyro_roll = v16/1000.;
                        
                        v16 = ((int16_t)(*msg++) << 8);
                        v16 |= ((uint16_t)(*msg++) << 0);
                        IMU.gyro_pitch = v16/1000.;
                        
                        v16 = ((int16_t)(*msg++) << 8);
                        v16 |= ((uint16_t)(*msg++) << 0);
                        IMU.gyro_yaw = v16/1000.;
			_newIMUAvailable = true;

			rosmsg.angular_velocity.x 	= IMU.gyro_roll ;
			rosmsg.angular_velocity.y 	= IMU.gyro_pitch ;
			rosmsg.angular_velocity.z 	= -IMU.gyro_yaw ;		// this is coming somehow with wrong sign from the lowlevel TODO: ask antony
			rosmsg.linear_acceleration.x  = IMU.acc_x;
			rosmsg.linear_acceleration.y  = IMU.acc_y;
			rosmsg.linear_acceleration.z  = IMU.acc_z;
					
			// ========== here complementary filter =============
			double ACC_GAIN = 0.02;
			// get dt:
			double dt = (ros::Time::now() - lastImuTime).toSec();
			lastImuTime = ros::Time::now();		
			// compelemtary filter burak
			estRoll += (float)IMU.gyro_roll  * dt;    // Angle around the Y-axis
			estPitch += (float)IMU.gyro_pitch   * dt; // Angle around the X-axis
			
			estRoll = estRoll * (1-ACC_GAIN) + (atan2f((float)IMU.acc_y, (float)IMU.acc_z)) * ACC_GAIN;
			estPitch = estPitch * (1-ACC_GAIN) - (atan2f((float)IMU.acc_x, (float)IMU.acc_z)) * ACC_GAIN;

			rosmsg.orientation.x=estRoll;
			rosmsg.orientation.y=estPitch;
			rosmsg.orientation.z=0.0;
			
			pthread_mutex_unlock(&biglock);
			
			imuPublisher.publish(rosmsg);
                    }
                    break;

                case 'M': /* Motor data */
                    if (len == 9) {
                        uint8_t id;
                        uint8_t seq;
			std_msgs::Float32MultiArray rosmsg;
			rosmsg.data.clear();
                        pthread_mutex_lock(&biglock);
                        
			seq = (*msg++);
			
                        id = ((*msg++) & 0x7) -1;
                        if (id < 0 || id > 3) {
                            pthread_mutex_unlock(&biglock);
                            printf("bad motor id: %d\n", id);
                            break;
                        }
                        
                        u16 = ((uint16_t)(*msg++) << 8);
                        u16 |= ((uint16_t)(*msg++) << 0);
                        if (u16 > 0) {
                            MOTOR[id].velocity = 1000000./u16;
                        } else {
                            MOTOR[id].velocity = 0.;
                        }
                        
                        u16 = ((uint16_t)(*msg++) << 8);
                        u16 |= ((uint16_t)(*msg++) << 0);
                        MOTOR[id].pwm = 100.*u16/1024.;
                        
                        u16 = ((uint16_t)(*msg++) << 8);
                        u16 |= ((uint16_t)(*msg++) << 0);
                        MOTOR[id].current = u16/1000.;
			_newMOTORAvailable= true;
			    
			for (int i = 0; i < 4; i++)
			{
			    rosmsg.data.push_back(MOTOR[i].velocity);
			}
			
                        pthread_mutex_unlock(&biglock);
			
			motorPublisher.publish(rosmsg);
                        
                    }
                    break;
                    
                case 'B': /* Battery level */
                    if (len == 4) {
//                         uint8_t id;
			uint8_t seq;
			std_msgs::Float32 rosmsg;
// 			rosmsg.data.clear();
                        pthread_mutex_lock(&biglock);
			seq = (*msg++);
                        
                        u16 = ((uint16_t)(*msg++) << 8);
                        u16 |= ((uint16_t)(*msg++) << 0);
                        BATTERY = (double){u16/1000.};

			_newBATTERYAvailable = true;
			
			rosmsg.data = BATTERY;
                        
                        pthread_mutex_unlock(&biglock);
			
			batteryPublisher.publish(rosmsg);
                    }
                    break;
		case 'Z': /* gyro drift estimation done */
                    _driftEstimationDone = true;
                    break;
            }
        }
    }
}

// This is the main routine which is run in the pthread
void* mikroRec(void* arg){
    return ((MKOmegaControlInterfaceConnection*)arg)->startReceiveLoop();
}


// PUBLIC METHODS FROM HERE
void* MKOmegaControlInterfaceConnection::startReceiveLoop(){
  receiveLoop();
}

bool MKOmegaControlInterfaceConnection::idVersion(std::string& output){
  bool answerIdVersion = false;
  mk_send_msg("?");
  while(!answerIdVersion){
    uint8_t* msg;
    uint8_t len;
    std::stringstream ss;
    
    while (mk_recv_msg(true)==1 && !answerIdVersion){
      msg = chan.msg;
      len = chan.len;
      switch(*msg++) {
	case '?': /* IMU data - µs */
	  if (len > 5) {
	    pthread_mutex_lock(&biglock);
	    ss << msg;
	    output = ss.str();
	    answerIdVersion = true;
	    pthread_mutex_unlock(&biglock);
	  }
	  break;
      }
    }
  }
  return true;
}

bool MKOmegaControlInterfaceConnection::turnMotorOn(){
  motorstate=MotorState::On;
  return mk_send_msg("g");
}

bool MKOmegaControlInterfaceConnection::turnMotorInit(){
  motorstate=MotorState::Init;
  return mk_send_msg("g");
}

bool MKOmegaControlInterfaceConnection::turnMotorOff(){
  motorstate=MotorState::Off;
  return mk_send_msg("x");
}

MKInt MKOmegaControlInterfaceConnection::getMotorState(){
  return motorstate;
}


// send command
void MKOmegaControlInterfaceConnection::sendCommand(double uPtrs[4])
{
  // write on Serial
  uint16_t sp[4];


  pthread_mutex_lock(&biglock);

  
  VEL.v1=uPtrs[0];
  VEL.v2=uPtrs[1];
  VEL.v3=uPtrs[2];
  VEL.v4=uPtrs[3];
  
  if (VEL.v1> 16.) {
    sp[0] = (uint16_t){1e6/VEL.v1};
  } else {
    /* invalid vel.: send low velocity for safety */
    sp[0] = (uint16_t){65500};
  }
  
  if (VEL.v2> 16.) {
    sp[1] = (uint16_t){1e6/VEL.v2};
  } else {
    /* invalid vel.: send low velocity for safety */
    sp[1] = (uint16_t){65500};
  }
  
  if (VEL.v3> 16.) {
    sp[2] = (uint16_t){1e6/VEL.v3};
  } else {
    /* invalid vel.: send low velocity for safety */
    sp[2] = (uint16_t){65500};
  }
  
  if (VEL.v4> 16.) {
    sp[3] = (uint16_t){1e6/VEL.v4};
  } else {
    /* invalid vel.: send low velocity for safety */
    sp[3] = (uint16_t){65500};
  }
  
  // send tata only when motor state is on
  if (motorstate==MotorState::On) {
//     std::cout <<" setpoints " << sp[0]<< " " << sp[1] << " " << sp[2] << " " << sp[3] << std::endl;
    mk_send_msg("w%2%2%2%2", sp[0], sp[1], sp[2], sp[3]);
  }
  

  pthread_mutex_unlock(&biglock);
  
  
  // after sending the command, save the last command
  lastuPtrs[0] = uPtrs[0];
  lastuPtrs[1] = uPtrs[1];
  lastuPtrs[2] = uPtrs[2];
  lastuPtrs[3] = uPtrs[3];
}

bool MKOmegaControlInterfaceConnection::performOnboardGyroDriftEstimation(uint8_t period){
  if(!mk_send_msg("zg%1", /*(unsigned char)*/period)){
    ROS_ERROR("Unable to perform Onboard Gyro Drift Estimation.");
    return false;
  };
  ROS_INFO("MK will perform Onboard Gyro Drift Estimation for %d second(s).", period);
  _driftEstimationDone = false;
  while(!_driftEstimationDone){
    usleep(1000);
  }
  ROS_INFO("Onboard Gyro Drift Estimation was succesfull.");
  _driftEstimationDone = false;
  return true;
}

bool MKOmegaControlInterfaceConnection::performOnboardAccDriftEstimation(uint8_t period){
  if(!mk_send_msg("za%1", /*(unsigned char)*/period)){
    ROS_ERROR("Unable to perform Onboard Linear Acceleration Drift Estimation.");
    return false;
  };
  ROS_INFO("MK will perform Onboard Linear Acceleration Drift Estimation for %d second(s).", period);
  _driftEstimationDone = false;
  while(!_driftEstimationDone){
    usleep(1000);
  }
  ROS_INFO("Onboard Linear Acceleration Drift Estimation was succesfull.");
  _driftEstimationDone = false;
  return true;
}

bool MKOmegaControlInterfaceConnection::imuQuery(uint32_t period){
  return mk_send_msg("i%4", period);
}

bool MKOmegaControlInterfaceConnection::batteryLevelQuery(uint32_t period){
  return mk_send_msg("b%4", period);
}

bool MKOmegaControlInterfaceConnection::motorDataQuery(uint32_t period){
  return mk_send_msg("m%4", period);
}

bool MKOmegaControlInterfaceConnection::beep(uint16_t freqHz){
  return mk_send_msg("~%2", freqHz);  
}


// void MKOmegaControlInterfaceConnection::ComplementaryFilter(short accData[3], short gyrData[3], float *pitch, float *roll)
// {
//     float pitchAcc, rollAcc;               
//  
//     // Integrate the gyroscope data -> int(angularSpeed) = angle
//     *pitch += (float)gyrData[0]  * dt; // Angle around the X-axis
//     *roll -= (float)gyrData[1]  * dt;    // Angle around the Y-axis
//  
// 	// Turning around the X axis results in a vector on the Y-axis
//         pitchAcc = atan2f((float)accData[1], (float)accData[2]);
//         *pitch = *pitch * 0.98 + pitchAcc * 0.02;
//  
// 	// Turning around the Y axis results in a vector on the X-axis
//         rollAcc = atan2f((float)accData[0], (float)accData[2]);
//         *roll = *roll * 0.98 + rollAcc * 0.02;
// } 

// Find Connetion that meets conditions
MKOmegaControlInterfaceConnection* MKOmegaControlInterfaceConnection::findConnection(
		const std::string& serialDeviceDirectory,
		const std::string& serialDeviceNameRegEx,
		int id,
		int version)
{
  MKOmegaControlInterfaceConnection* connection = NULL;
  
  std::stringstream ss;
  ss << serialDeviceDirectory << serialDeviceNameRegEx;

  if (!fs::exists(ss.str())) {
    ROS_ERROR_STREAM(ss.str() << "does not exist.");
    return connection;
  }
  
  connection = new MKOmegaControlInterfaceConnection(ss.str(), id, version);
  return connection;
}




}

