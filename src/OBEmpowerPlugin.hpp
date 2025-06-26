#ifndef OBEMPOWERPLUGIN_HPP_
#define OBEMPOWERPLUGIN_HPP_

#define	_USE_MATH_DEFINES
#include "OSUtils.hpp"
#include <AngleAndComsumerPlugin.h>
#include <string>
#include <set>
#include <mutex>
#include <thread>
#include <memory>
//#include <tcAdsClient.h>
// #include <PluginCommon.hpp>

// Code modified from the original OBEmpowerPlugin.hpp file
// for use on Raspberry Pi (Unix) and UDP communication to OSLv2 in Personify
// port: 12345

// OSLv2 sends ankle angle and current ankle torque (fixed size packet with float ankleAngle_rad and float ankleTorque_Nm)
// CEINMS-RT sends float torqueSetpoint_Nm
// port: 12346
// both localhost (same hardware)

#include <atomic>

// For UDP sockets
#include <sys/socket.h>
#include <netinet/in.h> // For sockaddr_in, IPPROTO_UDP
#include <arpa/inet.h>  // For inet_addr

// UDP Communication Definitions
#define OSLV2_UDP_RECV_PORT 12345 
#define OSLV2_UDP_SEND_PORT 12346 
#define OSLV2_IP_ADDRESS "127.0.0.1" // Localhost IP

#define EMPOWER_MAX_TORQUE	30.0f	// Nm
#define EMPOWER_MIN_TORQUE -30.0f	// Nm

// Structure for data received from OSLv2
// This ensures consistent byte order and size, similar to Python's struct.pack/unpack
struct Oslv2DataPacket {
    float ankleAngle_rad;
    float ankleTorque_Nm;
};

// Structure for command sent to OSLv2
struct Oslv2CommandPacket {
    float torqueSetpoint_Nm;
};


#ifdef WIN32
// While we're targeting Unix, keeping WIN32 guards for completeness
	class __declspec(dllexport) OBEmpowerPlugin : public AngleAndComsumerPlugin {
#endif
#ifdef UNIX
	class OBEmpowerPlugin : public AngleAndComsumerPlugin{
	#endif

	// Private Variables:
	std::vector<std::string> _muscleNames;
	std::vector<std::string> _dofNames;
	std::map<std::string, double> _jointAngle, _jointTorqueFromCEINMS, _jointTorqueFromExternalOrID, _jointStiffness, _muscleActivation, _muscleForce, _muscleFiberLength, _muscleFiberVelocity, _position, _velocity, _acceleration, _grf;
	double _outputTimeStamp, _dataTimeStamp;
	std::string _currState;
	std::map<std::string, std::map<std::string, double>> _subjectMuscleParameters;

	// UDP variables (replaces EtherCAT variables)
	int _udpSocketFd; // File descriptor for the UDP socket
	struct sockaddr_in _oslv2SendAddr; // Address for sending commands to OSLv2
	struct sockaddr_in _pluginRecvAddr; // Address this plugin will bind to for receiving
	
	std::shared_ptr<std::thread> _udpCommThread; // Thread for UDP communication
	std::map<std::string, double> _dataAngleUdp;  // Data received from UDP
	std::map<std::string, double> _dataTorqueUdp; // Data received from UDP
	std::mutex _mtxUdpData; // Mutex for UDP received data
	std::mutex _mtxTime;    // Mutex for time variables
	std::atomic<bool> _threadStop; // <--- CHANGE: Use std::atomic<bool> for thread synchronization
	int _cpt; // Counter, not strictly UDP-related but kept from original
	double _timeStamp;	
	double _timeStampUdp; // Timestamp of last UDP data reception

	// Removed VarName enum as it was specific to ADS

	void setMuscleForcePassive(const std::vector<double>& muscleForcePassive){}
	void setMuscleForceActive(const std::vector<double>& muscleForceActive){}
	void setTendonStrain(const std::vector<double>& tendonStrain){}

	// Removed _varNameVect as it was specific to ADS
	void udpCommThread(); // Renamed from ethercatThread
	void initUdp(int recvPort, const std::string& sendIp, int sendPort); // New UDP initialization
	const double& GetAngleTimeStamp();


	

public:
	/**
	* Constructor
	*/
	OBEmpowerPlugin();

	/**
	* Destructor
	*/
	virtual ~OBEmpowerPlugin();

	/**
	* Initialization method
	* @param executionName execution XML for CEINMS-RT software configuration
	*/
	void init( std::string& executionName );

	void reset()
	{
	}

	/**
	* Get the data with the name of the EMG channel mapping the EMG.
	* The correspondence between the channel and the muscle are in the subject specific XML.
	* Also get the angle for pronation and supination
	*/
	const std::map<std::string, double>& GetDataMap();

	/**
	* Get the time stamp of the EMG capture.
	*/
	const double& getTime();

	void stop();

	void setDirectory(std::string outDirectory, std::string inDirectory = std::string())
	{
	}

	void setVerbose(int verbose)
	{
	}

	void setRecord(bool record)
	{
	}

	const std::map<std::string, double>& GetDataMapTorque();

	void setMuscleName(const std::vector<std::string>& muscleNames)
	{
		this->_muscleNames = muscleNames;
	}

	void setDofName(const std::vector<std::string>& dofName)
	{
		this->_dofNames = dofName;
	}

	void setDofTorque(const std::vector<double>& dofTorque);

	void setDofStiffness(const std::vector<double>& dofStiffness) {
		for(int idx = 0; idx < dofStiffness.size(); idx++ ){
			std::string& tag = this->_dofNames[idx];
			this->_jointStiffness[tag] = dofStiffness[idx];
		}
	}

	void setOutputTimeStamp(const double& timeStamp) {
		this->_outputTimeStamp = timeStamp;
	}

	const std::vector<std::string>& GetDofName() {
		return this->_dofNames;
		}

	// Interface to provide an easy access method to collect data supplied by CEINMS
	double getDofAngle(const std::string& dofName) const;
	double getDofTorque(const std::string& dofName) const;
	double getCEINMSDofTorque(const std::string& dofName) const;
	double getDofStiffness(const std::string& dofName) const;
	double getMuscleForce(const std::string& muscleName) const;
	double getMuscleFiberLength(const std::string& muscleName) const;
	double getMuscleFiberVelocity(const std::string& muscleName) const;
	double getMuscleActivation(const std::string& muscleName) const;

	void setMuscleForce(const std::vector<double>& muscleForce){
		for(int idx = 0; idx < muscleForce.size(); idx++ ){
			std::string& tag = this->_muscleNames[idx];
			this->_muscleForce[tag] = muscleForce[idx];
		}
	}

	void setMuscleFiberLength(const std::vector<double>& muscleFiberLength) {
		for(int idx = 0; idx < muscleFiberLength.size(); idx++ ){
			std::string& tag = this->_muscleNames[idx];
			this->_muscleFiberLength[tag] = muscleFiberLength[idx];
		}
	}

	void setMuscleFiberVelocity(const std::vector<double>& muscleFiberVelocity) {
		for(int idx = 0; idx < muscleFiberVelocity.size(); idx++ ){
			std::string& tag = this->_muscleNames[idx];
			this->_muscleFiberVelocity[tag] = muscleFiberVelocity[idx];
		}
	}

};

#endif