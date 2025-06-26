#include "OBEmpowerPlugin.hpp"
#include <boost/foreach.hpp> // Still used in some loops, could be replaced with C++11 range-based for
#include <mutex>
#include <thread>
#include <cmath>
#include <iostream>
#include <map>
#include <getTime.h> // Custom time utility
#include <ExecutionXmlReader.h> // For reading CEINMS configuration
#include <algorithm>
#include <stdexcept> // For std::runtime_error
#include <unistd.h>  // For close() on Unix
#include <cstring>   // For memset, memcpy

// Removed ADS_PORT


OBEmpowerPlugin::OBEmpowerPlugin() : _udpSocketFd(-1) // Initialize socket FD to invalid state
{
}

OBEmpowerPlugin::~OBEmpowerPlugin()
{
    // Destructor doesn't do much here; `stop()` handles thread join and socket close
}

void OBEmpowerPlugin::init(std::string& executionFilename)
{
	ExecutionXmlReader executionCfg(executionFilename);
    // The CEINMS XML might provide a port for an EMG plugin, but we'll use our own
    // defined UDP ports for OSLv2 communication.
    // If you need CEINMS to configure the ports, you'd add methods to ExecutionXmlReader
    // to parse custom UDP port settings from the XML.
	initUdp(OSLV2_UDP_RECV_PORT, OSLV2_IP_ADDRESS, OSLV2_UDP_SEND_PORT);
}

const std::map<std::string, double>& OBEmpowerPlugin::GetDataMap()
{
	std::unique_lock<std::mutex> lock(this->_mtxUdpData); // Lock the UDP data mutex
	_jointAngle = _dataAngleUdp; // Copy UDP received angle data
	return this->_jointAngle;
}

const std::map<std::string, double>& OBEmpowerPlugin::GetDataMapTorque()
{
	std::unique_lock<std::mutex> lock(this->_mtxUdpData); // Lock the UDP data mutex
	_jointTorqueFromExternalOrID = _dataTorqueUdp; // Copy UDP received torque data
	return this->_jointTorqueFromExternalOrID;
}

void OBEmpowerPlugin::stop()
{
	_threadStop = true; // Signal the UDP communication thread to stop

    // Ensure the thread exists before trying to join it
    if (_udpCommThread && _udpCommThread->joinable()) {
	    _udpCommThread->join(); // Wait for the UDP thread to finish
    }
	
    // Close the UDP socket
    if (_udpSocketFd != -1) {
        close(_udpSocketFd); // Unix-specific socket close
        _udpSocketFd = -1; // Mark as closed
    }
}

const double& OBEmpowerPlugin::getTime(){
	return this->_timeStamp;
}

// Getters remain largely the same, just the source of data changes upstream
double OBEmpowerPlugin::getDofAngle(const std::string& dofName) const {
	if(this->_jointAngle.find(dofName) == this->_jointAngle.end()){
		std::cout << "Warning: Cannot find angle data for source " << dofName << " at timestep " << this->_timeStamp << std::endl;
		return 0;
	}
	return this->_jointAngle.at(dofName);
}
double OBEmpowerPlugin::getDofTorque(const std::string& dofName) const {
	if(this->_jointTorqueFromExternalOrID.find(dofName) == this->_jointTorqueFromExternalOrID.end()){ // Note: Original had "_moment" suffix, removed for simplicity with OSLv2
		std::cout << "Warning: Cannot find torque data for source " << dofName << " at timestep " << this->_timeStamp << std::endl;
		return 0;
	}
	// std::string tagName = dofName + "_moment"; // Removed suffix assumption
	return this->_jointTorqueFromExternalOrID.at(dofName); // Use raw dofName
}
double OBEmpowerPlugin::getCEINMSDofTorque(const std::string& dofName) const {
	if(this->_jointTorqueFromCEINMS.find(dofName) == this->_jointTorqueFromCEINMS.end()){
		std::cout << "Warning: Cannot find torque data for source " << dofName << " at timestep " << this->_timeStamp << std::endl;
		return 0;
	}
	return this->_jointTorqueFromCEINMS.at(dofName);
}
double OBEmpowerPlugin::getDofStiffness(const std::string& dofName) const {
	if(this->_jointStiffness.find(dofName) == this->_jointStiffness.end()){
		std::cout << "Warning: Cannot find stiffness data for source " << dofName << " at timestep " << this->_timeStamp << std::endl;
		return 0;
	}
	return this->_jointStiffness.at(dofName);
}
double OBEmpowerPlugin::getMuscleForce(const std::string& muscleName) const {
	if(this->_muscleForce.find(muscleName) == this->_muscleForce.end()){
		std::cout << "Warning: Cannot find force data for source " << muscleName << " at timestep " << this->_timeStamp << std::endl;
		return 0;
	}
	return this->_muscleForce.at(muscleName);
}
double OBEmpowerPlugin::getMuscleFiberLength(const std::string& muscleName) const {
	if(this->_muscleFiberLength.find(muscleName) == this->_muscleFiberLength.end()){
		std::cout << "Warning: Cannot find fiber length data for source " << muscleName << " at timestep " << this->_timeStamp << std::endl;
		return 0;
	}
    // Assuming 'optimalFiberLength' is a scaling factor for normalized fiber length
	return this->_muscleFiberLength.at(muscleName) * this->_subjectMuscleParameters.at(muscleName).at("optimalFiberLength");
}
double OBEmpowerPlugin::getMuscleFiberVelocity(const std::string& muscleName) const {
	if(this->_muscleFiberVelocity.find(muscleName) == this->_muscleFiberVelocity.end()){
		std::cout << "Warning: Cannot find fiber velocity data for source " << muscleName << " at timestep " << this->_timeStamp << std::endl;
		return 0;
	}
	return this->_muscleFiberVelocity.at(muscleName);
}
double OBEmpowerPlugin::getMuscleActivation(const std::string& muscleName) const {
	if(this->_muscleActivation.find(muscleName) == this->_muscleActivation.end()){
		std::cout << "Warning: Cannot find activation data for source " << muscleName << " at timestep " << this->_timeStamp << std::endl;
		return 0;
	}
	return this->_muscleActivation.at(muscleName);
}


void OBEmpowerPlugin::initUdp(int recvPort, const std::string& sendIp, int sendPort)
{
    // 1. Create UDP Socket
    _udpSocketFd = socket(AF_INET, SOCK_DGRAM, 0); // AF_INET for IPv4, SOCK_DGRAM for UDP
    if (_udpSocketFd < 0) {
        throw std::runtime_error("Failed to create UDP socket.");
    }
    std::cout << "UDP Socket created with FD: " << _udpSocketFd << std::endl;

    // Optional: Allow reuse of address/port (useful for quick restarts)
    int reuse = 1;
    if (setsockopt(_udpSocketFd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) < 0) {
        std::cerr << "Warning: setsockopt(SO_REUSEADDR) failed." << std::endl;
    }
    // SO_REUSEPORT is also common on Linux
    if (setsockopt(_udpSocketFd, SOL_SOCKET, SO_REUSEPORT, &reuse, sizeof(reuse)) < 0) {
        std::cerr << "Warning: setsockopt(SO_REUSEPORT) failed." << std::endl;
    }


    // 2. Configure plugin's receive address (where OSLv2 will send data)
    memset(&_pluginRecvAddr, 0, sizeof(_pluginRecvAddr)); // Clear the structure
    _pluginRecvAddr.sin_family = AF_INET; // IPv4
    _pluginRecvAddr.sin_addr.s_addr = htonl(INADDR_ANY); // Listen on all available interfaces (0.0.0.0)
    _pluginRecvAddr.sin_port = htons(recvPort); // Port for receiving (e.g., 12345)

    // 3. Bind the socket to the receive address
    if (bind(_udpSocketFd, (const struct sockaddr *)&_pluginRecvAddr, sizeof(_pluginRecvAddr)) < 0) {
        close(_udpSocketFd);
        throw std::runtime_error("Failed to bind UDP socket to receive port " + std::to_string(recvPort));
    }
    std::cout << "UDP Socket bound to port: " << recvPort << std::endl;

    // 4. Configure OSLv2's send address (where we send commands to OSLv2)
    memset(&_oslv2SendAddr, 0, sizeof(_oslv2SendAddr)); // Clear the structure
    _oslv2SendAddr.sin_family = AF_INET; // IPv4
    _oslv2SendAddr.sin_port = htons(sendPort); // Port for sending (e.g., 12346)
    if (inet_pton(AF_INET, sendIp.c_str(), &_oslv2SendAddr.sin_addr) <= 0) { // Convert IP string to binary
        close(_udpSocketFd);
        throw std::runtime_error("Invalid IP address: " + sendIp);
    }
    std::cout << "OSLv2 Send Address configured to " << sendIp << ":" << sendPort << std::endl;


	this->_threadStop = false;
	_timeStamp = rtb::getTime();

    // Start the UDP communication thread
	_udpCommThread = std::make_shared<std::thread>(&OBEmpowerPlugin::udpCommThread, this);
}


void OBEmpowerPlugin::udpCommThread()
{
	double timeLocal;
	// Use local variables to store data before copying to shared members to minimize lock time
	std::map<std::string, double> ikDataLocal;
	std::map<std::string, double> idDataLocal;

    // Define a buffer for receiving UDP packets
    Oslv2DataPacket recvPacket; // Use the defined struct for direct reception
    socklen_t addrLen = sizeof(struct sockaddr_in); // Size of address structure for recvfrom

	while (!_threadStop)
	{
		timeLocal = rtb::getTime();

        // Try to receive a UDP packet. Blocking call, will wait here until a packet arrives
        // or the thread is signaled to stop by some mechanism (e.g., non-blocking socket with timeout).
        // For simplicity, we'll keep it blocking here. A timeout could be added for responsiveness.
        ssize_t bytes_received = recvfrom(_udpSocketFd, &recvPacket, sizeof(recvPacket), 0,
                                          NULL, NULL); // No need for sender address for now

        if (bytes_received > 0) {
            if (bytes_received == sizeof(Oslv2DataPacket)) {
                // Packet received and is of the expected size.
                // Assuming OSLv2 sends only ankle data for now.
                // You might need to check dofNames to map it correctly.
                ikDataLocal["ankle_angle_r"] = recvPacket.ankleAngle_rad;
                idDataLocal["ankle_angle_r"] = recvPacket.ankleTorque_Nm;

                // Update shared data under lock
                {
                    std::unique_lock<std::mutex> lock(this->_mtxUdpData);
                    _dataAngleUdp = ikDataLocal;
                    _dataTorqueUdp = idDataLocal;
                }
                {
                    std::unique_lock<std::mutex> lock(this->_mtxTime);
                    _timeStampUdp = timeLocal;
                }

            } else {
                std::cerr << "Warning: Received UDP packet of unexpected size: " << bytes_received
                          << " bytes. Expected: " << sizeof(Oslv2DataPacket) << std::endl;
            }
        } else if (bytes_received < 0) {
            // An error occurred during recvfrom.
            // Note: If the socket is closed while recvfrom is blocking, it might return -1.
            if (!_threadStop) { // Only report error if not in shutdown sequence
                 std::cerr << "UDP recvfrom error: " << strerror(errno) << std::endl;
            }
        }
        // If bytes_received is 0, it means the peer has performed an orderly shutdown (not common for UDP)
        // or nothing was received (if non-blocking).
	}
}

const double& OBEmpowerPlugin::GetAngleTimeStamp()
{
	std::unique_lock<std::mutex> lock(this->_mtxTime);
	_timeStamp = _timeStampUdp;
	return _timeStamp;
}

void OBEmpowerPlugin::setDofTorque(const std::vector<double>& dofTorque){
	for(int idx = 0; idx < dofTorque.size(); idx++ ){
		std::string& tag = this->_dofNames[idx];
		this->_jointTorqueFromCEINMS[tag] = dofTorque[idx];
	}

	#undef max
	#undef min

    // Prepare the command packet
    Oslv2CommandPacket sendPacket;
    // Clamp the ankle torque setpoint to be within defined limits
	sendPacket.torqueSetpoint_Nm = std::min(std::max((float)this->_jointTorqueFromCEINMS["ankle_angle_r"], EMPOWER_MIN_TORQUE), EMPOWER_MAX_TORQUE);

    // Send the UDP packet to OSLv2
    // sendto returns the number of bytes sent, or -1 on error
    ssize_t bytes_sent = sendto(_udpSocketFd, &sendPacket, sizeof(sendPacket), 0,
                                (const struct sockaddr *)&_oslv2SendAddr, sizeof(_oslv2SendAddr));

    if (bytes_sent < 0) {
        std::cerr << "UDP sendto error: " << strerror(errno) << std::endl;
    } else if (bytes_sent != sizeof(sendPacket)) {
        std::cerr << "Warning: Sent " << bytes_sent << " bytes, but expected "
                  << sizeof(sendPacket) << " bytes for torque command." << std::endl;
    }
}


// Factory functions (remain unchanged)
#ifdef UNIX
extern "C" AngleAndComsumerPlugin * create() {
	return new OBEmpowerPlugin;
}

extern "C" void destroy(AngleAndComsumerPlugin * p) {
	delete p;
}
#endif

#if defined(WIN32) && !defined(STATIC_UNIT_TESTS)
extern "C" __declspec (dllexport) AngleAndComsumerPlugin * __cdecl create() {
	return new OBEmpowerPlugin;
}

extern "C" __declspec (dllexport) void __cdecl destroy(AngleAndComsumerPlugin * p) {
	delete p;
}
#endif