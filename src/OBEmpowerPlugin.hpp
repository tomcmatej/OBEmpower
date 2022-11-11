#pragma once

#define	_USE_MATH_DEFINES
#include "OSUtils.hpp"
#include <PluginInterface.hpp>
#include <string>
#include <set>
#include <mutex>
#include <thread>
#include <tcAdsClient.h>
#include <PluginCommon.hpp>
#include <OpenSimFileLogger.h>
#include <ConstantRateFilter.hpp>
#include <deque>

#define EMPOWER_MAX_TORQUE	30.0f	// Nm
#define EMPOWER_MIN_TORQUE -30.0f	// Nm

// Implemented to consume torques, to provide external torques and angles

class DYNLIBEXPORT OBEmpowerPlugin : public PluginInterface
{
	// Private Variables:
	std::vector<std::string> _dofNames;
	std::map<std::string, double> _jointTorqueFromCEINMS, _jointAngle, _jointTorqueFromExternalOrID;
	std::string _currState;

	// EtherCAT variables
	std::shared_ptr<tcAdsClient> _tcAdsClientObj;
	std::shared_ptr<std::thread> _ethercatThread;
	std::map<std::string, double> _dataAngleEthercat;
	std::map<std::string, double> _dataTorqueEthercat;
	std::mutex _dataAccess;
	std::mutex _mtxEthercat;
	std::mutex _mtxTime;
	bool _threadStop;
	int _cpt;
	double _timeStamp;	
	double _timeStampEthercat;
	InterThreadRO* _consumerInstance;
	std::string _outDirectory;

	bool _record;

	std::shared_ptr<OpenSimFileLogger<double>> _logger;

	double _ankleAngleOld;

	std::deque<double> _angleLog;

	// ConstantRateFilter<double> _angleFilter;
	FilterKin::Filter<double> _angleFilter;


	void start() {}
	void iterationStart() {}
	void iterationEnd();

		enum VarName
	{
		ankleTorque, 
		ankleAngle,
		torqueControl,
		time, 
		LAST
	};

	void setMuscleForcePassive(const std::vector<double>& muscleForcePassive){}
	void setMuscleForceActive(const std::vector<double>& muscleForceActive){}
	void setTendonStrain(const std::vector<double>& tendonStrain){}

	std::vector<unsigned long> _varNameVect;
	void ethercatThread();
	void initTcAds(int portno);
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
	* @param xmlName Subject specific XML
	* @param executionName execution XML for CEINMS-RT software configuration
	*/
	void init( const std::string& subjectName, const std::string& executionName, const PluginConfig& config );

	void reset()
	{
	}
	void kill(){}
	virtual PLUGIN_STATUS_CODES pluginExecutionStatus(){
		return PLUGIN_STATUS_CODES::OPERATION_NORMAL;
	}

	/**
	* Get the data with the name of the EMG channel mapping the EMG.
	* The correspondence between the channel and the muscle are in the subject specific XML.
	* Also get the angle for pronation and supination
	*/
	const std::map<std::string, double>& GetDataMap();

	/**
	* Get a set of the channel name
	*/
	// const std::set<std::string>& GetNameSet()
	// {
	// 	return this->_emgChannels;
	// }

	/**
	* Get the time stamp of the EMG capture.
	*/
	const double& getTime();
	// {
	// 	this->_dataTimeStamp = double(OSUtils::getTime());
	// 	return this->_dataTimeStamp;
	// }

	void stop();

	void setDirectory(const std::string& outDirectory, const std::string& inDirectory = std::string())
	{
		_outDirectory = outDirectory;
	}

	void setVerbose(int verbose)
	{
	}

	void setRecord(bool record)
	{
		_record = record;
	}

	const std::map<std::string, double>& GetDataMapTorque();


	const std::vector<std::string>& GetDofName() { 
		return this->_dofNames; 
		}

	void setDataSourcePointer(InterThreadRO* instance);
	void setDataReadyNotification(PLUGIN_DATA_TYPE dataCode);

	const double& getAngleTime();
	const std::vector<std::string>& getAngleDofName();
	const std::map<std::string, double>& getDataMapAngle();

	const double& getTorqueTime();
	const std::vector<std::string>& getTorqueDofName();
	const std::map<std::string, double>& getDataMapTorque();

};