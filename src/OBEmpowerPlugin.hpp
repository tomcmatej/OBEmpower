#ifndef OBEMPOWERPLUGIN_HPP_
#define OBEMPOWERPLUGIN_HPP_

#define	_USE_MATH_DEFINES
#include "OSUtils.hpp"
#include <AngleAndComsumerPlugin.h>
#include <string>
#include <set>
#include <mutex>
#include <thread>
#include <tcAdsClient.h>

#define EMPOWER_MAX_TORQUE	30.0f	// Nm
#define EMPOWER_MIN_TORQUE -30.0f	// Nm


#if defined(_WIN32) && !defined(STATIC_UNIT_TESTS)
class __declspec(dllexport) OBEmpowerPlugin : public AngleAndComsumerPlugin
#else
class OBEmpowerPlugin : public AngleAndComsumerPlugin
#endif
{
	// Private Variables:
	std::vector<std::string> _muscleNames;
	std::vector<std::string> _dofNames;
	std::map<std::string, double> _jointAngle, _jointTorqueFromCEINMS, _jointTorqueFromExternalOrID, _jointStiffness, _muscleActivation, _muscleForce, _muscleFiberLength, _muscleFiberVelocity, _position, _velocity, _acceleration, _grf;
	double _outputTimeStamp, _dataTimeStamp;
	std::string _currState;
	std::map<std::string, std::map<std::string, double>> _subjectMuscleParameters;

	// EtherCAT variables
	std::shared_ptr<tcAdsClient> _tcAdsClientObj;
	std::shared_ptr<std::thread> _ethercatThread;
	std::map<std::string, double> _dataAngleEthercat;
	std::map<std::string, double> _dataTorqueEthercat;
	std::mutex _mtxEthercat;
	std::mutex _mtxTime;
	bool _threadStop;
	int _cpt;
	double _timeStamp;	
	double _timeStampEthercat;



		enum VarName
	{
		ankleTorque, 
		ankleAngle,
		torqueControl,
		time, 
		LAST
	};

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

	void setMuscleForcePassive(const std::vector<double>& muscleForcePassive){}
	void setMuscleForceActive(const std::vector<double>& muscleForceActive){}
	void setTendonStrain(const std::vector<double>& tendonStrain){}

	/**
	* Initialization method
	* @param xmlName Subject specific XML
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
	double getDofAngle(const std::string& dofName) const; //{ return this->_jointAngle.at(dofName);}
	double getDofTorque(const std::string& dofName) const;// { return this->_jointTorqueFromExternalOrID.at(dofName);} // From ID or sensor
	double getCEINMSDofTorque(const std::string& dofName) const;// { return this->_jointTorqueFromExternalOrID.at(dofName);} // From ID or sensor
	double getDofStiffness(const std::string& dofName) const;// { return this->_jointStiffness.at(dofName);}
	double getMuscleForce(const std::string& muscleName) const;// { return this->_muscleForce.at(muscleName);}
	double getMuscleFiberLength(const std::string& muscleName) const;// { return this->_muscleFiberLength.at(muscleName);}
	double getMuscleFiberVelocity(const std::string& muscleName) const;// { return this->_muscleFiberVelocity.at(muscleName);}
	double getMuscleActivation(const std::string& muscleName) const;// { return this->_muscleActivation.at(muscleName);}


	void setMuscleForce(const std::vector<double>& muscleForce)  {
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

	void setMuscleFiberVelocity(const std::vector<double>& muscleFiberVelocity)  {
		for(int idx = 0; idx < muscleFiberVelocity.size(); idx++ ){
			std::string& tag = this->_muscleNames[idx];
			this->_muscleFiberVelocity[tag] = muscleFiberVelocity[idx];
		}
	}

	// Optimization Cost Functions

};


#endif