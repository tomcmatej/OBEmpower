#include "OBEmpowerPlugin.hpp"
#include <boost/foreach.hpp>
#include <mutex>
#include <thread>
#include <cmath>
#include <iostream>
#include <map>
#include <GetTime.h>
#include <ExecutionXmlReader.h>
#include <algorithm>
		


#define ADS_PORT 851


OBEmpowerPlugin::OBEmpowerPlugin()
{
}

OBEmpowerPlugin::~OBEmpowerPlugin()
{	
}

void OBEmpowerPlugin::init(std::string& executionFilename)
{
	ExecutionXmlReader executionCfg(executionFilename);
	initTcAds(atoi(executionCfg.getComsumerPort().c_str()));
}

const std::map<std::string, double>& OBEmpowerPlugin::GetDataMap()
{
	std::unique_lock<std::mutex> lock(this->_mtxEthercat);
	_jointAngle = _dataAngleEthercat;

	return this->_jointAngle;
}

const std::map<std::string, double>& OBEmpowerPlugin::GetDataMapTorque()
{
	std::unique_lock<std::mutex> lock(this->_mtxEthercat);
	_jointTorqueFromExternalOrID = _dataTorqueEthercat;
	return this->_jointTorqueFromExternalOrID;
}

void OBEmpowerPlugin::stop()
{
	// TODO: Kill optmization thread


	_threadStop = true;
	_ethercatThread->join();
	// delete _ethercatThread;
	// if (record_)
	// {
	// 	logger_->stop();
	// 	delete logger_;
	// }
	for (std::vector<unsigned long>::const_iterator it = _varNameVect.begin(); it != _varNameVect.end(); it++)
	_tcAdsClientObj->releaseVariableHandle(*it);
	_tcAdsClientObj->disconnect();
	// delete _AdsClientObj;
}

const double& OBEmpowerPlugin::getTime(){
	return this->_timeStamp;
}


double OBEmpowerPlugin::getDofAngle(const std::string& dofName) const { 
	if(this->_jointAngle.find(dofName) == this->_jointAngle.end()){ // Requested tag not found
		std::cout << "Warning: Cannot find angle data for source " << dofName << " at timestep " << this->_timeStamp << std::endl;
		return 0;
	}
	return this->_jointAngle.at(dofName);
}
double OBEmpowerPlugin::getDofTorque(const std::string& dofName) const {
	if(this->_jointTorqueFromExternalOrID.find(dofName + std::string("_moment")) == this->_jointTorqueFromExternalOrID.end()){ // Requested tag not found
		std::cout << "Warning: Cannot find torque data for source " << dofName << " at timestep " << this->_timeStamp  << std::endl;
		return 0;
	}
	std::string tagName = dofName + "_moment";
	return this->_jointTorqueFromExternalOrID.at(tagName);
} // From ID or sensor
double OBEmpowerPlugin::getCEINMSDofTorque(const std::string& dofName) const { 
	if(this->_jointTorqueFromCEINMS.find(dofName) == this->_jointTorqueFromCEINMS.end()){ // Requested tag not found
		std::cout << "Warning: Cannot find torque data for source " << dofName << " at timestep " << this->_timeStamp  << std::endl;
		return 0;
	}
	return this->_jointTorqueFromCEINMS.at(dofName);
} // From CEINMS core output
double OBEmpowerPlugin::getDofStiffness(const std::string& dofName) const { 
	if(this->_jointStiffness.find(dofName) == this->_jointStiffness.end()){ // Requested tag not found
		std::cout << "Warning: Cannot find stiffness data for source " << dofName << " at timestep " << this->_timeStamp  << std::endl;
		return 0;
	}
	return this->_jointStiffness.at(dofName);
}
double OBEmpowerPlugin::getMuscleForce(const std::string& muscleName) const { 
	if(this->_muscleForce.find(muscleName) == this->_muscleForce.end()){ // Requested tag not found
		std::cout << "Warning: Cannot find force data for source " << muscleName << " at timestep " << this->_timeStamp  << std::endl;
		return 0;
	}
	return this->_muscleForce.at(muscleName);
}
double OBEmpowerPlugin::getMuscleFiberLength(const std::string& muscleName) const { 
	if(this->_muscleFiberLength.find(muscleName) == this->_muscleFiberLength.end()){ // Requested tag not found
		std::cout << "Warning: Cannot find fiber length data for source " << muscleName << " at timestep " << this->_timeStamp  << std::endl;
		return 0;
	}
	return this->_muscleFiberLength.at(muscleName) * this->_subjectMuscleParameters.at(muscleName).at("optimalFiberLength");
}
double OBEmpowerPlugin::getMuscleFiberVelocity(const std::string& muscleName) const { 
	if(this->_muscleFiberVelocity.find(muscleName) == this->_muscleFiberVelocity.end()){ // Requested tag not found
		std::cout << "Warning: Cannot find fiber velocity data for source " << muscleName  << " at timestep " << this->_timeStamp << std::endl;
		return 0;
	}
	return this->_muscleFiberVelocity.at(muscleName);
}
double OBEmpowerPlugin::getMuscleActivation(const std::string& muscleName) const { 
	if(this->_muscleActivation.find(muscleName) == this->_muscleActivation.end()){ // Requested tag not found
		std::cout << "Warning: Cannot find activation data for source " << muscleName  << " at timestep " << this->_timeStamp << std::endl;
		return 0;
	}
	return this->_muscleActivation.at(muscleName);
}


void OBEmpowerPlugin::initTcAds(int portno)
{

	_tcAdsClientObj = std::make_shared<tcAdsClient>(ADS_PORT);	// XML only allows for one port, and the EMG plugin port is different than the one to be used for the Empower.
															// This will be hardcoded for now.
	this->_threadStop = false;
	// std::string torqueVar = "MAIN.my_struct_emp.Inputs.Empower_series_spring_torque";
	// std::string angleVar = "MAIN.my_struct_emp.Inputs.Empower_ankle_angle";
	// std::string torqueControlVar = "MAIN.my_struct_emp.Outputs.Empower_torque_sp";
	// std::string timeVar = "MAIN.my_struct_emp.Inputs.Empower_ankle_angle_ts";

	// std::string torqueVar = "MAIN.Inputs.my_struct_emp.Empower_series_spring_torque";
	// std::string angleVar = "MAIN.Inputs.my_struct_emp.Empower_ankle_angle";
	// std::string torqueControlVar = "MAIN.Outputs.my_struct_emp.Empower_torque_sp";
	// std::string timeVar = "MAIN.Inputs.my_struct_emp.Empower_ankle_angle_ts";

	std::string torqueVar = "MAIN.my_struct_emp.Empower_series_spring_torque";
	std::string angleVar = "MAIN.filtAngle";
	std::string torqueControlVar = "MAIN.out_struct.Empower_torque_sp";
	std::string timeVar = "MAIN.my_struct_emp.Empower_ankle_angle_ts";

	// std::string torqueVar = "Inputs.MAIN.my_struct_emp.Empower_series_spring_torque";
	// std::string angleVar = "Inputs.MAIN.my_struct_emp.Empower_ankle_angle";
	// std::string torqueControlVar = "Outputs.MAIN.my_struct_emp.Empower_torque_sp";
	// std::string timeVar = "Inputs.MAIN.my_struct_emp.Empower_ankle_angle_ts";

	// std::string torqueVar = "NucleoPLC.MAIN.my_struct_emp.Inputs.Empower_series_spring_torque";
	// std::string angleVar = "NucleoPLC.MAIN.my_struct_emp.Inputs.Empower_ankle_angle";
	// std::string torqueControlVar = "NucleoPLC.MAIN.my_struct_emp.Outputs.Empower_torque_sp";
	// std::string timeVar = "NucleoPLC.MAIN.my_struct_emp.Inputs.Empower_ankle_angle_ts";


	_varNameVect.resize(VarName::LAST);

	_varNameVect[VarName::ankleTorque] = _tcAdsClientObj->getVariableHandle(&torqueVar[0], (int)torqueVar.size());
	_varNameVect[VarName::ankleAngle] = _tcAdsClientObj->getVariableHandle(&angleVar[0], (int)angleVar.size());
	_varNameVect[VarName::torqueControl] = _tcAdsClientObj->getVariableHandle(&torqueControlVar[0], (int)torqueControlVar.size());
	_varNameVect[VarName::time] = _tcAdsClientObj->getVariableHandle(&timeVar[0], (int)timeVar.size());
	

	if (_varNameVect[VarName::ankleTorque] == -1 || _varNameVect[VarName::ankleAngle] == -1 || _varNameVect[VarName::torqueControl] == -1 || _varNameVect[VarName::torqueControl] == -1)
		throw std::runtime_error( "One or more TcAds variables not found" );

	_timeStamp = rtb::getTime();

	_ethercatThread = std::make_shared<std::thread>(&OBEmpowerPlugin::ethercatThread, this);
}


void OBEmpowerPlugin::ethercatThread()
{
	double timeLocal;
	std::map<std::string, double> ikDataLocal;
	std::map<std::string, double> idDataLocal;

	while (!_threadStop)
	{

		int length = 1; //Lankle, Rankle
		unsigned int numberOfVariables = VarName::LAST;
		float ankleAngle = 0, ankleTorque = 0;
		long unsigned int time = 0;

		std::vector<double> dataIK, dataSaveIK;
		std::vector<double> dataID, dataSaveID;

		timeLocal = rtb::getTime();



		_tcAdsClientObj->read(_varNameVect[VarName::ankleAngle], &ankleAngle, sizeof(ankleAngle));
		_tcAdsClientObj->read(_varNameVect[VarName::ankleTorque], &ankleTorque, sizeof(ankleTorque));
		_tcAdsClientObj->read(_varNameVect[VarName::ankleTorque], &time, sizeof(time));

		for (std::vector<std::string>::const_iterator it = this->_dofNames.begin(); it != this->_dofNames.end(); it++)
		{

			if (*it == "ankle_angle_r")
			{
				// dataSaveIK.push_back(dataIK[0]);
				ikDataLocal[*it] = 0;
				// dataSaveID.push_back(dataID[0]);
				idDataLocal[*it] = ankleTorque;
				// _tcAdsClientObj

			}
			// else if (*it == "ankle_angle_l")
			// {
			// 	// dataSaveIK.push_back(dataIK[1]);
			// 	ikDataLocal[*it] = dataIK[1];
			// 	// dataSaveID.push_back(dataID[1]);
			// 	idDataLocal[*it] = dataID[1];
				// _tcAdsClientObj->read(_varNameVect[VarName::ankleTorque], &TcAdsVars[VarName::ankleTorque], numberOfVariables * sizeof(float));

			// }
			else
			{
				ikDataLocal[*it] = 0;
				dataSaveIK.push_back(0);
				idDataLocal[*it] = 0;
				dataSaveID.push_back(0);
			}
		}
		{
		std::unique_lock<std::mutex> lock(this->_mtxEthercat);
		_dataAngleEthercat = ikDataLocal;
		_dataTorqueEthercat = idDataLocal;
		}
		{
		std::unique_lock<std::mutex> lock(this->_mtxTime);
		_timeStampEthercat = timeLocal;
		}

		// if (record_)
		// {

		// 	logger_->log(Logger::IK, timeLocal, dataSaveIK);
		// 	logger_->log(Logger::ID, timeLocal, dataSaveID);

		// 	logger_->log(Logger::RandomSignal, timeLocal, randSignal);
		// }

	}
}

const double& OBEmpowerPlugin::GetAngleTimeStamp()
{
	std::unique_lock<std::mutex> lock(this->_mtxTime);
	_timeStamp = _timeStampEthercat;
	return _timeStamp;
}

void OBEmpowerPlugin::setDofTorque(const std::vector<double>& dofTorque){
	for(int idx = 0; idx < dofTorque.size(); idx++ ){
		std::string& tag = this->_dofNames[idx];
		this->_jointTorqueFromCEINMS[tag] = dofTorque[idx];
	}

	// static float ankleTorqueSp = 0;
	float ankleTorqueSp = 0;
	for (std::vector<std::string>::const_iterator it = _dofNames.begin(); it != _dofNames.end(); it++)
	{
		int64_t cpt = std::distance<std::vector<std::string>::const_iterator>(_dofNames.begin(), it);
		if (*it == "ankle_angle_l")
		{
			// ankleTorqueSp = (float)dofTorque[cpt];
		}
		if (*it == "ankle_angle_r")
		{			
			// ankleTorqueSp = (float)dofTorque[cpt];
		}
	}

	#undef max
	#undef min
	ankleTorqueSp = std::min(std::max((float)this->_jointTorqueFromCEINMS["ankle_angle_r"],EMPOWER_MIN_TORQUE), EMPOWER_MAX_TORQUE);

	// ankleTorqueSp = (float)std::min(1, 9999);

	
	_tcAdsClientObj->write(_varNameVect[VarName::torqueControl], &ankleTorqueSp, sizeof(ankleTorqueSp));
	// _tcAdsClientObj->write(_varNameVect[VarName::torqueControl], &ankleTorqueSp, sizeof(float));
}


#ifdef UNIX
extern "C" AngleAndComsumerPlugin * create() {
	return new OBEmpowerPlugin;
}

extern "C" void destroy(AngleAndComsumerPlugin * p) {
	delete p;
}
#endif

#if defined(WIN32) && !defined(STATIC_UNIT_TESTS) // __declspec (dllexport) id important for dynamic loading
extern "C" __declspec (dllexport) AngleAndComsumerPlugin * __cdecl create() {
	return new OBEmpowerPlugin;
}

extern "C" __declspec (dllexport) void __cdecl destroy(AngleAndComsumerPlugin * p) {
	delete p;
}
#endif




