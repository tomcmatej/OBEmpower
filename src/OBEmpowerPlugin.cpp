#include "OBEmpowerPlugin.hpp"
#include <boost/foreach.hpp>
#include <mutex>
#include <thread>
#include <cmath>
#include <iostream>
#include <map>
#include <GetTime.h>
#include <ExecutionXmlReader.h>
#define NOMINMAX
#include <algorithm>
#include <mapTools.hpp>
		


#define ADS_PORT 851


OBEmpowerPlugin::OBEmpowerPlugin()
{
}

OBEmpowerPlugin::~OBEmpowerPlugin()
{	
}

void OBEmpowerPlugin::init(const std::string& subjectName, const std::string& executionName, const PluginConfig& config )
{
	ExecutionXmlReader executionCfg(executionName);
	initTcAds(atoi(executionCfg.getComsumerPort().c_str()));
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
	std::string angleVar = "MAIN.my_struct_emp.Empower_ankle_angle";
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
				ikDataLocal[*it] = ankleAngle;
				// dataSaveID.push_back(dataID[0]);
				idDataLocal[*it] = ankleTorque;

			}
			// else if (*it == "ankle_angle_l")
			// {
			// 	// dataSaveIK.push_back(dataIK[1]);
			// 	ikDataLocal[*it] = dataIK[1];
			// 	// dataSaveID.push_back(dataID[1]);
			// 	idDataLocal[*it] = dataID[1];
			// 	_tcAdsClientObj->read(_varNameVect[VarName::ankleTorque], &TcAdsVars[VarName::ankleTorque], numberOfVariables * sizeof(float));

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

void OBEmpowerPlugin::setDataSourcePointer(InterThreadRO* instance){
	_consumerInstance = instance;
}

void OBEmpowerPlugin::setDataReadyNotification(PLUGIN_DATA_TYPE dataCode){
	std::unique_lock<std::mutex> lock(this->_dataAccess);
	switch(dataCode){
		case PLUGIN_DATA_TYPE::EMG:
			break;
		case PLUGIN_DATA_TYPE::Angle:
			break;
		case PLUGIN_DATA_TYPE::Torque:
			_jointTorqueFromExternalOrID = _consumerInstance->getExternalTorqueMap();
			break;		
		case PLUGIN_DATA_TYPE::XLD:
			break;		
		case PLUGIN_DATA_TYPE::Acceleration:
			break;		
		case PLUGIN_DATA_TYPE::Velocity:
			break;		
		case PLUGIN_DATA_TYPE::Position:
			break;
		default:
			break;
	}
}


const double& OBEmpowerPlugin::getAngleTime(){
	std::unique_lock<std::mutex> lock(this->_mtxEthercat);
	return _timeStampEthercat;
}
const std::vector<std::string>& OBEmpowerPlugin::getAngleDofName(){
	std::unique_lock<std::mutex> lock(this->_mtxEthercat);
	return _dofNames;
}
const std::map<std::string, double>& OBEmpowerPlugin::getDataMapAngle(){
	std::unique_lock<std::mutex> lock(this->_mtxEthercat);
	_jointAngle = _dataAngleEthercat;
	return _jointAngle;
}

const double& OBEmpowerPlugin::getTorqueTime(){
	std::unique_lock<std::mutex> lock(this->_mtxEthercat);
	return _timeStampEthercat;
}
const std::vector<std::string>& OBEmpowerPlugin::getTorqueDofName(){
	std::unique_lock<std::mutex> lock(this->_mtxEthercat);
	return _dofNames;
}
const std::map<std::string, double>& OBEmpowerPlugin::getDataMapTorque(){
	std::unique_lock<std::mutex> lock(this->_mtxEthercat);
	_jointTorqueFromExternalOrID = _dataTorqueEthercat;
	return _jointTorqueFromExternalOrID;
}

void OBEmpowerPlugin::iterationEnd() {
	auto modelTorques = _consumerInstance->getModelTorques();
	auto dofNames = _consumerInstance->getDofNames();
	if(dofNames.size() != modelTorques.size()){ // This should never happen
		modelTorques.clear();
		for(const auto& name : dofNames){
			modelTorques.push_back(0);
		}
	}
	_jointTorqueFromCEINMS = vectorToMap(dofNames, modelTorques);
	float ankleTorqueSp = 0;
	ankleTorqueSp = (std::min)((std::max)((float)this->_jointTorqueFromCEINMS["ankle_angle_r"],EMPOWER_MIN_TORQUE), EMPOWER_MAX_TORQUE);
	_tcAdsClientObj->write(_varNameVect[VarName::torqueControl], &ankleTorqueSp, sizeof(ankleTorqueSp));
}


#ifdef UNIX
extern "C" void* create() {
    return new OBEmpowerPlugin;
}

extern "C" void destroy(void* p) {
    delete (OBEmpowerPlugin*)p;
}

#endif
#ifdef WIN32
extern "C" __declspec (dllexport) void* __cdecl create() {
	return new OBEmpowerPlugin;
}

extern "C" __declspec (dllexport) void __cdecl destroy(void* p) {
	delete (OBEmpowerPlugin*)p;
}
#endif



