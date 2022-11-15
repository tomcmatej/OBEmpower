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
		
// #define AFILT_COEFF 1.0000,-1.4755,0.5869
// #define BFILT_COEFF 0.0279,0.0557,0.0279		// 6 Hz cutoff frequency, 100Hz update rate

// #define AFILT_COEFF 1.0000,-1.7640,0.7890
// #define BFILT_COEFF 0.0063,0.0125,0.0063		// 12 Hz cutoff frequency, 450Hz update rate

#define AFILT_COEFF 1.0000,-1.9408,0.9425
#define BFILT_COEFF 0.0004260,0.0008519,0.0004260	// 3 Hz cutoff frequency, 450Hz update rate

#define SAMPLING_FREQ	100

#define ADS_PORT 851
#define NO_SAMPLES_FILTER 10
#define DOF_NAME "ankle_angle_r"


OBEmpowerPlugin::OBEmpowerPlugin()
{
}

OBEmpowerPlugin::~OBEmpowerPlugin()
{	
}

void OBEmpowerPlugin::init(const std::string& subjectName, const std::string& executionName, const PluginConfig& config )
{
	ExecutionXmlReader executionCfg(executionName);

	std::vector<double> aFilter{AFILT_COEFF}, bFilter{BFILT_COEFF};
	aFilter.erase(aFilter.begin());
	// aFilter.pop_front();
	// _angleFilter.init(aFilter, bFilter, SAMPLING_FREQ, std::vector<double>(3));
	_angleFilter.init(aFilter, bFilter, std::vector<double>(3));
    _ankleAngleOld=0; 
	initTcAds(atoi(executionCfg.getComsumerPort().c_str()));
	_dofNames.push_back(DOF_NAME);
	_angleLog = std::deque<double>(NO_SAMPLES_FILTER);
}


void OBEmpowerPlugin::stop()
{
	_threadStop = true;
	_ethercatThread->join();
	for (std::vector<unsigned long>::const_iterator it = _varNameVect.begin(); it != _varNameVect.end(); it++)
		_tcAdsClientObj->releaseVariableHandle(*it);
	_tcAdsClientObj->disconnect();
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
	// std::string angleVar = "MAIN.my_struct_emp.Empower_ankle_angle";
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
		float ankleTorque = 0;
		double ankleAngle = 0;
		long unsigned int time = 0;

		std::vector<double> dataIK, dataSaveIK;
		std::vector<double> dataID, dataSaveID;

		timeLocal = rtb::getTime();

		_tcAdsClientObj->read(_varNameVect[VarName::ankleAngle], &ankleAngle, sizeof(ankleAngle));
 		_tcAdsClientObj->read(_varNameVect[VarName::ankleTorque], &ankleTorque, sizeof(ankleTorque));
		_tcAdsClientObj->read(_varNameVect[VarName::time], &time, sizeof(time));

		if(ankleAngle == 0 && _ankleAngleOld !=0)
			ankleAngle = _ankleAngleOld;

		_ankleAngleOld = ankleAngle;

		for (std::vector<std::string>::const_iterator it = this->_dofNames.begin(); it != this->_dofNames.end(); it++)
		{

			if (*it == DOF_NAME)
			{
				dataSaveIK.push_back(ankleAngle);
				ikDataLocal[*it] = ankleAngle;
				dataSaveID.push_back(-ankleTorque);
				idDataLocal[*it + "_moment"] = -ankleTorque;

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
				idDataLocal[*it + "_moment"] = 0;
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
	// _dofNames = instance->getDofNames();
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
	double twinCATAngle = 0;
	{
	std::unique_lock<std::mutex> lock(this->_mtxEthercat);
	twinCATAngle = _dataAngleEthercat.at(DOF_NAME);
	// _jointAngle = _dataAngleEthercat;
	// _angleLog.push_back(_dataAngleEthercat.at(DOF_NAME));
	}
	double ankleAngle = _angleFilter.filter((double)twinCATAngle);

	// _angleLog.pop_front();
	
	// double ankleAngle = 0;
	// for(auto angle : _angleLog)
	// 	ankleAngle += angle;

	// ankleAngle = ankleAngle / NO_SAMPLES_FILTER;

	_jointAngle[DOF_NAME] = ankleAngle;
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



