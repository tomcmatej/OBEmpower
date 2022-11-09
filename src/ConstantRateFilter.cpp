/*
 * Copyright 2015 <copyright holder> <email>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

// #include <ConstantRateFilter.hpp>

template<typename T>
ConstantRateFilter<T>::ConstantRateFilter(const std::vector<T>& aCoeff, const std::vector<T>& bCoeff, double samplingFreq): _filt(aCoeff, bCoeff)
{
	_samplingFreq = samplingFreq;
	_lastSampleTimeStamp = -1;
}

template<typename T>
ConstantRateFilter<T>::ConstantRateFilter( const std::vector<T>& aCoeff, const std::vector<T>& bCoeff, double samplingFreq, std::vector<T> pastData ) : _filt(aCoeff, bCoeff, pastData)
{
	_samplingFreq = samplingFreq;
	_lastSampleTimeStamp = -1;
}

template<typename T>
ConstantRateFilter<T>::~ConstantRateFilter()
{

}

template<typename T>
void ConstantRateFilter<T>::init(const std::vector<T>& aCoeff, const std::vector<T>& bCoeff, double samplingFreq, std::vector<T> pastData)
{
	_filt.init(aCoeff, bCoeff, pastData);
	_samplingFreq = samplingFreq;
}

template<typename T>
T ConstantRateFilter<T>::filter(const T& data, double timeStamp)
{
	if(_lastSampleTimeStamp == -1 || ((timeStamp - _lastSampleTimeStamp) >= 1/_samplingFreq)){
		T dataOut = _filt.filter(data);
		_lastDataOut = dataOut;
		_lastSampleTimeStamp = timeStamp - remainder(timeStamp, 1/_samplingFreq);
		return dataOut;
	}
	return _lastDataOut;
}

template<typename T>
void ConstantRateFilter<T>::reset(){
	_filt.reset();
	_lastSampleTimeStamp = -1;
}