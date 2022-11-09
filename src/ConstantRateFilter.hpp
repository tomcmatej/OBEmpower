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

#pragma once

#include <vector>
#include <deque>
#include <cmath>
#include <iostream>
#include <cstdlib>
#include <Filter.h>

template<typename T> class ConstantRateFilter
{
	public:
		ConstantRateFilter ( const std::vector<T>& aCoeff, const std::vector<T>& bCoeff, double samplingFreq);
		ConstantRateFilter ( const std::vector<T>& aCoeff, const std::vector<T>& bCoeff, double samplingFreq, std::vector<T> pastData );
		ConstantRateFilter(){};

		~ConstantRateFilter();

		ConstantRateFilter ( const ConstantRateFilter& other )
		{
			*this = other; // Uses the equality operator below
		}


		ConstantRateFilter& operator= ( const ConstantRateFilter& other )
		{
			_filt = other._filt;
			_samplingFreq = other._samplingFreq;
		}

		bool operator== ( const ConstantRateFilter& other )
		{
			return _filt == other._filt && _samplingFreq = other._samplingFreq;
		}

		void init(const std::vector<T>& aCoeff, const std::vector<T>& bCoeff, double samplingFreq, std::vector<T> pastData);

		T filter(const T& data, double timeStamp);

		void reset();

	protected:
		FilterKin::Filter<T> _filt;
		double _samplingFreq, _lastSampleTimeStamp;
		T _lastDataOut;

};

#include "ConstantRateFilter.cpp"