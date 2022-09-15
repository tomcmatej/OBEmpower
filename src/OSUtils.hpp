#ifndef OSUTILS_HPP_
#define OSUTILS_HPP_
#define	_USE_MATH_DEFINES

#include <chrono>

class OSUtils{
    OSUtils(){}

public:
    static OSUtils getInstance(){
        static OSUtils instace;
        return instace;
    }

    // double time in seconds (double) since beggining of the program
    static double getTime(void){
        static std::chrono::time_point<std::chrono::high_resolution_clock> start, default_value;
        start == default_value? start = std::chrono::high_resolution_clock::now() : start;
        auto time_elapsed =  std::chrono::high_resolution_clock::now() - start;
        return ((double)std::chrono::duration_cast<std::chrono::microseconds>(time_elapsed).count())/1000000.0;
    }
    
};


#endif