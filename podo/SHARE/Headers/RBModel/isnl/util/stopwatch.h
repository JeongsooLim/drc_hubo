#ifndef ISNL_STOPWATCH_H
#define ISNL_STOPWATCH_H
#include <ctime>
#include <stdio.h>

namespace isnl{

class stopwatch{
	long cum, base;
public:
	stopwatch();
	void start(); // start to count time
	void reset(); // reset to 0
	long pause(); // stop temporally and return currently cumulated time
	long stemp(); // return currently cumulated time without stopping
	long stop();  // stop and reset

	void printstemp(); // print stemp result
	void printstop();  // print stop result

};
inline stopwatch::stopwatch(){this->reset();}
inline void stopwatch::start(){
	this->base = clock();
}
inline void stopwatch::reset(){
	this->cum  = 0;
	this->base = -1;
}
inline long stopwatch::pause(){
	if(this->base > 0){ // if the watch is running
		this->cum += clock() - this->base;
		this->base = -1;
	}
	return this->cum;
}
inline long stopwatch::stemp(){
	if(this->base > 0){ // if the watch is running
		return clock() - this->base;
	}else{
		return 0;
	}
}
inline long stopwatch::stop(){
	long ret = this->pause();
	this->cum = 0;
	return ret;
}
inline void stopwatch::printstemp(){
	printf("%fs elapsed\n",this->stemp()/1000.0);
}
inline void stopwatch::printstop() {
	printf("%fs elapsed\n",this->stop()/1000.0);
}

}
#endif
