#ifndef TIMER_64_H
#define TIMER_64_H

#include <chrono>

class TIMER
{
public:
	void start();
	void stop();
	double get_elapsed_time() const;
private:
	std::chrono::high_resolution_clock::time_point t1, t2;
};

#endif
