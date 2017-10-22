#include <as64_utils/misc/timer.h>

void TIMER::start() 
{ 
	t1 = std::chrono::high_resolution_clock::now(); 
}

void TIMER::stop() 
{ 
	t2 = std::chrono::high_resolution_clock::now(); 
}

double TIMER::get_elapsed_time() const 
{ 
	return (std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1)).count(); 
}

