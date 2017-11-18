#ifndef TIME_LIB_TIME_64_H
#define TIME_LIB_TIME_64_H

#include <ctime>
#include <chrono>
#include <string>
#include <sstream>

#include <time_lib/timer.h>

namespace as64
{

/** \brief Returns the current time stamp in the format ???
 *  @return The current time stamp as an std::string
 */
std::string getTimeStamp();

} // namespace as64

#endif // TIME_LIB_TIME_64_H
