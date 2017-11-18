#include <time_lib/time.h>

namespace as64
{

std::string getTimeStamp()
{
	std::time_t t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
	std::ostringstream out_s;
	out_s << std::ctime(&t);
	std::string time_stamp = out_s.str();
	for (int i=0;i<time_stamp.size();i++)
	{
		if (time_stamp[i]==' ') time_stamp[i] = '_';
		else if (time_stamp[i]==':') time_stamp[i] = '-';
	}

	return time_stamp;
}

} // namespace as64
