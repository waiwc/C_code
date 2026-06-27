#ifndef __VERIFY_IP_
#define __VERIFY_IP_

#include <iostream>
#include <vector>
#include <string>

namespace common{
namespace verify_ip{
	bool chkNumber(const std::string& str);
	std::vector<std::string> split(const std::string& str, char delim);
	bool validateIP(std::string ip);
}
}

#endif
