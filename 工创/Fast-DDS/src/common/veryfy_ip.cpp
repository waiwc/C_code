#include "common/verify_ip.hpp"

namespace common{
namespace verify_ip{

bool chkNumber(const std::string& str){
   return !str.empty() &&
   (str.find_first_not_of("[0123456789]") == std::string::npos);
}

std::vector<std::string> split(const std::string& str, char delim){
   auto i = 0;
   std::vector<std::string> list;
   auto pos = str.find(delim);
   while (pos != std::string::npos){
      list.push_back(str.substr(i, pos - i));
      i = ++pos;
      pos = str.find(delim, pos);
   }
   list.push_back(str.substr(i, str.length()));
   return list;
}
bool validateIP(std::string ip){
   std::vector<std::string> slist = split(ip, '.');
   if (slist.size() != 4)
      return false;
   for (std::string str : slist){
      if (!chkNumber(str) || stoi(str) < 0 || stoi(str) > 255)
         return false;
   }
   return true;
}

}
}
