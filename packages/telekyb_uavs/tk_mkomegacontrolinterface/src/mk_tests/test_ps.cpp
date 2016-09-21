// Boost Filesystem and Regex to Find Serial
#include <boost/regex.hpp>
#include <boost/filesystem.hpp>
#include <sstream>

namespace fs = boost::filesystem;

int main(void){
  
  std::string path("/dev/");
  std::string filename("tty10");
  std::stringstream ss;
  
  ss << path << filename;
  
  std::cout << fs::is_directory("/dev/") << " " << fs::is_directory(path) << std::endl;
  std::cout << fs::is_directory("/dev/tty10") << " " << fs::is_directory(ss.str()) << std::endl;
  std::cout << fs::is_regular("/dev/tty10") << " " << fs::is_regular(ss.str()) << std::endl;
  std::cout << fs::exists("/dev/tty10") << " " << fs::exists(ss.str()) << std::endl;
  
  
  return 0;
}


