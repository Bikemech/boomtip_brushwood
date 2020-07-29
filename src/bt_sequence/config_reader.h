#ifndef CONFIG_READER_H
#define CONFIG_READER_H

#include <string>
#include <map>

class ConfigReader
{
public:
	ConfigReader();
	~ConfigReader();
	void addConfigOption(std::string option, std::string default_val);

private:
	bool test_for_prescence(std::string item);
	std::map<std::string, std::string> config_info;

}

#endif