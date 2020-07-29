#include "config_reader.h"

ConfigReader::ConfigReader() {}
ConfigReader::~ConfigReader() {}

void ConfigReader::addConfigOption(std::string option, std::string default_val)
{
	this->config_info[option] = default_val;
}

bool ConfigReader::test_for_prescence(std::string item)
{
	return (bool) *( ( this->config_info[item] ).c_str() );
}