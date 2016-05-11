/*
 * Config.h
 *
 *  Created on: May 2, 2016
 *      Author: Nicolai Ommer <nicolai.ommer@gmail.com>
 */

#ifndef AFFW_CTRL_SRC_CONFIG_H_
#define AFFW_CTRL_SRC_CONFIG_H_

#include <string>
#include <map>

namespace affw {

class Config {
public:
	Config();
	virtual ~Config();

	bool read(const char* file);
	bool write(const char* file);
	void print();

	std::string getString(const std::string& key, const std::string& defValue);
	void setString(const std::string& key, const std::string& value);

	double getDouble(const std::string& key, double defValue);
	void setDouble(const std::string& key, double value);

	int getInt(const std::string& key, int defValue);
	void setInt(const std::string& key, int value);

	int getBool(const std::string& key, bool defValue);
	void setBool(const std::string& key, bool value);
private:
	std::map<std::string, std::string> map;
};

} /* namespace affw */

#endif /* AFFW_CTRL_SRC_CONFIG_H_ */
