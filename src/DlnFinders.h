#ifndef DLNFINDERS_H
#define DLNFINDERS_H

#include <libudev.h>
#include <string>

bool find_upper_dln(std::string& i2c_path);
bool find_lower_dln(std::string& i2c_path, std::string& iio_root,
                    int& gpio_base);
bool find_gps(std::string& gps_out);

#endif
