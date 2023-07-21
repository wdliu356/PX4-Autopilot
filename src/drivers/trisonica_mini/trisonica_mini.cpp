/****************************************************************************
 *
 *   Copyright (c) 2016-2023 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "trisonica_mini.h"
#include <px4_platform_common/tasks.h>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <pthread.h>
// #include<iostream>
#include <systemlib/err.h>
#include <parameters/param.h>
#include <cstdlib>


#define TRISONICA_MINI_DEVICE_PATH	"/dev/trisonica_mini"
#define TRISONICA_MINI_DEVICE_BAUDRATE	115200
#define VERBOSE_INFO(...) if (_verbose) { PX4_INFO(__VA_ARGS__); }
trisonica_mini *trisonica_mini::instance;
int trisonica_mini::task_handle;

trisonica_mini::trisonica_mini() : CDev(TRISONICA_MINI_DEVICE_PATH)
{
	_trisonica_status_pub.advertise();
}

trisonica_mini::~trisonica_mini() {
	deinit();
}

void trisonica_mini::deinit()
{
	if (_uart_fd >= 0) {
		/* discard all pending data, as close() might block otherwise on NuttX with flow control enabled */
		tcflush(_uart_fd, TCIOFLUSH);
		::close(_uart_fd);
		_uart_fd = -1;
	}
}

int trisonica_mini::start(int argc,char *argv[]){
	PX4_INFO("starting");

	if (trisonica_mini::instance != nullptr) {
		PX4_WARN("already started");
		return PX4_ERROR;
	}
	trisonica_mini::instance = new trisonica_mini();
	trisonica_mini::task_handle = px4_task_spawn_cmd("trisonica_mini",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_DEFAULT,
				      2000,
				      (main_t)&main_loop_helper_wrapper,
				      argv);
	int counter = 0;
	trisonica_mini::instance->_start_completed = false;
	trisonica_mini::instance->_task_should_exit = false;
	while (!trisonica_mini::instance->_start_completed && counter < 100 && (trisonica_mini::task_handle != -1)) {
		usleep(100000);
		counter++;
	}
	if (trisonica_mini::instance->_start_completed && (trisonica_mini::task_handle != -1)) {
		return PX4_OK;
	} else {
		trisonica_mini::instance->_task_should_exit = true;
		for (counter = 0; (counter < 10+1) && (trisonica_mini::task_handle != -1); counter++) {
			sleep(1);
		}
		return PX4_ERROR;
	}
}

int trisonica_mini::stop(){
	if (trisonica_mini::instance == nullptr) {
		PX4_WARN("not running");
		return PX4_ERROR;
	}
	if (trisonica_mini::instance->_cdev_used){
		PX4_WARN("driver is in use. Stop all users");
		return PX4_ERROR;
	}
	PX4_WARN("stopping");
	trisonica_mini::instance->_task_should_exit = true;
	for (int counter = 0; (counter < 10+1) && (trisonica_mini::task_handle != -1); counter++) {
		sleep(1);
	}
	if (trisonica_mini::task_handle != -1) {
		PX4_WARN("driver could not be stopped,kill it forcefully");
		::close(trisonica_mini::instance->_uart_fd);
		task_delete(trisonica_mini::task_handle);
		trisonica_mini::task_handle = -1;
		delete trisonica_mini::instance;
		trisonica_mini::instance = nullptr;
		return PX4_ERROR;
	}
	return OK;
}

int main_loop_helper_wrapper(int argc, char* argv[]) {
  trisonica_mini::main_loop_helper(argc, argv);
  return 0;
}

void trisonica_mini::main_loop_helper(int argc, char *argv[]){
	trisonica_mini::instance->main_loop(argc, argv);

	::close(trisonica_mini::instance->_uart_fd);
	trisonica_mini::task_handle = -1;
	delete trisonica_mini::instance;
	trisonica_mini::instance = nullptr;

	PX4_WARN("exiting");
}

void trisonica_mini::main_loop(int argc, char *argv[]){
	CDev::init();
	// if (ret != PX4_OK){
	// 	PX4_ERR("CDev init failed");
	// 	return ;
	// }

	pthread_mutex_init(&_buf_mutex, NULL);

	int arg_i = 3;
	int arg_uart_name = 0;

	while (arg_i < argc){
		if (!strcmp(argv[arg_i], "-d")){
			if (argc > arg_i + 1){
				arg_i++;
				arg_uart_name = arg_i;
			} else {
				PX4_ERR("missing parameter");
				return;
			}
		}else if (!strcmp(argv[arg_i], "-v")) {
			PX4_INFO("verbose mode ON");
			_verbose = true;
		}
		else {
			PX4_ERR("unknown parameter");
			return;
		}
		arg_i++;
	}

	if (arg_uart_name == 0){
		PX4_ERR("No serial port specified.");
		return;
	}

	bool command_executed = false;
	for (int counter = 0; (counter < 20) && !command_executed; counter++) {
		if (open_uart(argv[arg_uart_name]) == true) {
			command_executed = true;

		} else {
			usleep(100000);
		}
	}
	if (!command_executed) {
		PX4_ERR("failed to open UART port!");
		_task_should_exit = true;
		return;
	}
	_start_completed = true;
	while(!_task_should_exit){
		bool data_read = read_from_sensor(raw_data,100);
		if (data_read){
			publish_status();
		}
		usleep(100000);
	}


}

// int trisonica_mini::task_spawn(int argc, char *argv[]){
// 	_task_id = px4_task_spawn_cmd("trisonica_mini",
// 				      SCHED_DEFAULT,
// 				      SCHED_PRIORITY_DEFAULT,
// 				      2000,
// 				      (px4_main_t)&run_trampoline,
// 				      (char *const *)argv);

// 	if (_task_id < 0){
// 		delete instance;
// 		return -errno;
// 	}

// 	// Wait until task is up & running
// 	if (wait_until_running(6000) < 0){
// 		return -1;
// 	}

// 	return 0;
// }

// trisonica_mini *trisonica_mini::instantiate(int argc, char *argv[]){
// 	trisonica_mini *instance = new trisonica_mini();

// 	if (instance){
// 		int ret = instance->init(argc, argv);
// 		if (ret != PX4_OK){
// 			PX4_ERR("trisonica_mini init failed");
// 			delete instance;
// 			instance = nullptr;
// 		}


// 	}

// 	return instance;
// }

// int trisonica_mini::init(int argc, char *argv[]){
// 	int ret = CDev::init();
// 	if (ret != PX4_OK){
// 		PX4_ERR("CDev init failed");
// 		return ret;
// 	}

// 	pthread_mutex_init(&_buf_mutex, NULL);

// 	int arg_i = 1;
// 	int arg_uart_name = 0;

// 	while (arg_i < argc){
// 		if (!strcmp(argv[arg_i], "-d")){
// 			if (argc > arg_i + 1){
// 				arg_i++;
// 				arg_uart_name = arg_i;
// 			} else {
// 				PX4_ERR("missing parameter");
// 				return -1;
// 			}
// 		}else if (!strcmp(argv[arg_i], "-v")) {
// 			PX4_INFO("verbose mode ON");
// 			_verbose = true;
// 		}
// 		else {
// 			PX4_ERR("unknown parameter");
// 			return -1;
// 		}
// 		arg_i++;
// 	}

// 	if (arg_uart_name == 0){
// 		PX4_ERR("No serial port specified.");
// 		return -1;
// 	}

// 	bool command_executed = false;
// 	for (int counter = 0; (counter < 20) && !command_executed; counter++) {
// 		if (open_uart(argv[arg_uart_name]) == true) {
// 			command_executed = true;

// 		} else {
// 			usleep(100000);
// 		}
// 	}
// 	if (!command_executed) {
// 		PX4_ERR("failed to open UART port!");
// 		return -EIO;
// 	}
// 	return PX4_OK;
// }

// void trisonica_mini::deinit(){
// 	if (_uart_fd >= 0) {
// 		/* discard all pending data, as close() might block otherwise on NuttX with flow control enabled */
// 		tcflush(_uart_fd, TCIOFLUSH);
// 		::close(_uart_fd);
// 		_uart_fd = -1;
// 	}
// }



bool trisonica_mini::open_uart(char *uart_name)
{
	VERBOSE_INFO("opening trisonica mini UART: %s", uart_name);

	_uart_fd = ::open(uart_name, O_RDWR | O_BINARY);

	if (_uart_fd < 0) {
		VERBOSE_INFO("UART open failed!");
		return false;
	}

	// set the UART speed to 115200
	struct termios uart_config;
	tcgetattr(_uart_fd, &uart_config);
	cfsetspeed(&uart_config, 115200);
	tcsetattr(_uart_fd, TCSANOW, &uart_config);

	VERBOSE_INFO("UART opened");

	return true;
}

// void trisonica_mini::run(){
// 	while(!should_exit()){
// 		bool data_read = read_from_sensor(raw_data,100);
// 		if (data_read){
// 			publish_status();
// 		}
// 		usleep(100000);
// 	}
// }

bool trisonica_mini::read_from_sensor(char *rx_buf,  int16_t timeout){

	struct pollfd fds[1];
	fds[0].fd = _uart_fd;
	fds[0].events = POLLIN;

	char buf = 0;
	// int last_rn_idx = 0;
	int rx_buf_pos = 0;
	// *rx_len = 0;
	pthread_mutex_lock(&_buf_mutex);
	while(1){
		if (::poll(&fds[0], 1, timeout) > 0) {
			if (::read(_uart_fd, &buf, 1) > 0) {
				if (rx_buf_pos == 0 && (buf == '\r' || buf == '\n')) {
					// ignore the leading \r\n
					continue;
				}

				rx_buf[rx_buf_pos++] = buf;
				if (rx_buf[rx_buf_pos - 1] == '\n' && rx_buf[rx_buf_pos - 2] == '\r') {
					if (rx_buf_pos == raw_data_size && rx_buf[0] == 'S') {
						return true;
					}
					else{
						memset(rx_buf, 0, raw_data_size+10);
						pthread_mutex_unlock(&_buf_mutex);
						return false;
					}
				}


			} else {
				pthread_mutex_unlock(&_buf_mutex);
				PX4_ERR("read error");
				return false;
			}

		}
		else {
			pthread_mutex_unlock(&_buf_mutex);
			PX4_ERR("poll error");
			return false;
		}
}}

void trisonica_mini::publish_status(){
	_trisonica_msg.timestamp = hrt_absolute_time();
	// VERBOSE_INFO("raw data !!!!!!!!!!!!!!!!!!: %s", raw_data);
	VERBOSE_INFO("raw data: %s", raw_data);
	for (int i = 0; i < raw_data_size;i++){
		if (raw_data[i] == 'S' && raw_data[i+1] !='2'){
			_trisonica_msg.wind_speed3d = convert2double(raw_data, i+3, 5);
			i+=7;
			VERBOSE_INFO("wind speed 3d: %f", double(_trisonica_msg.wind_speed3d))
		}else if(raw_data[i] =='S' && raw_data[i+1] =='2'){
			_trisonica_msg.wind_speed2d = convert2double(raw_data, i+4, 5);
			i+=8;
			VERBOSE_INFO("wind speed 2d: %f", double(_trisonica_msg.wind_speed2d))
		}else if (raw_data[i] =='D'){
			_trisonica_msg.horizontal_wind_direction = int(convert2double(raw_data, i+3, 3))/10;
			i+=5;
			VERBOSE_INFO("horizontal wind direction: %d", int(_trisonica_msg.horizontal_wind_direction))
		}else if (raw_data[i] =='U') {
			_trisonica_msg.u_vector = convert2double(raw_data, i+3, 5);
			i+=7;
			VERBOSE_INFO("u vector: %f", double(_trisonica_msg.u_vector))
		}else if (raw_data[i] =='V') {
			_trisonica_msg.v_vector = convert2double(raw_data, i+3, 5);
			i+=7;
			VERBOSE_INFO("v vector: %f", double(_trisonica_msg.v_vector))
		}else if (raw_data[i] =='W') {
			_trisonica_msg.w_vector = convert2double(raw_data, i+3, 5);
			i+=7;
			VERBOSE_INFO("w vector: %f", double(_trisonica_msg.w_vector))
		}else if (raw_data[i] =='T'  && raw_data[i+1] !='D'){
			_trisonica_msg.temperature = convert2double(raw_data, i+3, 5);
			i+=7;
			VERBOSE_INFO("temperature: %f", double(_trisonica_msg.temperature))
		}else if (raw_data[i] =='P'  && raw_data[i+1] !='I') {
			_trisonica_msg.pressure = convert2double(raw_data, i+3, 6);
			i+=8;
			VERBOSE_INFO("pressure: %f", double(_trisonica_msg.pressure))
		}else if (raw_data[i] =='A'  && raw_data[i+1] =='D') {
			_trisonica_msg.air_density = convert2double(raw_data, i+4, 9);
			i+=12;
			VERBOSE_INFO("air density: %f", double(_trisonica_msg.air_density))
		}else if (raw_data[i] =='P'  && raw_data[i+1] =='I'){
			_trisonica_msg.pitch = convert2double(raw_data, i+4, 5);
			i+=8;
			VERBOSE_INFO("pitch: %f", double(_trisonica_msg.pitch))
		}else if (raw_data[i] =='R' && raw_data[i+1] =='O'){
			_trisonica_msg.roll = convert2double(raw_data, i+4, 5);
			i+=8;
			VERBOSE_INFO("roll: %f", double(_trisonica_msg.roll))
		}else if (raw_data[i] =='T' && raw_data[i+1] =='D' ){
			_trisonica_msg.true_heading = int(convert2double(raw_data, i+4, 3))/10;
			i+=6;
			VERBOSE_INFO("true heading: %d", int(_trisonica_msg.true_heading))
		}
	}

	_trisonica_status_pub.publish(_trisonica_msg);
	memset(raw_data, 0, raw_data_size+10);
	pthread_mutex_unlock(&_buf_mutex);
}

// float trisonica_mini::convertBytesToFloat(char* byteArray, int startByte, int endByte) {
//     int intValue = 0;
//     for (int i = startByte; i <= endByte; i++) {
//         intValue = (intValue << 8) | byteArray[i];
//     }
//     float floatValue = *reinterpret_cast<float*>(&intValue);
//     return floatValue;
// }

// int trisonica_mini::convertBytesToInt(char* byteArray, int startByte, int endByte) {
//     int intValue = 0;
//     for (int i = startByte; i <= endByte; i++) {
//         intValue = (intValue << 8) | byteArray[i];
//     }
//     return intValue;
// }

double trisonica_mini::convert2double(char* byteArray, int startByte, int length) {
    char* doubleArray = new char[length];
    std::strncpy(doubleArray, byteArray + startByte, length);
//     doubleArray[length] = '\0';
    double doubleValue = std::strtod(doubleArray, NULL);
    delete[] doubleArray;
    if (byteArray[startByte - 1] == '-') {
	doubleValue = -doubleValue;
    }
    return doubleValue;
}

int trisonica_mini::convert2int(char* byteArray, int startByte, int length) {
    char* intArray = new char[length];
    std::strncpy(intArray, byteArray + startByte, length);
//     doubleArray[length] = '\0';
    int intValue = std::atoi(intArray);
    delete[] intArray;
    if (byteArray[startByte - 1] == '-') {
	intValue = -intValue;
    }
    return intValue;
}

int trisonica_mini::open_first(struct file *filep)
{
	_cdev_used = true;
	return CDev::open_first(filep);
}

int trisonica_mini::close_last(struct file *filep)
{
	_cdev_used = false;
	return CDev::close_last(filep);
}

int trisonica_mini_main(int argc, char *argv[]){
	if (argc < 2){
		PX4_ERR("missing command");
		return PX4_ERROR;
	}
	if (!strcmp(argv[1], "start")){
		PX4_INFO("starting");
		return trisonica_mini::start(argc, argv);
	}else if (!strcmp(argv[1], "stop")){
		PX4_INFO("stopping");
		return trisonica_mini::stop();
	}else if (!strcmp(argv[1], "status")){
		if (trisonica_mini::instance == nullptr){
			PX4_INFO("not running");
			return PX4_OK;
		}
		if (trisonica_mini::instance->_task_should_exit){
			PX4_INFO("stopped");
			return PX4_OK;
		}
		PX4_INFO("running");
		return PX4_OK;
	}else{
		PX4_ERR("unknown command");
		return PX4_ERROR;
	}
}

