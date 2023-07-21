/****************************************************************************
 *
 *   Copyright (c) 2016-2018 PX4 Development Team. All rights reserved.
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

#pragma once

#include <stdlib.h>
#include <stdbool.h>

#include <lib/cdev/CDev.hpp>
#include <drivers/drv_hrt.h>

#include <uORB/Publication.hpp>
#include <uORB/topics/trisonica_status.h>

#include <px4_platform_common/atomic.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>

extern "C" __EXPORT int trisonica_mini_main(int argc, char *argv[]);

// class trisonica_mini : public cdev::CDev, public ModueBase<trisonica_mini>
// {
// 	public:
// 		trisonica_mini();
// 		~trisonica_mini();


// 		// Module related methods
// 		static int task_spawn(int argc, char *argv[]);
// 		static trisonica_mini *instantiate(int argc, char *argv[]);

// 		void run() override;

// 		int print_status() override;// might not be useful

// 		void test(int argc, char *argv[]);
// 		/*
// 		 * Passes everything to CDev
// 		 */
// 		// int ioctl(cdev::file_t *filp, int cmd, unsigned long arg);

// 		static bool can_stop() { return !get_instance()->_cdev_used.load(); }

// 	private:
// 		int init(int argc, char *argv[]);
// 		void deinit();
// 		bool read_from_sensor(uint8_t *rx_buf, int16_t timeout = 100);
// 		void publish_status();
// 		bool open_uart();
// 		pollevent_t poll_state(struct file *filp);
// 		virtual int	open_first(struct file *filep) override;
// 		virtual int	close_last(struct file *filep) override;

// 		// uORB publisher
// 		uORB::Publication<anemometer_s> _anemometer_pub{ORB_ID(anemometer)};
// 		uint8_t raw_data_size = 115;
// 		uint8_t raw_data[raw_data_size+10];
// 		pthread_mutex_t _buf_mutex = pthread_mutex_t();
// 		bool _verbose = false;
// 		int _uart_fd = -1;
// 		int _rx_msg_end_idx = 0;
// 		anemometer_s _anemometer_msg{};
// 		float convertBytesToFloat(const unsigned char* byteArray, int startByte, int endByte);
// 		int convertBytesToInt(const unsigned char* byteArray, int startByte, int endByte);
// };


class trisonica_mini : public cdev::CDev{
	public:
		/*
		* Constructor
		*/
		trisonica_mini();
		~trisonica_mini();
		/*
		* Starts the driver
		*/
		static int start(int argc, char *argv[]);

		static int stop();

		/*
		* Passes everything to CDev
		*/
		// int ioctl(cdev::file_t *filp, int cmd, unsigned long arg);
		static void main_loop_helper(int argc, char *argv[]);
		static trisonica_mini *instance;
		bool _task_should_exit = false;
		void deinit();
		bool _start_completed = false;
		double convert2double(char* byteArray, int startByte, int length);
		int convert2int(char* byteArray, int startByte, int length);
	private:
		// DEFINE_PARAMETERS(
		// 	(ParamInt<px4::params::SENS_EN_TRISONICA>) _param_sens_en_trisonica,
		// )

		void main_loop(int argc, char *argv[]);

		bool read_from_sensor(char *rx_buf, int16_t timeout = 100);

		void publish_status();
		bool open_uart(char *uart_name);
		// pollevent_t poll_state(struct file *filp);
		virtual int	open_first(struct file *filep) override;
		virtual int	close_last(struct file *filep) override;

		// uORB publisher
		uORB::Publication<trisonica_status_s> _trisonica_status_pub{ORB_ID(trisonica_status)};
		const static uint8_t raw_data_size = 115;
		char raw_data[raw_data_size+10];
		pthread_mutex_t _buf_mutex = pthread_mutex_t();
		bool _verbose = false;
		int _uart_fd = -1;
		int _rx_msg_end_idx = 0;
		trisonica_status_s _trisonica_msg{};
		// float convertBytesToFloat(char* byteArray, int startByte, int endByte);
		// int convertBytesToInt(char* byteArray, int startByte, int endByte);

		static int task_handle;
		// orb_advert_t _trisonica_status_pub = nullptr;

		bool _cdev_used = false;

};

int main_loop_helper_wrapper(int argc, char* argv[]);

