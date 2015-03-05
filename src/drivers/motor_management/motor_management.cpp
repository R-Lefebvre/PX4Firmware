/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
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

/**
 * @file motor_management.cpp
 *
 * Driver for a motor management device connected via SMBus (I2C).
 *
 * @author Robert Lefebvre <robert.lefebvre@gmail.com>
 */

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <sched.h>
#include <semaphore.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <ctype.h>

#include <nuttx/arch.h>
#include <nuttx/wqueue.h>
#include <nuttx/clock.h>

#include <board_config.h>

#include <systemlib/perf_counter.h>
#include <systemlib/err.h>
#include <systemlib/systemlib.h>

#include <uORB/uORB.h>
#include <uORB/topics/subsystem_info.h>
#include <uORB/topics/motor_management.h>

#include <float.h>

#include <drivers/device/i2c.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_motor_management.h>
#include <drivers/device/ringbuffer.h>

#define MOTOR_MANAGEMENT_BUS			PX4_I2C_BUS_EXPANSION
#define MOTOR_MANAGEMENT_ADDR					0x36	///< I2C address

#define MOTOR_MANAGEMENT_REQUEST_PPM_1				0x20
#define MOTOR_MANAGEMENT_REQUEST_PPM_2				0x21
#define MOTOR_MANAGEMENT_REQUEST_PPM_3				0x22
#define MOTOR_MANAGEMENT_REQUEST_PPM_4				0x23
#define MOTOR_MANAGEMENT_REQUEST_TEMP_1				0x24
#define MOTOR_MANAGEMENT_REQUEST_TEMP_2				0x25
#define D_BUFF_PPM_1								0
#define D_BUFF_PPM_2								1
#define D_BUFF_PPM_3								2
#define D_BUFF_PPM_4								3
#define D_BUFF_TEMP_1								4
#define D_BUFF_TEMP_2								5
#define NUM_FLOAT_DATA								6
#define BYTES_PER_FLOAT								4

#define MOTOR_MANAGEMENT_MEASUREMENT_INTERVAL_MS		(1000000 / 20)	///< time in microseconds, measure at 20hz
#define MOTOR_MANAGEMENT_TIMEOUT_MS		10000000	///< timeout looking for motor manager 10seconds after startup

#ifndef CONFIG_SCHED_WORKQUEUE
# error This requires CONFIG_SCHED_WORKQUEUE.
#endif

class MOTOR_MANAGEMENT : public device::I2C
{
public:
	MOTOR_MANAGEMENT();
	//Destructor
	virtual ~MOTOR_MANAGEMENT();

	/**
	 * Initialize device
	 *
	 * Calls probe() to check for device on bus.
	 *
	 * @return 0 on success, error code on failure
	 */
	virtual int		init();

	/**
	 * Test device
	 *
	 * @return 0 on success, error code on failure
	 */
	virtual int		test();

	/**
	 * Search all possible slave addresses for a motor manager
	 */
	int			search();

protected:
	/**
	 * Check if the device can be contacted
	 */
	virtual int		probe();

private:

	/**
	 * Start periodic reads from the motor manager
	 */
	void			start();

	/**
	 * Stop periodic reads from the motor manager
	 */
	void			stop();

	/**
	 * static function that is called by worker queue
	 */
	static void		cycle_trampoline(void *arg);

	/**
	 * perform a read from the motor manager
	 */
	void			cycle();

	/**
	 * Read a word from specified register
	 */
	int			read_reg(uint8_t reg, uint16_t &val);

	/**
	 * Read block from bus
	 * @return returns number of characters read if successful, zero if unsuccessful
	 */
	uint8_t			read_block(uint8_t reg, uint8_t *data, uint8_t max_len);

	// internal variables
	bool			_enabled;	///< true if we have successfully connected to motor manager
	work_s			_work;		///< work queue for scheduling reads
	RingBuffer		*_reports;	///< buffer of recorded voltages, currents
	struct motor_management_s _last_report;	///< last published report, used for test()
	orb_advert_t		_motor_management_topic;	///< uORB motor management topic
	orb_id_t		_motor_management_orb_id;	///< uORB motor management ID
	uint64_t		_start_time;	///< system time we first attempt to communicate with motor manager
};

namespace
{
MOTOR_MANAGEMENT *g_motor_management;	///< device handle. 
}

void motor_management_usage();

extern "C" __EXPORT int motor_management_main(int argc, char *argv[]);

MOTOR_MANAGEMENT::MOTOR_MANAGEMENT() :
	I2C("motor_management", MOTOR_MANAGEMENT0_DEVICE_PATH, MOTOR_MANAGEMENT_BUS,
		MOTOR_MANAGEMENT_ADDR, 100000),
	_enabled(false),
	_work{},
	_reports(nullptr),
	_motor_management_topic(-1),
	_motor_management_orb_id(nullptr),
	_start_time(0)
{
	// work_cancel in the dtor will explode if we don't do this...
	memset(&_work, 0, sizeof(_work));

	// capture startup time
	_start_time = hrt_absolute_time();
}

MOTOR_MANAGEMENT::~MOTOR_MANAGEMENT()
{
	// make sure we are truly inactive
	stop();

	if (_reports != nullptr) {
		delete _reports;
	}
}

int
MOTOR_MANAGEMENT::init()
{
	int ret = ENOTTY;

	// attempt to initialise I2C bus
	ret = I2C::init();

	if (ret != OK) {
		errx(1, "failed to init I2C");
		return ret;

	} else {
		// allocate basic report buffers
		_reports = new RingBuffer(2, sizeof(struct motor_management_s));

		if (_reports == nullptr) {
			ret = ENOTTY;

		} else {
			// start work queue
			start();
		}
	}

	// init orb id
	_motor_management_orb_id = ORB_ID(motor_management);

	return ret;
}

int
MOTOR_MANAGEMENT::test()
{
	int sub = orb_subscribe(ORB_ID(motor_management));
	bool updated = false;
	struct motor_management_s status;
	uint64_t start_time = hrt_absolute_time();

	// loop for 5 seconds
	while ((hrt_absolute_time() - start_time) < 5000000) {

		// display new info that has arrived from the orb
		orb_check(sub, &updated);

		if (updated) {
			if (orb_copy(ORB_ID(motor_management), sub, &status) == OK) {
				warnx("PPM 1=%5.1f Temp 1 =%2.4f", status.ppm_1, status.temp_1);
			}
		}

		// sleep for 0.05 seconds
		usleep(50000);
	}

	return OK;
}

int
MOTOR_MANAGEMENT::search()
{
	bool found_slave = false;
	uint8_t tmp[4];

	if (read_block(MOTOR_MANAGEMENT_REQUEST_TEMP_1, tmp, 4) == 4){
		warnx("motor manager found at 0x%x", (int)MOTOR_MANAGEMENT_ADDR);
		found_slave = true;
	}

	// display completion message
	if (found_slave) {
		warnx("Done.");

	} else {
		warnx("No motor managers found.");
	}

	return OK;
}

int
MOTOR_MANAGEMENT::probe()
{
	// always return OK to ensure device starts
	return OK;
}

void
MOTOR_MANAGEMENT::start()
{
	// reset the report ring and state machine
	_reports->flush();

	// schedule a cycle to start things
	work_queue(HPWORK, &_work, (worker_t)&MOTOR_MANAGEMENT::cycle_trampoline, this, 1);
}

void
MOTOR_MANAGEMENT::stop()
{
	work_cancel(HPWORK, &_work);
}

void
MOTOR_MANAGEMENT::cycle_trampoline(void *arg)
{
	MOTOR_MANAGEMENT *dev = (MOTOR_MANAGEMENT *)arg;

	dev->cycle();
}

void
MOTOR_MANAGEMENT::cycle()
{
	// get current time
	uint64_t now = hrt_absolute_time();

	// exit without rescheduling if we have failed to find a motor manager after 10 seconds
	if (!_enabled && (now - _start_time > MOTOR_MANAGEMENT_TIMEOUT_MS)) {
		warnx("did not find motor manager");
		return;
	}

	// read data from sensor
	struct motor_management_s new_report;

	// set time of reading
	new_report.timestamp = now;

	union D_Buff {uint8_t D_Buff_byte[BYTES_PER_FLOAT * NUM_FLOAT_DATA]; float D_Buff_float[NUM_FLOAT_DATA];} D_Buff_Union;

	if (read_block(MOTOR_MANAGEMENT_REQUEST_PPM_1, D_Buff_Union.D_Buff_byte,
		BYTES_PER_FLOAT * NUM_FLOAT_DATA) == OK) {

		// initialise new_report
		memset(&new_report, 0, sizeof(new_report));

		new_report.ppm_1 = D_Buff_Union.D_Buff_float[D_BUFF_PPM_1];
		new_report.ppm_2 = D_Buff_Union.D_Buff_float[D_BUFF_PPM_2];
		new_report.ppm_3 = D_Buff_Union.D_Buff_float[D_BUFF_PPM_3];
		new_report.ppm_4 = D_Buff_Union.D_Buff_float[D_BUFF_PPM_4];
		new_report.temp_1 = D_Buff_Union.D_Buff_float[D_BUFF_TEMP_1];
		new_report.temp_2 = D_Buff_Union.D_Buff_float[D_BUFF_TEMP_2];

		// publish to orb
		if (_motor_management_topic != -1) {
			orb_publish(_motor_management_orb_id, _motor_management_topic, &new_report);
		} else {
			_motor_management_topic = orb_advertise(_motor_management_orb_id, &new_report);

			if (_motor_management_topic < 0) {
				errx(1, "ADVERT FAIL");
			}
		}

		// copy report for test()
		_last_report = new_report;

		// post a report to the ring
		_reports->force(&new_report);

		// notify anyone waiting for data
		poll_notify(POLLIN);

		// record we are working
		_enabled = true;
	}

	// schedule a fresh cycle call when the measurement is done
	work_queue(HPWORK, &_work, (worker_t)&MOTOR_MANAGEMENT::cycle_trampoline, this,
		   USEC2TICK(MOTOR_MANAGEMENT_MEASUREMENT_INTERVAL_MS));
}

int
MOTOR_MANAGEMENT::read_reg(uint8_t reg, uint16_t &val)
{
	uint8_t buff[1];	// 1 byte of data

	// read from register
	int ret = transfer(&reg, 1, buff, 1);

	// return success or failure
	return ret;
}

uint8_t
MOTOR_MANAGEMENT::read_block(uint8_t reg, uint8_t *data, uint8_t max_len)
{

	uint8_t buff_out[2] {reg, max_len};
	uint8_t buff_in[max_len];  // buffer to hold results

	usleep(1);

	// read bytes
	int ret = transfer(buff_out, 2, buff_in, max_len);

	// return zero on failure
	if (ret != OK) {
		return ret;
	}

	// copy data
	memcpy(data, buff_in, max_len);

	// return success
	return ret;
}

///////////////////////// shell functions ///////////////////////

void
motor_management_usage()
{
	warnx("missing command: try 'start', 'test', 'stop', 'search'");
}

int
motor_management_main(int argc, char *argv[])
{

	int ch;

	// jump over start/off/etc and look at options first
	while ((ch = getopt(argc, argv, "a:b:")) != EOF) {
		switch (ch) {

		default:
			motor_management_usage();
			exit(0);
		}
	}

	if (optind >= argc) {
		motor_management_usage();
		exit(1);
	}

	const char *verb = argv[optind];

	if (!strcmp(verb, "start")) {
		if (g_motor_management != nullptr) {
			errx(1, "already started");

		} else {
			// create new global object
			g_motor_management = new MOTOR_MANAGEMENT();

			if (g_motor_management == nullptr) {
				errx(1, "new failed");
			}

			if (OK != g_motor_management->init()) {
				delete g_motor_management;
				g_motor_management = nullptr;
				errx(1, "init failed");
			}
		}

		exit(0);
	}

	/* need the driver past this point */
	if (g_motor_management == nullptr) {
		warnx("not started");
		motor_management_usage();
		exit(1);
	}

	if (!strcmp(verb, "test")) {
		g_motor_management->test();
		exit(0);
	}

	if (!strcmp(verb, "stop")) {
		delete g_motor_management;
		g_motor_management = nullptr;
		exit(0);
	}

	if (!strcmp(verb, "search")) {
		g_motor_management->search();
		exit(0);
	}

	motor_management_usage();
	exit(0);
}
