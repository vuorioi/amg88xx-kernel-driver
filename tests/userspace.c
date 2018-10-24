/*
 * A simple userspace application that allows to test the driver functionality
 *
 * Copyright (C) 2018  Iiro Vuorio
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <fcntl.h>
#include <poll.h>
#include <signal.h>
#include <sys/stat.h>
#include <unistd.h>

#define PAGE_SIZE 4096

#define device_mode_file	"/sys/bus/i2c/devices/1-0068/device_mode" 
#define interrupt_mode_file	"/sys/bus/i2c/devices/1-0068/interrupt_mode" 
#define interrupt_levels_file	"/sys/bus/i2c/devices/1-0068/interrupt_levels" 
#define interrupt_state_file	"/sys/bus/i2c/devices/1-0068/interrupt_state" 
#define interrupt_file		"/sys/bus/i2c/devices/1-0068/interrupt" 
#define sensor_file		"/sys/bus/i2c/devices/1-0068/sensor"
#define interrupt_map_file	"/sys/bus/i2c/devices/1-0068/interrupt_map"

#define COLOR_RED	"\x1b[31m"
#define COLOR_RESET	"\x1b[0m"
#define MOVE_CURSOR_UP	"\x1b[17A"

static int quit = 0;

/*
 * This function writes a value to a sysfs file
 * @file path to the file
 * @value the value that should be writen to the file
 *
 * Returns 1 on success and 0 otherwise
 */
static int write_value(const char* file, const char* value)
{
	int fd;
	size_t len;
	int ret = 0;

	len = strlen(value);

	fd = open(file, O_WRONLY);
	if (fd < 0) {
		perror("Failed to open file for writing");
		return 0;
	}

	ret = write(fd, value, len + 1);
	if (ret < 0) {
		perror("Failed to write to file");
		ret = 0;
		goto exit;
	} else if (ret != len + 1) {
		fprintf(stderr, "Write was only partially completed\n");
		ret = 0;
		goto exit;
	}

	ret = 1;

exit:
	close(fd);

	return ret;
}

/*
 * This function reads a value from a sysfs file
 * @file path to the file
 * @buf pointer to the buffer for the result
 * @size size of the buffer allocated for the result
 *
 * Returns 1 on success and 0 otherwise
 */
static ssize_t read_value(const char* file, char* buf, size_t size)
{
	int fd;
	int ret;

	fd = open(file, O_RDONLY);
	if (fd < 0) {
		perror("Failed to open file for writing");
		return 0;
	}

	ret = read(fd, buf, size);
	if (ret < 0) {
		perror("Failed to read the file\n");
		ret = 0;
		goto exit;
	}

	ret = 1;

exit:
	close(fd);

	return ret;
}

static inline int start_device()
{
	return write_value(device_mode_file, "normal");
}

static inline int stop_device()
{
	return write_value(device_mode_file, "sleep");
}

static int setup_interrupt()
{
	int ret;

	ret = write_value(interrupt_levels_file, "125,30,0");
	if (!ret)
		return ret;
	
	ret = write_value(interrupt_mode_file, "absolute");
	if (!ret)
		return ret;
	
	ret = write_value(interrupt_state_file, "enabled");
	if (!ret)
		return ret;
	
	return 1;
}

static int get_interrupt_state(int* state)
{
	char buffer[PAGE_SIZE];
	int ret;

	memset(buffer, 0, PAGE_SIZE);

	ret = read_value(interrupt_file, buffer, PAGE_SIZE);
	if (!ret)
		return ret;
	
	if (strcmp("active\n", buffer) == 0)
		*state = 1;
	else if (strcmp("not_active\n", buffer) == 0)
		*state = 0;
	else {
		printf("got int state: %s\n", buffer);
		return 0;
	}

	return 1;
}

static int wait_for_interrupt()
{
	int fd;
	struct pollfd interrupt_fd;
	char buffer[PAGE_SIZE];
	int ret = 0;

	fd = open(interrupt_file, O_RDONLY);
	if (fd < 0) {
		perror("Failed to open file for reading");
		goto exit;
	}
	
	// We need to do a dummy read before polling the file
	// otherwise the poll() will just return instantly
	ret = read(fd, buffer, PAGE_SIZE);
	if (ret < 0) {
		perror("Failed to read the file\n");
		ret = 0;
		goto exit;
	}

	interrupt_fd.fd = fd;
	interrupt_fd.events = POLLPRI | POLLERR;

	ret = poll(&interrupt_fd, 1, -1);
	if (ret < 0) {
		perror("Failed to poll the interrupt files");
		ret = 0;
		goto exit;
	} else if (ret) {
		if (interrupt_fd.revents & POLLPRI)
			ret = 1;
		else
			ret = 0;
	}

exit:
	close(fd);

	return ret;
}

struct pixel {
	float temp;
	int interrupt;
	int eol;
};

static int get_pixels(char* sensor, char* interrupts, struct pixel* result)
{
	int i;

	for (i = 0; i < 64; i++) {
		char* sensor_substr;
		int sensor_val;

		sensor_substr = strtok(i == 0 ? sensor : NULL, ",\n");
		if (sensor_substr == NULL)
			return 0;

		sensor_val = atoi(sensor_substr);
		result[i].temp = sensor_val * 0.25f;

		if ((i + 1) % 8 == 0)
			result[i].eol = 1;
	}

	for (i = 0; i < 64; i++) {
		char* interrupt_substr;

		interrupt_substr = strtok(i == 0 ? interrupts : NULL, ",\n");
		if (interrupt_substr == NULL)
			return 0;

		result[i].interrupt = atoi(interrupt_substr); 
	}

	return 1;
}

static inline void render_pixel(const struct pixel* pixel)
{
	printf("%s%4d%s%s",
	       pixel->interrupt ? COLOR_RED : COLOR_RESET,
	       (int)pixel->temp,
	       pixel->eol ? "\n" : "",
	       COLOR_RESET);
}

static int render_sensor()
{
	char sensor[PAGE_SIZE];
	char int_map[PAGE_SIZE];
	struct pixel result[64];
	int row;
	int col;
	int ret;

	ret = read_value(sensor_file, sensor, PAGE_SIZE);
	if (!ret)
		return 0;

	ret = read_value(interrupt_map_file, int_map, PAGE_SIZE);
	if (!ret)
		return 0;

	ret = get_pixels(sensor, int_map, result);
	if (!ret)
		return 0;

	for (row = 0; row < 8; row++) {
		printf("\n");

		for (col = 0; col < 8; col++)
			render_pixel(&result[row*8 + col]);
	}
	printf("\n");

	return 1;
}

void signal_handler(int signo)
{
	if (signo == SIGINT)
		quit = 1;
}

int main(int argc, char** argv)
{
	int ret;
	
	if (signal(SIGINT, signal_handler) == SIG_ERR)
		fprintf(stderr, "Failed to catch SIGINT. Shutdown will be ugly\n");

	ret = start_device();
	if (!ret)
		return -1;

	sleep(1);

	ret = setup_interrupt();
	if (!ret)
		return -1;

	printf("Waiting for an interrupt\n");

	while (!quit) {
		ret = wait_for_interrupt();
		if (!ret) {
			printf("Poll terminated but no interrupt was recieved\n");
			break;
		}

		while (1) {
			int int_state;

			ret = render_sensor();
			if (!ret)
				return -1;

			if (quit)
				break;

			printf(MOVE_UP);
			sleep(1);

			ret = get_interrupt_state(&int_state);
			if (!ret)
				return -1;

			if (int_state == 0)
				quit = 1;
		};
	
	}

	ret = stop_device();
	if (!ret)
		return -1;
	
	return 0;
}
