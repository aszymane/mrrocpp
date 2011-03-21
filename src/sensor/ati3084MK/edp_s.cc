#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <cstdlib>
#include <cstdio>
#include <termios.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"
#include "base/lib/sr/srlib.h"

#include "base/edp/edp_e_manip.h"

#include "sensor/ati3084MK/edp_s.h"

// Konfigurator
#include "base/lib/configurator.h"

namespace mrrocpp {
namespace edp {
namespace sensor {

ATI3084_force::ATI3084_force(common::manip_effector &_master) :
	force(_master)
{
	sensor_frame = lib::Homog_matrix(0, 1, 0, 0, -1, 0, 0, 0, 0, 0, 1, 0.09);
	force_sensor_name = edp::sensor::FORCE_SENSOR_ATI3084;
}

void ATI3084_force::connect_to_hardware(void)
{

	uart = open_port();
	//printf("2\n");
	tcflush(uart, TCIFLUSH);

	sendBias();

}

ATI3084_force::~ATI3084_force(void)
{
	if (!force_sensor_test_mode) {
		disconnect_from_hardware();
	}

}

void ATI3084_force::disconnect_from_hardware(void)
{

	close(uart);
}

/**************************** inicjacja czujnika ****************************/
void ATI3084_force::configure_particular_sensor(void)
{// by Y

	sendBias();

}

void ATI3084_force::wait_for_particular_event()
{
	//	sr_msg->message("wait_for_event");


	ftxyz = getFT();

}

/***************************** odczyt z czujnika *****************************/
void ATI3084_force::get_particular_reading(void)
{

	lib::Ft_vector kartez_force;

	for (int i = 0; i < 6; i++) {
		ft_table[i] = static_cast <double> (ftxyz.ft[i]);
	}
	/*
	 char aaa[50];
	 sprintf(aaa,"%f",ft_table[0]);
	 sr_msg->message(aaa);g
	 */
	is_reading_ready = true;

	// jesli ma byc wykorzytstywana biblioteka transformacji sil
	if (gravity_transformation) {
		for (int i = 0; i < 3; i++) {
			ft_table[i] *= 10;
			ft_table[i] /= 115;
		}
		//ft_table[i]*=(10/115);
		//			for(int i=3;i<6;i++) ft_table[i]/=333;
		for (int i = 3; i < 6; i++) {
			ft_table[i] *= 5; // by Y - korekta 5/1000
			ft_table[i] /= 940; // by Y - korekta 5/1000
		}

	}

}

// metoda na wypadek skasowanie pamiecia nvram
// uwaga sterownik czujnika wysyla komunikat po zlaczu szeregowym zaraz po jego wlaczeniu

void ATI3084_force::solve_transducer_controller_failure(void)
{
	usleep(10);
	sendBias();
	usleep(100);
}

void ATI3084_force::sendBias()
{
	const char bias = 'b';
	if (write(uart, &bias, 1) != 1) {
		// TODO: throw
		perror("write()");
	}
}

ATI3084_force::forceReadings_t ATI3084_force::getFT()
{
	const char query = 'q';
	int r;
	int current_bytes = 14;
	int iter_counter = 0; // okresla ile razy pod rzad zostala uruchomiona petla

	//	printf("bbb \n");
	uint8_t reads[14];

	//	base_cycle = ClockCycles();

	do {
		r = 1;
		iter_counter++;

		if (write(uart, &query, 1) != 1) {
			// TODO: throw
			perror("write()");
		}
		//current_cycle = ClockCycles();
		while ((current_bytes > 0) && (r != 0)) {//printf("aaa: %d\n",r);
			r = read(uart, &(reads[14 - current_bytes]), current_bytes);

			current_bytes -= r;
		}
		//	current_cycle2 = ClockCycles();

		if (r == 0) {
			if (iter_counter == 1) {
				printf("Nie otrzymano oczekiwanej ilosci znakow: %d\n", r);
				sr_msg->message(lib::NON_FATAL_ERROR, "MK Force / Torque read error - check sensor controller");
			}

			solve_transducer_controller_failure();
		} else {
			if (iter_counter > 1) {
				sr_msg->message("MK Force / Torque sensor connection reastablished");
			}
		}
	} while (r == 0);

	forceReadings_t ftxyz;

	for (int i = 0; i < 6; i++) {
		ftxyz.ft[i] = (reads[2 * i]) << 8 | reads[2 * i + 1];
	}

	return ftxyz;
}

int ATI3084_force::open_port(void)
{
	int fd = open(PORT, O_RDWR);
	if (fd == -1) {
		perror("open()");
		return -1;
	}
	//printf("3\n");

	struct termios options;
	struct termios org_port_options;

	tcgetattr(fd, &org_port_options);
	options = org_port_options;

	cfsetispeed(&options, SPEED);
	cfsetospeed(&options, SPEED);

	options.c_cflag |= CLOCAL; // Local line - Do not change the owner
	options.c_cflag |= CREAD; // Enable receiver
	options.c_cflag &= ~PARENB; // parity - enabled
	options.c_cflag &= ~PARODD; // Odd parity - even
	options.c_cflag &= ~CSTOPB; // stop bit - 1
	options.c_cflag &= ~CSIZE; // bit mask for data bits
	options.c_cflag |= CS8; // data bits - 8
	options.c_lflag &= ~ECHO; // Echo the Terminal screen - off
	options.c_lflag &= ~ICANON; // Canonical Input else raw - disable
	options.c_iflag &= ~INPCK; // Parity Check - disable
	options.c_iflag &= ~IGNBRK; // Parity Errors - Ignore
	options.c_iflag &= ~PARMRK; // Mark Parity Errors - disable
	options.c_iflag &= ~ISTRIP; // Strip Parity bits - disable
	options.c_iflag &= ~IXON; // Software outgoing flow control - disable
	options.c_iflag &= ~IXOFF; // Software incoming flow control - disable
	options.c_oflag &= ~OPOST; // Post process output (not set = raw output)
	options.c_cc[VTIME] = 5;
	options.c_cc[VMIN] = 0;
	tcsetattr(fd, TCSADRAIN, &options);
	fcntl(fd, F_SETFL, 0);
	//	printf("4\n");
	return fd;
}

force* return_created_edp_force_sensor(common::manip_effector &_master)
{
	return new ATI3084_force(_master);
}

} // namespace sensor
} // namespace edp
} // namespace mrrocpp
