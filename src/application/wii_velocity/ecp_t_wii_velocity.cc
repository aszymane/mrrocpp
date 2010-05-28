#include <string.h>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"
#include <fstream>

#include "lib/srlib.h"
#include "application/wii_teach/sensor/ecp_mp_s_wiimote.h"

#include "ecp/irp6ot_m/ecp_r_irp6ot_m.h"
#include "application/wii_velocity/ecp_t_wii_velocity.h"
#include "lib/mrmath/mrmath.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot_m {
namespace task {

wii_velocity::wii_velocity(lib::configurator &_config) :
	task(_config) {
	ecp_m_robot = new irp6ot_m::robot(*this);
	sr_ecp_msg->message("ECP loaded");

	//create Wii-mote virtual sensor object
	sensor_m[lib::SENSOR_WIIMOTE] = new ecp_mp::sensor::wiimote(
			lib::SENSOR_WIIMOTE, "[vsp_wiimote]", *this,
			sizeof(lib::sensor_image_t::sensor_union_t::wiimote_t));
	//configure the sensor
	sensor_m[lib::SENSOR_WIIMOTE]->configure_sensor();
}

void wii_velocity::main_task_algorithm(void) {
	eg = new generator::wii_velocity(*this);
	//	eg = new ecp_tff_nose_run_generator(*this,8);

	eg->sensor_m[lib::SENSOR_WIIMOTE] = sensor_m[lib::SENSOR_WIIMOTE];

	while (1) {
		eg->Move();
	}
}

}
} // namespace irp6ot

namespace common {
namespace task {

task* return_created_ecp_task(lib::configurator &_config) {
	return new irp6ot_m::task::wii_velocity(_config);
}

}
} // namespace common
} // namespace ecp
} // namespace mrrocpp


