/*
 * $Id$
 *
 *  Created on: Mar 3, 2010
 *      Author: mboryn
 */

#include "visual_servo.h"

using namespace logger;

namespace mrrocpp {
namespace ecp {
namespace servovision {

visual_servo::visual_servo(boost::shared_ptr <visual_servo_regulator> regulator) :
	regulator(regulator), object_visible(false), max_steps_without_reading(5), steps_without_reading(0)
{

}

visual_servo::~visual_servo()
{
}

lib::Homog_matrix visual_servo::get_position_change(const lib::Homog_matrix& current_position, double dt)
{
	log_dbg("visual_servo::get_position_change() begin\n");
	log_dbg("visual_servo::get_position_change(): get_sensor_report()=%d\n", get_sensor_report());

	lib::Homog_matrix delta_position;

	if (get_sensor_report() == lib::sensor::VSP_SENSOR_NOT_CONFIGURED) {
		// Sensor not ready yet.
		return delta_position;
	} else if (get_sensor_report() == lib::sensor::VSP_READING_NOT_READY) {
		// Maybe there is a valid reading
		if (steps_without_reading > max_steps_without_reading) {
			// The object is no longer visible
			object_visible = false;
			log_dbg("pb_eih_visual_servo::get_position_change(): object considered no longer visible\n");
			return delta_position;
		} else {
			steps_without_reading++;
		}
	} else if (get_sensor_report() == lib::sensor::VSP_REPLY_OK) {
		// There's a reading, reset the counter.
		steps_without_reading = 0;
	}

	object_visible = is_object_visible_in_latest_reading();
	lib::Homog_matrix servoss=compute_position_change(current_position, dt);
	double tr[3], rot[3][3];
	servoss.get_translation_vector(tr);
	servoss.get_rotation_matrix(rot);
	printf("change: %g,%g,%g\n",tr[0], tr[1], tr[2]);
	printf("%g, %g, %g\n",rot[0][0], rot[0][1], rot[0][2]);
	printf("%g, %g, %g\n",rot[1][0], rot[1][1], rot[1][2]);
	printf("%g, %g, %g\n",rot[2][0], rot[2][1], rot[2][2]);

	if (object_visible) {
		printf("visible\n"); fflush(stdout);
		return compute_position_change(current_position, dt);
	}

	return delta_position;
} // get_position_change

bool visual_servo::is_object_visible()
{
	return object_visible;
}

const Eigen::Matrix <double, 6, 1> & visual_servo::get_error()
{
	return error;
}

} // namespace servovision
} // namespace ecp
} // namespace mrrocpp
