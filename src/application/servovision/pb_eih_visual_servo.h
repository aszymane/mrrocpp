/*
 * pb_eih_visual_servo.h
 *
 *  Created on: May 11, 2010
 *      Author: mboryn
 */

#ifndef PB_EIH_VISUAL_SERVO_H_
#define PB_EIH_VISUAL_SERVO_H_

#include "visual_servo.h"
#include "visual_servo_types.h"

using visual_servo_types::position_based_configuration;
using visual_servo_types::position_based_reading;

namespace mrrocpp {

namespace ecp {

namespace common {

namespace generator {

/** @addtogroup servovision
 *  @{
 */

/**
 *
 */
class pb_eih_visual_servo : public mrrocpp::ecp::common::generator::visual_servo
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	pb_eih_visual_servo( boost::shared_ptr <visual_servo_regulator> regulator, const std::string& section_name, mrrocpp::lib::configurator& configurator);
	virtual ~pb_eih_visual_servo();
	virtual lib::Homog_matrix get_position_change(const lib::Homog_matrix& current_position, double dt);
	virtual boost::shared_ptr <mrrocpp::lib::sensor> get_vsp_fradia();
protected:
	boost::shared_ptr <ecp_mp::sensor::fradia_sensor <position_based_reading, position_based_configuration> > vsp_fradia;

	lib::Homog_matrix G_T_E_desired;
	lib::Homog_matrix E_T_C;
};

/** @} */

} // namespace generator

}

}

}

#endif /* PB_EIH_VISUAL_SERVO_H_ */
