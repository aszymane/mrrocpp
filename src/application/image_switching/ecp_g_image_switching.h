/*
 * ecp_g_image_switching_new.h
 *
 *  Created on: Oct 12, 2010
 *      Author: aszymane
 */

#ifndef ECP_G_IMAGE_SWITCHING_NEW_H_
#define ECP_G_IMAGE_SWITCHING_NEW_H_


#include "base/ecp/ecp_generator.h"
#include "sensor/fradia/ecp_mp_s_fradia_sensor.h"
#include "base/lib/mrmath/mrmath.h"
#include "base/ecp_mp/ecp_mp_sensor.h"
#include "image_switching_types.h"
#include "../servovision/visual_servo_manager.h"

namespace mrrocpp {

namespace ecp {

namespace common {

namespace generator {

/** @addtogroup image_switching
 *  @{
 */

class ecp_g_image_switching_new : public common::generator::visual_servo_manager
{
public:
	ecp_g_image_switching_new(mrrocpp::ecp::common::task::task & ecp_task, const char * section_name, boost::shared_ptr <
			mrrocpp::ecp::servovision::visual_servo> vs);
	virtual ~ecp_g_image_switching_new();
	virtual bool first_step();
	virtual bool next_step();

//	static const char configSectionName[];
//	ecp_mp::sensor::fradia_sensor <image_switching_types::fradia_configuration, image_switching_types::position_based_reading, image_switching_types::position_based_searching> *vsp_fradia;

//	bool tracking;
//	bool found;
//	bool search;

//	image_switching_types::position_based_position readings;
//	image_switching_types::position_based_searching searching;
protected:
//	boost::shared_ptr <ecp_mp::sensor::fradia_sensor <image_based_reading, image_based_configuration> > vsp_fradia;
	/** Is log enabled*/
//	bool logEnabled;
	/**
	 * Print message to the console only if logEnabled is set to true.
	 * @param fmt printf-like format
	 */
//	void log(const char *fmt, ...);
	/**
	 * Check if frame is within constraints.
	 */
	virtual lib::Homog_matrix get_aggregated_position_change();
	virtual void configure_all_servos();
private:
//	int licznik;
//	double promien;
//	lib::Homog_matrix currentFrame;
//	double currentGripperCoordinate;
//	double firstTransVector[3];

//	lib::sensor vsp_fradia;
};

/** @} */// ecp_g_image_switching


} // namespace generator

} // namespace common

} // namespace ecp

} // namespace mrrocpp

#endif /* ECP_G_IMAGE_SWITCHING_NEW_H_ */
