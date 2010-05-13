/*
 * ecp_g_image_switching.h
 *
 *  Created on: May 12, 2010
 *      Author: aszymane
 */

#ifndef ECP_G_IMG_SW_H_
#define ECP_G_IMG_SW_H_

#include "ecp/common/generator/ecp_generator.h"
#include "lib/mrmath/mrmath.h"
#include "image_switching_types.h"

namespace mrrocpp {

namespace ecp {

namespace irp6ot {

namespace generator {

/** @addtogroup image_switching
 *  @{
 */

class ecp_g_image_switching: public mrrocpp::ecp::common::generator::generator
{
public:
	ecp_g_image_switching(mrrocpp::ecp::common::task::task & _ecp_task);
	virtual ~ecp_g_image_switching();
	virtual bool first_step();
	virtual bool next_step();

	static const char configSectionName[];
protected:
	boost::shared_ptr <ecp_mp::sensor::fradia_sensor <image_based_reading, image_based_configuration> > vsp_fradia;
	/** Is log enabled*/
	bool logEnabled;
	/**
	 * Print message to the console only if logEnabled is set to true.
	 * @param fmt printf-like format
	 */
	void log(const char *fmt, ...);
	/**
	 * Check if frame is within constraints.
	 */
private:
	int licznik;
	double promien;
	lib::Homog_matrix currentFrame;
	double currentGripperCoordinate;
	double firstTransVector[3];

//	lib::sensor vsp_fradia;

	double Kp;
	double maxT;
};

/** @} */// ecp_g_image_switching

} // namespace generator

} // namespace irp6ot

} // namespace ecp

} // namespace mrrocpp

#endif /* ECP_G_IMG_SW_H_ */
