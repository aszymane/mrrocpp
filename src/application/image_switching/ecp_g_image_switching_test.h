/*
 * ecp_g_image_switching_test.h
 *
 *  Created on: Oct 13, 2010
 *      Author: aszymane
 */

#ifndef ECP_G_IMAGE_SWITCHING_TEST_H_
#define ECP_G_IMAGE_SWITCHING_TEST_H_

#include "../servovision/visual_servo_manager.h"

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
class ecp_g_image_switching_test : public visual_servo_manager
{
public:
			ecp_g_image_switching_test(mrrocpp::ecp::common::task::task & ecp_task, const char * section_name, boost::shared_ptr <
					mrrocpp::ecp::servovision::visual_servo> vs1, boost::shared_ptr <mrrocpp::ecp::servovision::visual_servo> vs2);
	virtual ~ecp_g_image_switching_test();
protected:
	virtual lib::Homog_matrix get_aggregated_position_change();
	virtual void configure_all_servos();
	int state;
};

/** @} */

}//namespace generator

}

}

}

#endif /* ECP_G_IMAGE_SWITCHING_TEST_H_ */
