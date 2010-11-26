/*
 * simple_binary_image_switching.h
 *
 *  Created on: Oct 13, 2010
 *      Author: aszymane
 */

#ifndef SIMPLE_BINARY_IMAGE_SWITCHING_H_
#define SIMPLE_BINARY_IMAGE_SWITCHING_H_

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
class simple_binary_image_switching : public visual_servo_manager
{
public:
	simple_binary_image_switching(mrrocpp::ecp::common::task::task & ecp_task, const char * section_name1, boost::shared_ptr <
					mrrocpp::ecp::servovision::visual_servo> vs1, const char * section_name2, boost::shared_ptr <
					mrrocpp::ecp::servovision::visual_servo> vs2);
	virtual ~simple_binary_image_switching();
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

#endif /* SIMPLE_BINARY_IMAGE_SWITCHING_H_ */
