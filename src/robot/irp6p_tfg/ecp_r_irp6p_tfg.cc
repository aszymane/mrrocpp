/*!
 * @file
 * @brief File contains ecp robot class definition for IRp6 postument two finger gripper
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup irp6p_tfg
 */

#include "robot/irp6p_tfg/ecp_r_irp6p_tfg.h"
#include "robot/irp6p_tfg/kinematic_model_irp6p_tfg.h"

namespace mrrocpp {
namespace ecp {
namespace irp6p_tfg {

robot::robot(lib::configurator &_config, lib::sr_ecp &_sr_ecp) :
	ecp::common::robot::ecp_robot(lib::irp6p_tfg::ROBOT_NAME, lib::irp6p_tfg::NUM_OF_SERVOS, _config, _sr_ecp)
{
	//  Stworzenie listy dostepnych kinematyk.
	create_kinematic_models_for_given_robot();
}

robot::robot(common::task::task_base& _ecp_object) :
	ecp::common::robot::ecp_robot(lib::irp6p_tfg::ROBOT_NAME, lib::irp6p_tfg::NUM_OF_SERVOS, _ecp_object)
{
	//  Stworzenie listy dostepnych kinematyk.
	create_kinematic_models_for_given_robot();
}

// Stworzenie modeli kinematyki dla robota IRp-6 na postumencie.
void robot::create_kinematic_models_for_given_robot(void)
{
	// Stworzenie wszystkich modeli kinematyki.
	add_kinematic_model(new kinematics::irp6p_tfg::model());
	// Ustawienie aktywnego modelu.
	set_kinematic_model(0);
}

} // namespace irp6p
} // namespace ecp
} // namespace mrrocpp
