/*!
 * \file edp_e_sbench.h
 * \brief File containing the declaration of edp::sbench::effector class.
 *
 * \author yoyek
 * \date 2009
 *
 */

#ifndef __EDP_E_SBENCH_H
#define __EDP_E_SBENCH_H

#include "base/edp/edp_e_motor_driven.h"
#include "dp_sbench.h"
#include <comedilib.h>

namespace mrrocpp {
namespace edp {
namespace sbench {

// Klasa reprezentujaca robota IRp-6 na postumencie.

/*!
 * \brief class of EDP SwarmItFix head effector
 *
 * This head is built on top of the SPKM manipulator
 */
class effector : public common::motor_driven_effector
{
protected:

	lib::sbench::cbuffer ecp_edp_cbuffer;
	lib::sbench::rbuffer edp_ecp_rbuffer;

	// Metoda tworzy modele kinematyczne dla robota IRp-6 na postumencie.
	/*!
	 * \brief method,  creates a list of available kinematic models for sbench effector.
	 *
	 * It will be used if any motor will be commanded to move. Then motor to joint transform will be implemented in kinematics.
	 */
	virtual void create_kinematic_models_for_given_robot(void);

	/*!
	 * \brief current pins state
	 *
	 * it is cipied from desired in test_mode or read in hardware_mode
	 */
	lib::sbench::pins_state_td current_pins_state;

public:

	/*!
	 * \brief class constructor
	 *
	 * The attributes are initialized here.
	 */
	effector(common::shell &_shell);

	/*!
	 * \brief method to create threads other then EDP master thread.
	 *
	 * Here there is only one extra thread - reader_thread.
	 */
	void create_threads();

	void get_controller_state(lib::c_buffer &instruction);

	/*!
	 * \brief method to set position of the motors or joints
	 *
	 * It will be used if there will be any motor used.
	 */
	void move_arm(const lib::c_buffer &instruction); // przemieszczenie ramienia

	/*!
	 * \brief method to get position of the motors or joints
	 *
	 * It will be used if there will be any motor used.
	 */

	void get_arm_position(bool read_hardware, lib::c_buffer &instruction); // odczytanie pozycji ramienia

	/*!
	 * \brief method to choose master_order variant
	 *
	 * IHere the single thread variant is chosen
	 */
	void master_order(common::MT_ORDER nm_task, int nm_tryb);

	/*!
	 * \brief method to deserialize part of the reply
	 *
	 * Currently simple memcpy implementation
	 */
	void instruction_deserialization();

	/*!
	 * \brief method to serialize part of the reply
	 *
	 * Currently simple memcpy implementation
	 */
	void reply_serialization();

private:
	const std::string dev_name;
	comedi_t *device; // device descriptor

};

} // namespace smb
} // namespace edp
} // namespace mrrocpp

#endif
