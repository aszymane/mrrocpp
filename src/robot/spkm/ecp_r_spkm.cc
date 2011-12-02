/*!
 * @file
 * @brief File contains ecp robot class definition for SwarmItFix Parallel Kinematic Machine
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup spkm
 */

#include "base/lib/impconst.h"

#include "ecp_r_spkm.h"
#include "base/lib/sr/sr_ecp.h"
#include "kinematic_model_spkm.h"

namespace mrrocpp {
namespace ecp {
namespace spkm {

robot::robot(const lib::robot_name_t & _robot_name, lib::configurator &_config, lib::sr_ecp &_sr_ecp) :
		ecp::common::robot::ecp_robot(_robot_name, lib::spkm::NUM_OF_SERVOS, _config, _sr_ecp),
		epos_motor_command_data_port(lib::epos::EPOS_MOTOR_COMMAND_DATA_PORT, port_manager), epos_joint_command_data_port(lib::epos::EPOS_JOINT_COMMAND_DATA_PORT, port_manager), epos_external_command_data_port(lib::epos::EPOS_EXTERNAL_COMMAND_DATA_PORT, port_manager), epos_brake_command_data_port(lib::epos::EPOS_BRAKE_COMMAND_DATA_PORT, port_manager), epos_clear_fault_data_port(lib::epos::EPOS_CLEAR_FAULT_DATA_PORT, port_manager), epos_motor_reply_data_request_port(lib::epos::EPOS_MOTOR_REPLY_DATA_REQUEST_PORT, port_manager), epos_joint_reply_data_request_port(lib::epos::EPOS_JOINT_REPLY_DATA_REQUEST_PORT, port_manager), epos_external_reply_data_request_port(lib::epos::EPOS_EXTERNAL_REPLY_DATA_REQUEST_PORT, port_manager)
{
	// Stworzenie listy dostepnych kinematyk.
	create_kinematic_models_for_given_robot();
}

robot::robot(const lib::robot_name_t & _robot_name, common::task::task_base& _ecp_object) :
		ecp::common::robot::ecp_robot(_robot_name, lib::spkm::NUM_OF_SERVOS, _ecp_object), epos_motor_command_data_port(lib::epos::EPOS_MOTOR_COMMAND_DATA_PORT, port_manager), epos_joint_command_data_port(lib::epos::EPOS_JOINT_COMMAND_DATA_PORT, port_manager), epos_external_command_data_port(lib::epos::EPOS_EXTERNAL_COMMAND_DATA_PORT, port_manager), epos_brake_command_data_port(lib::epos::EPOS_BRAKE_COMMAND_DATA_PORT, port_manager), epos_clear_fault_data_port(lib::epos::EPOS_CLEAR_FAULT_DATA_PORT, port_manager), epos_motor_reply_data_request_port(lib::epos::EPOS_MOTOR_REPLY_DATA_REQUEST_PORT, port_manager), epos_joint_reply_data_request_port(lib::epos::EPOS_JOINT_REPLY_DATA_REQUEST_PORT, port_manager), epos_external_reply_data_request_port(lib::epos::EPOS_EXTERNAL_REPLY_DATA_REQUEST_PORT, port_manager)
{
	// Stworzenie listy dostepnych kinematyk.
	create_kinematic_models_for_given_robot();

	// Create bypass data buffer for communication with MP
	nextstateBuffer = (boost::shared_ptr<InputBuffer<lib::spkm::next_state_t> >) new InputBuffer<lib::spkm::next_state_t>(_ecp_object, lib::commandBufferId);
}

void robot::create_command()
{
	//	int new_data_counter;
	bool is_new_data;
	bool is_new_request;

	//sr_ecp_msg.message("create_command");

	is_new_data = false;
	is_new_request = false;

	// Set default variant to error in order to help tracking errors in communication
	// TODO: the following should be if-then-elseif-elseif-elseif...-else branch tree
	ecp_edp_cbuffer.variant = (lib::spkm::CBUFFER_VARIANT) -1;

	if (epos_motor_command_data_port.get() == mrrocpp::lib::NewData) {
		ecp_command.set_type = ARM_DEFINITION;
		if (!is_synchronised()) {
			ecp_command.motion_type = lib::RELATIVE;
			ecp_command.set_arm_type = lib::MOTOR;
		}

		ecp_edp_cbuffer.variant = lib::spkm::POSE;

		ecp_edp_cbuffer.set_pose_specification = lib::spkm::MOTOR;

		ecp_edp_cbuffer.motion_variant = epos_motor_command_data_port.data.motion_variant;
		ecp_edp_cbuffer.estimated_time = epos_motor_command_data_port.data.estimated_time;

		for (int i = 0; i < lib::spkm::NUM_OF_SERVOS; ++i) {
			ecp_edp_cbuffer.motor_pos[i] = epos_motor_command_data_port.data.desired_position[i];
		}

		check_then_set_command_flag(is_new_data);
	}

	if (epos_joint_command_data_port.get() == mrrocpp::lib::NewData) {
		ecp_command.set_type = ARM_DEFINITION;

		ecp_edp_cbuffer.variant = lib::spkm::POSE;

		ecp_edp_cbuffer.set_pose_specification = lib::spkm::JOINT;

		ecp_edp_cbuffer.motion_variant = epos_joint_command_data_port.data.motion_variant;
		ecp_edp_cbuffer.estimated_time = epos_joint_command_data_port.data.estimated_time;
		for (int i = 0; i < lib::spkm::NUM_OF_SERVOS; ++i) {
			ecp_edp_cbuffer.joint_pos[i] = epos_joint_command_data_port.data.desired_position[i];
		}

		check_then_set_command_flag(is_new_data);
	}

	if (epos_external_command_data_port.get() == mrrocpp::lib::NewData) {
		ecp_command.set_type = ARM_DEFINITION;

		ecp_edp_cbuffer.variant = lib::spkm::POSE;

		ecp_edp_cbuffer.set_pose_specification = lib::spkm::FRAME;

		ecp_edp_cbuffer.motion_variant = epos_external_command_data_port.data.motion_variant;
		ecp_edp_cbuffer.estimated_time = epos_external_command_data_port.data.estimated_time;

		for (int i = 0; i < 6; ++i) {
			ecp_edp_cbuffer.goal_pos[i] = epos_external_command_data_port.data.desired_position[i];
		}

		check_then_set_command_flag(is_new_data);
	}

	if (epos_brake_command_data_port.get() == mrrocpp::lib::NewData) {
		ecp_command.set_type = ARM_DEFINITION;
		// generator command interpretation
		// narazie proste przepisanie

		ecp_edp_cbuffer.variant = lib::spkm::QUICKSTOP;

		check_then_set_command_flag(is_new_data);
	}

	if (epos_clear_fault_data_port.get() == mrrocpp::lib::NewData) {
		ecp_command.set_type = ARM_DEFINITION;
		// generator command interpretation
		// narazie proste przepisanie
		if (!is_synchronised()) {
			ecp_command.motion_type = lib::RELATIVE;
			ecp_command.set_arm_type = lib::MOTOR;
		}
		ecp_edp_cbuffer.variant = lib::spkm::CLEAR_FAULT;

		check_then_set_command_flag(is_new_data);
	}

	if (epos_motor_reply_data_request_port.is_new_request()) {
		ecp_edp_cbuffer.get_pose_specification = lib::spkm::MOTOR;
		//	ecp_command.get_arm_type = lib::MOTOR;
		//sr_ecp_msg.message("epos_motor_reply_data_request_port");

		check_then_set_command_flag(is_new_request);
	}

	if (epos_joint_reply_data_request_port.is_new_request()) {
		ecp_edp_cbuffer.get_pose_specification = lib::spkm::JOINT;
		//ecp_command.get_arm_type = lib::JOINT;
		//	sr_ecp_msg.message("epos_joint_reply_data_request_port.is_new_request()");
		check_then_set_command_flag(is_new_request);
	}

	if (epos_external_reply_data_request_port.is_new_request()) {
		ecp_edp_cbuffer.get_pose_specification = lib::spkm::FRAME;
		//ecp_command.get_arm_type = lib::FRAME;
		//sr_ecp_msg.message("epos_external_reply_data_request_port.is_new_request()");
		check_then_set_command_flag(is_new_request);
	}

	communicate_with_edp = true;
	if (is_new_data && is_new_request) {
		ecp_command.instruction_type = lib::SET_GET;
		//sr_ecp_msg.message("is_new_data is_new_request");
	} else if (is_new_data) {
		ecp_command.instruction_type = lib::SET;
		//sr_ecp_msg.message("is_new_data");
	} else if (is_new_request) {
		ecp_command.instruction_type = lib::GET;
		//sr_ecp_msg.message("is_new_request");
	} else {
		communicate_with_edp = false;
	}

	if (is_new_request) {
		ecp_command.get_type = ARM_DEFINITION; // arm - ORYGINAL
	}

	// message serialization
	if (communicate_with_edp) {
		assert(sizeof(ecp_command.serialized_command) >= sizeof(ecp_edp_cbuffer));
		memcpy(ecp_command.serialized_command, &ecp_edp_cbuffer, sizeof(ecp_edp_cbuffer));
	}
}

void robot::get_reply()
{
	// message deserialization
	memcpy(&edp_ecp_rbuffer, reply_package.serialized_reply, sizeof(edp_ecp_rbuffer));

	if (epos_motor_reply_data_request_port.is_new_request()) {
		// generator reply generation
		for (int i = 0; i < lib::spkm::NUM_OF_SERVOS; i++) {
			epos_motor_reply_data_request_port.data.epos_controller[i].position =
					edp_ecp_rbuffer.epos_controller[i].position;
			epos_motor_reply_data_request_port.data.epos_controller[i].current =
					edp_ecp_rbuffer.epos_controller[i].current;
			epos_motor_reply_data_request_port.data.epos_controller[i].motion_in_progress =
					edp_ecp_rbuffer.epos_controller[i].motion_in_progress;
		}
		epos_motor_reply_data_request_port.data.contact = edp_ecp_rbuffer.contact;

		epos_motor_reply_data_request_port.set();
	}

	if (epos_joint_reply_data_request_port.is_new_request()) {
		// generator reply generation
		sr_ecp_msg.message("ECP get_reply epos_joint_reply_data_request_port");

		for (int i = 0; i < lib::spkm::NUM_OF_SERVOS; i++) {
			epos_joint_reply_data_request_port.data.epos_controller[i].position =
					edp_ecp_rbuffer.epos_controller[i].position;
			epos_joint_reply_data_request_port.data.epos_controller[i].current =
					edp_ecp_rbuffer.epos_controller[i].current;
			epos_joint_reply_data_request_port.data.epos_controller[i].motion_in_progress =
					edp_ecp_rbuffer.epos_controller[i].motion_in_progress;
		}
		epos_joint_reply_data_request_port.data.contact = edp_ecp_rbuffer.contact;

		epos_joint_reply_data_request_port.set();
	}

	if (epos_external_reply_data_request_port.is_new_request()) {
		sr_ecp_msg.message("ECP get_reply epos_external_reply_data_request_port");
		// generator reply generation
		for (int i = 0; i < lib::spkm::NUM_OF_SERVOS; i++) {
			epos_external_reply_data_request_port.data.epos_controller[i].position =
					edp_ecp_rbuffer.epos_controller[i].position;
			epos_external_reply_data_request_port.data.epos_controller[i].current =
					edp_ecp_rbuffer.epos_controller[i].current;
			epos_external_reply_data_request_port.data.epos_controller[i].motion_in_progress =
					edp_ecp_rbuffer.epos_controller[i].motion_in_progress;
		}
		epos_external_reply_data_request_port.data.contact = edp_ecp_rbuffer.contact;

		epos_external_reply_data_request_port.data.current_frame = edp_ecp_rbuffer.current_pose;

		epos_external_reply_data_request_port.set();
	}

}

// Stworzenie modeli kinematyki dla robota IRp-6 na postumencie.
void robot::create_kinematic_models_for_given_robot(void)
{
	// Stworzenie wszystkich modeli kinematyki.
	add_kinematic_model(new kinematics::spkm::kinematic_model_spkm());
	// Ustawienie aktywnego modelu.
	set_kinematic_model(0);
}

} // namespace spkm
} // namespace ecp
} // namespace mrrocpp

