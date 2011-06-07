/*!
 * @file
 * @brief File contains ecp base task definition
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup ecp
 */

#include <cstring>
#include <unistd.h>
#include <cerrno>
#include <cctype>
#include <cstdio>

#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>

#include "base/lib/configurator.h"
#include "base/lib/sr/sr_ecp.h"
#include "base/ecp/ecp_task.h"
#include "base/ecp/ecp_sub_task.h"
#include "base/ecp/ecp_robot.h"
#include "base/ecp/ECP_main_error.h"
#include "base/ecp/ECP_error.h"
#include "base/ecp/ecp_generator.h"

#include "base/lib/messip/messip_dataport.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {

task_base::task_base(lib::configurator &_config) :
	ecp_mp::task::task(_config), MP(lib::MP_SECTION), reply(MP, _config.section_name), command("command"),
			mp_command(command.access), continuous_coordination(false)
{
	initialize_communication();
}

void task_base::main_task_algorithm(void)
{
	for (;;) {
		sr_ecp_msg->message("Waiting for MP order");

		get_next_state();

		sr_ecp_msg->message("Order received");

		subtasks_conditional_execution();

		mp_2_ecp_next_state_string_handler();

		termination_notice();
	} //end for
}

void task_base::mp_2_ecp_next_state_string_handler(void)
{
}

void task_base::ecp_stop_accepted_handler(void)
{
}

task_base::~task_base()
{
	// TODO: error check
	messip::port_delete(trigger_attach);
}

bool task_base::pulse_check()
{
	int32_t type, subtype;
	int recvid;
	if ((recvid = messip::port_receive_pulse(trigger_attach, type, subtype, 0)) == -1) {
		perror("messip::port_receive()");
		return false;
	}

	if (recvid == MESSIP_MSG_NOREPLY && type == ECP_TRIGGER) {
		return true;
	}

	return false;
}

// ---------------------------------------------------------------
void task_base::initialize_communication()
{
	const std::string ecp_attach_point = config.get_ecp_attach_point();

	const std::string sr_net_attach_point = config.get_sr_attach_point();

	// Obiekt do komuniacji z SR
	sr_ecp_msg = (boost::shared_ptr <lib::sr_ecp>) new lib::sr_ecp(lib::ECP, ecp_attach_point, sr_net_attach_point);

	//	std::cout << "ecp: Opening MP pulses channel at '" << mp_pulse_attach_point << "'" << std::endl;

	const std::string trigger_attach_point = config.get_ecp_trigger_attach_point();

	if ((trigger_attach = messip::port_create(trigger_attach_point)) == NULL) {
		int e = errno; // kod bledu systemowego
		perror("Failed to attach TRIGGER pulse chanel for ecp");
		sr_ecp_msg->message(lib::SYSTEM_ERROR, e, "Failed  Failed to name attach (trigger pulse)");
		throw ECP_main_error(lib::SYSTEM_ERROR, 0);
	}

	registerBuffer(command);
}
// -------------------------------------------------------------------

// Badanie typu polecenia z MP
lib::MP_COMMAND task_base::mp_command_type(void) const
{
	return mp_command.command;
}

// Ustawienie typu odpowiedzi z ECP do MP
void task_base::set_ecp_reply(lib::ECP_REPLY ecp_r)
{
	ecp_reply.reply = ecp_r;
}

// Informacja dla MP o zakonczeniu zadania uzytkownika
void task_base::termination_notice(void)
{
	if (mp_command_type() != lib::END_MOTION) {

		set_ecp_reply(lib::TASK_TERMINATED);
		reply.Send(ecp_reply);
	}
}

void task_base::subtasks_conditional_execution()
{
	BOOST_FOREACH(const subtask_pair_t & subtask_node, subtask_m)
				{
					if (mp_2_ecp_next_state_string == subtask_node.first) {
						subtask_node.second->conditional_execution();
					}
				}
}

// Petla odbierania wiadomosci.
void task_base::wait_for_stop(void)
{
	while (command.Get().command != lib::STOP) {
		ReceiveSingleMessage(true);
	}
}

// Oczekiwanie na polecenie START od MP
void task_base::wait_for_start(void)
{
	// Awaiting for the START command
	bool start_received = false;

	while (!start_received) {
		while (!command.isFresh()) {
			ReceiveSingleMessage(true);
		}

		command.markAsUsed();

		switch (mp_command.command)
		{
			case lib::START_TASK:
				// by Y - ECP_ACKNOWLEDGE zamienione na lib::TASK_TERMINATED w celu uproszczenia oprogramowania zadan wielorobotowych
				set_ecp_reply(lib::ECP_ACKNOWLEDGE);

				// Reply with ACK
				reply.Send(ecp_reply);

				// OK, ready to start!
				start_received = true;

				break;
			case lib::STOP:
				set_ecp_reply(lib::ECP_ACKNOWLEDGE);
				// Reply with ACK
				reply.Send(ecp_reply);
				throw common::generator::ECP_error(lib::NON_FATAL_ERROR, ECP_STOP_ACCEPTED);
				break;
			default:
				set_ecp_reply(lib::INCORRECT_MP_COMMAND);
				// Reply with NACK
				reply.Send(ecp_reply);
				throw common::generator::ECP_error(lib::NON_FATAL_ERROR, INVALID_MP_COMMAND);
				break;
		}
	}

	sr_ecp_msg->message("ecp user program is running");
}

// Oczekiwanie na kolejne zlecenie od MP
void task_base::get_next_state(void)
{
	sr_ecp_msg->message(lib::NON_FATAL_ERROR, "get_next_state poczatek");

	bool next_state_received = false;

	while (!next_state_received) {
		while (!command.isFresh()) {
			sr_ecp_msg->message(lib::NON_FATAL_ERROR, "get_next_state 1");

			ReceiveSingleMessage(true);
			sr_ecp_msg->message(lib::NON_FATAL_ERROR, "get_next_state 2");

		}
		sr_ecp_msg->message(lib::NON_FATAL_ERROR, "get_next_state 3");

		command.markAsUsed();

		switch (mp_command.command)
		{
			case lib::NEXT_STATE:
				set_ecp_reply(lib::ECP_ACKNOWLEDGE);
				sr_ecp_msg->message(lib::NON_FATAL_ERROR, "get_next_state lib::NEXT_STATE");

				// Reply with ACK
				reply.Send(ecp_reply);
				next_state_received = true;
				break;
			case lib::PAUSE_TASK:
				//	set_ecp_reply(lib::ECP_ACKNOWLEDGE);
				sr_ecp_msg->message(lib::NON_FATAL_ERROR, "get_next_state lib::PAUSE_TASK");

				// Reply with ACK
				//	reply.Send(ecp_reply);
				wait_for_resume();
				break;
			case lib::STOP:
				set_ecp_reply(lib::ECP_ACKNOWLEDGE);
				sr_ecp_msg->message(lib::NON_FATAL_ERROR, "get_next_state lib::STOP");

				// Reply with ACK
				reply.Send(ecp_reply);
				throw common::generator::ECP_error(lib::NON_FATAL_ERROR, ECP_STOP_ACCEPTED);
				break;
			default:
				set_ecp_reply(lib::INCORRECT_MP_COMMAND);
				sr_ecp_msg->message(lib::NON_FATAL_ERROR, "get_next_state lib::INCORRECT_MP_COMMAND");

				// Reply with NACK
				reply.Send(ecp_reply);
				throw common::generator::ECP_error(lib::NON_FATAL_ERROR, INVALID_MP_COMMAND);
				break;
		}
	}

	// Extract the next command to the local variable
	mp_2_ecp_next_state_string = mp_command.ecp_next_state.mp_2_ecp_next_state;
}

// Receive a message from MP
bool task_base::peek_mp_message()
{
	command.markAsUsed();
	bool block;

	if (continuous_coordination) {
		block = true;
	} else {
		block = false;
	}
	if (ReceiveSingleMessage(block)) {
		if (command.isFresh()) {

			switch (mp_command.command)
			{

				case lib::NEXT_POSE:

					if (continuous_coordination) {
						reply.Send(ecp_reply);
					} else {
						sr_ecp_msg->message(lib::NON_FATAL_ERROR, "STRANGE COMMAND (NEXT_POSE) for non continuous_coordination");
					}

					break;

				case lib::END_MOTION:
					command.markAsUsed();
					return true;

					break;

				case lib::STOP:
					set_ecp_reply(lib::ECP_ACKNOWLEDGE);
					sr_ecp_msg->message(lib::NON_FATAL_ERROR, "peek_mp_message lib::STOP");
					command.markAsUsed();
					// Reply with ACK
					reply.Send(ecp_reply);
					throw common::generator::ECP_error(lib::NON_FATAL_ERROR, ECP_STOP_ACCEPTED);
					break;
				case lib::PAUSE_TASK:
					//	set_ecp_reply(lib::ECP_ACKNOWLEDGE);
					sr_ecp_msg->message(lib::NON_FATAL_ERROR, "peek_mp_message lib::PAUSE_TASK");

					// Reply with ACK
					//reply.Send(ecp_reply);
					wait_for_resume();
					break;
				default:
					set_ecp_reply(lib::INCORRECT_MP_COMMAND);
					sr_ecp_msg->message(lib::NON_FATAL_ERROR, "peek_mp_message lib::INCORRECT_MP_COMMAND");
					command.markAsUsed();
					// Reply with NACK
					reply.Send(ecp_reply);
					throw common::generator::ECP_error(lib::NON_FATAL_ERROR, INVALID_MP_COMMAND);
					break;
			}
		}

	}

	return false;
}

void task_base::wait_for_resume()
{
	if (ReceiveSingleMessage(true)) {
		command.markAsUsed();
		switch (mp_command.command)
		{
			case lib::STOP:
				set_ecp_reply(lib::ECP_ACKNOWLEDGE);
				sr_ecp_msg->message(lib::NON_FATAL_ERROR, "wait_for_resume lib::STOP");

				// Reply with ACK
				reply.Send(ecp_reply);
				throw common::generator::ECP_error(lib::NON_FATAL_ERROR, ECP_STOP_ACCEPTED);
				break;
			case lib::RESUME_TASK:
				//	set_ecp_reply(lib::ECP_ACKNOWLEDGE);
				sr_ecp_msg->message(lib::NON_FATAL_ERROR, "wait_for_resume lib::RESUME_TASK");

				// Reply with ACK
				//	reply.Send(ecp_reply);
				break;
			default:
				set_ecp_reply(lib::INCORRECT_MP_COMMAND);
				sr_ecp_msg->message(lib::NON_FATAL_ERROR, "wait_for_resume lib::INCORRECT_MP_COMMAND");

				// Reply with NACK
				reply.Send(ecp_reply);
				throw common::generator::ECP_error(lib::NON_FATAL_ERROR, INVALID_MP_COMMAND);
				break;
		}

	}

}

} // namespace task
} // namespace common
} // namespace ecp
} // namespace mrrocpp
