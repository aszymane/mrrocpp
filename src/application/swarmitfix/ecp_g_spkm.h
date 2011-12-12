/*
 * generator/ecp_g_epos.h
 *
 *Author: yoyek
 */

#ifndef ECP_G_SPKM_H_
#define ECP_G_SPKM_H_

#include "robot/spkm/ecp_r_spkm.h"
#include "robot/spkm/dp_spkm.h"

#include "base/ecp/ecp_generator.h"

namespace mrrocpp {
namespace ecp {
namespace spkm {
namespace generator {

class spkm_pose : public common::generator::_generator<ecp::spkm::robot>
{
public:
	//! Constructor
	spkm_pose(task_t & _ecp_task, const lib::spkm::next_state_t::segment_sequence_t & _segments);

	//! first step generation
	bool first_step();

	//! next step generation
	bool next_step();

private:
	//! Motion segment iterator
	lib::spkm::next_state_t::segment_sequence_t::const_iterator segment_iterator;

	//! Request execution of a single motion segment
	void request_segment_execution(robot_t & robot, const lib::spkm::segment_t & segment);

	//! Motion segments
	const lib::spkm::next_state_t::segment_sequence_t & segments;
};

class spkm_quickstop : public common::generator::_generator<ecp::spkm::robot>
{
public:
	//! Constructor
	spkm_quickstop(task_t & _ecp_task);

	//! first step generation
	bool first_step();

	//! next step generation
	bool next_step();
};

} // namespace generator
} // namespace spkm
} // namespace ecp
} // namespace mrrocpp

#endif
