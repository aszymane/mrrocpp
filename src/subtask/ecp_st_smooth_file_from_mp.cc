#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"

#include "base/lib/sr/sr_ecp.h"
#include "ecp_st_smooth_file_from_mp.h"
#include "generator/ecp/ecp_g_newsmooth.h"

#include "robot/irp6ot_m/const_irp6ot_m.h"
#include "robot/irp6p_m/const_irp6p_m.h"

#include "base/ecp/ecp_task.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace sub_task {

sub_task_smooth_file_from_mp::sub_task_smooth_file_from_mp(task::task & _ecp_t, lib::ECP_POSE_SPECIFICATION pose_spec, bool _detect_jerks) :
	sub_task(_ecp_t), detect_jerks(_detect_jerks)
{
	switch (pose_spec)
	{
		case lib::ECP_JOINT:
			sgen = new generator::newsmooth(ecp_t, pose_spec, ecp_t.ecp_m_robot->number_of_servos);
			break;
		case lib::ECP_XYZ_ANGLE_AXIS:
			sgen = new generator::newsmooth(ecp_t, pose_spec, 6);
			sgen->set_debug(true);
			break;
		default:
			break;
	}
}

void sub_task_smooth_file_from_mp::conditional_execution()
{
	sgen->reset();
	path = ecp_t.mp_command.ecp_next_state.get_mp_2_ecp_next_state_string();
	sgen->load_trajectory_from_file(path.c_str());

	if (detect_jerks) {
		if (sgen->calculate_interpolate() && sgen->detect_jerks(1) == 0) {
			sgen->Move();
		}
	} else {
		if (sgen->calculate_interpolate()) {
			sgen->Move();
		}
	}
}

sub_task_smooth_file_from_mp::~sub_task_smooth_file_from_mp()
{
	delete sgen;
}

} // namespace task
} // namespace common
} // namespace ecp
} // namespace mrrocpp
