// -------------------------------------------------------------------------
//                                   edp.h
// Definicje struktur danych i metod dla procesu EDP
//
// Ostatnia modyfikacja: 16.04.98
// -------------------------------------------------------------------------

#ifndef __EDP_E_MANIP_H
#define __EDP_E_MANIP_H

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"
#include "lib/srlib.h"

#if defined(USE_MESSIP_SRR)
#include <messip.h>
#endif

#include "kinematics/common/kinematics_manager.h"
#include "edp/common/edp_e_manip_and_conv.h"

// Konfigurator
#include "lib/configurator.h"

namespace mrrocpp {
namespace edp {
namespace common {

// base class for EDP robots with manipulators
class manip_effector: public common::manip_and_conv_effector
{

protected:

	virtual void compute_frame(const lib::c_buffer &instruction); // obliczenia dla ruchu ramienia (koncowka: FRAME)

	void set_tool_frame_in_kinematic_model(const lib::Homog_matrix& hm);

	// lib::r_buffer

	lib::Homog_matrix servo_current_frame_wo_tool; // by Y dla watku EDP_SERVO    XXXXX
	lib::Homog_matrix global_current_frame_wo_tool;// globalne dla procesu EDP    XXXXXX

	double servo_real_kartez_pos[6]; // by Y polozenie we wspolrzednych xyz_euler_zyz obliczane co krok servo   XXXXX
	double servo_real_kartez_vel[6]; // by Y predkosc we wspolrzednych xyz_euler_zyz obliczane co krok servo   XXXXX
	double servo_real_kartez_acc[6]; // by Y predkosc we wspolrzednych xyz_euler_zyz obliczane co krok servo   XXXXX

	lib::Homog_matrix desired_end_effector_frame; //  XXXXX
	// Podstawowa postac reprezentujaca zadane
	// wspolrzedne zewnetrzne koncowki manipulatora
	// wzgledem ukladu bazowego (polozenie w mm)

	lib::Homog_matrix current_end_effector_frame;
	// Podstawowa postac reprezentujaca ostatnio
	// odczytane wspolrzedne zewnetrzne koncowki
	// manipulatora wzgledem ukladu bazowego (polozenie w mm)

public:
	manip_effector(lib::configurator &_config, lib::robot_name_t l_robot_name); // konstruktor

	void synchronise(); // synchronizacja robota
	void get_controller_state(lib::c_buffer &instruction); // synchronizacja robota

	virtual void create_threads();

	// wyznaczenie polozenia lokalnego i globalnego transformera
	// przepisanie lokalnego zestawu lokalnego edp_servo na globalny (chronione mutexem)
	void master_joints_and_frame_download(void);// by Y przepisanie z zestawu globalnego na lokalny dla edp_master

	virtual void master_order(MT_ORDER nm_task, int nm_tryb);
};

} // namespace common
} // namespace edp
} // namespace mrrocpp

#endif
