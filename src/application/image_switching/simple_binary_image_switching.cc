/*
 * simple_binary_image_switching.cc
 *
 *  Created on: Oct 13, 2010
 *      Author: aszymane
 */

#include "simple_binary_image_switching.h"

#include "base/lib/logger.h"

namespace mrrocpp {

namespace ecp {

namespace common {

namespace generator {

simple_binary_image_switching::simple_binary_image_switching(mrrocpp::ecp::common::task::task & ecp_task, const char * section_name1, boost::shared_ptr <
		mrrocpp::ecp::servovision::visual_servo> eih, const char * section_name2, boost::shared_ptr <
		mrrocpp::ecp::servovision::visual_servo> sac) :
	visual_servo_manager(ecp_task, section_name1)
{
	servos.push_back(eih);
	servos.push_back(sac);
	state=0;
	fsac=fopen("/home/aszymane/workspace/mrrocpp-git/build/bin/sac.txt","w");
	feih=fopen("/home/aszymane/workspace/mrrocpp-git/build/bin/eih.txt","w");
}

simple_binary_image_switching::~simple_binary_image_switching()
{
	fclose(fsac);
	fclose(feih);
}

lib::Homog_matrix simple_binary_image_switching::get_aggregated_position_change()
{
	lib::Homog_matrix position_change_eih = servos[0]->get_position_change(get_current_position(), dt);
	lib::Homog_matrix position_change_sac = servos[1]->get_position_change(get_current_position(), dt);

	int visible_eih=servos[0]->is_object_visible();
	int visible_sac=servos[1]->is_object_visible();

	fprintf(fsac,"%g\t%g\t%g\t%g\n",position_change_sac(0,0),position_change_sac(0,1),position_change_sac(0,2),position_change_sac(0,3));
	fprintf(fsac,"%g\t%g\t%g\t%g\n",position_change_sac(1,0),position_change_sac(1,1),position_change_sac(1,2),position_change_sac(1,3));
	fprintf(fsac,"%g\t%g\t%g\t%g\n",position_change_sac(2,0),position_change_sac(2,1),position_change_sac(2,2),position_change_sac(2,3));
	fprintf(fsac,"%g\t%g\t%g\t%g\n",0.0,0.0,0.0,1.0);
	fflush(fsac);

	fprintf(feih,"%g\t%g\t%g\t%g\n",position_change_eih(0,0),position_change_eih(0,1),position_change_eih(0,2),position_change_eih(0,3));
	fprintf(feih,"%g\t%g\t%g\t%g\n",position_change_eih(1,0),position_change_eih(1,1),position_change_eih(1,2),position_change_eih(1,3));
	fprintf(feih,"%g\t%g\t%g\t%g\n",position_change_eih(2,0),position_change_eih(2,1),position_change_eih(2,2),position_change_eih(2,3));
	fprintf(feih,"%g\t%g\t%g\t%g\n",0.0,0.0,0.0,1.0);
	fflush(feih);

//	printf("czy widzi 0: %d\n", visible_eih);
//	printf("czy widzi 1: %d\n", visible_sac);

	// sprawdzic warunek zmiany stanu
	if(visible_eih==1)
		state=0;
	else
		state=1;

	fflush(stdout);

	if(state == 0){
		printf("Z 000000000000000000\n"); fflush(stdout);
		return position_change_eih;
	}
	if(state == 1){
		printf("Z 111111111111111111\n"); fflush(stdout);
		return position_change_sac;
	}
	return lib::Homog_matrix();
}

void simple_binary_image_switching::configure_all_servos()
{
	//logger::logDbg("simple_binary_image_switching::configure_all_servos()\n");
}



}//namespace generator

}//namespace common

}//namespace ecp

}//namespace mrrocpp
