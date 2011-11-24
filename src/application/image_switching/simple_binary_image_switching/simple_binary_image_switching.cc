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

	fboth=fopen("/home/aszymane/workspace/mrrocpp/build/bin/zzboth.txt","w");
	if(fboth!=NULL)
		fclose(fboth);
	else
		printf("aggregated_image_switching: Failed to open file\n");
}

simple_binary_image_switching::~simple_binary_image_switching()
{
}

lib::Homog_matrix simple_binary_image_switching::get_aggregated_position_change()
{
	lib::Homog_matrix position_change_eih = servos[0]->get_position_change(get_current_position(), get_dt());
	lib::Homog_matrix position_change_sac = servos[1]->get_position_change(get_current_position(), get_dt());

	int visible_eih=servos[0]->is_object_visible();
	int visible_sac=servos[1]->is_object_visible();

	// sprawdzic warunek zmiany stanu
	if(visible_eih==1)
		state=0;
	else
		state=1;

	Eigen::Matrix<double, 6, 1> error_eih;
	Eigen::Matrix<double, 6, 1> error_sac;
	Eigen::Matrix<double, 6, 1> agg_error;

	if(visible_eih == 1)
		error_eih = servos[0]->get_error();
	if(visible_sac == 1)
		error_sac = servos[1]->get_error();

	lib::Homog_matrix position_change;
	if( state == 0 )
	{
		agg_error(0,0)=error_eih(0,0);
		agg_error(1,0)=error_eih(1,0);
		agg_error(2,0)=error_eih(2,0);
	}
	else
	{
		agg_error(0,0)=error_sac(0,0);
		agg_error(1,0)=error_sac(1,0);
		agg_error(2,0)=error_sac(2,0);
	}
	fboth=fopen("/home/aszymane/workspace/mrrocpp/build/bin/zzboth.txt","a");

	if(fboth==NULL)
		printf("aggregated_image_switching: Failed to open file\n");
	else{
		fprintf(fboth,"%g\t%g\t%g\t%g\t%g\t%g\t%g\t%g\t%g\n",
				error_sac(0,0),error_sac(1,0),error_sac(2,0),
				error_eih(0,0),error_eih(1,0),error_eih(2,0),
				agg_error(0,0),agg_error(1,0),agg_error(2,0));
		fflush(fboth);

		fclose(fboth);
	}

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
