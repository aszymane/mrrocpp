add_executable(image_switching
	ecp_t_image_switching.cc
#	ecp_g_image_switching.cc
	simple_binary_image_switching.cc
	../servovision/visual_servo_manager.cc
#    ../servovision/single_visual_servo_manager.cc
	../servovision/ib_eih_visual_servo.cc
    ../servovision/pb_eih_visual_servo.cc
    ../servovision/pb_sac_visual_servo.cc
    ../servovision/pb_visual_servo.cc
    ../servovision/visual_servo.cc
    ../servovision/position_constraint.cc
    ../servovision/cubic_constraint.cc
    ../servovision/object_reached_termination_condition.cc
    ../servovision/timeout_termination_condition.cc
    ../servovision/termination_condition.cc
)

target_link_libraries(image_switching 
	ecp_r_irp6ot_m
	ecp ${COMMON_LIBRARIES}
)

install(TARGETS image_switching DESTINATION bin)