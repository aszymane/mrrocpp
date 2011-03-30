add_executable(image_switching
	ecp_t_image_switching.cc
#	ecp_g_image_switching.cc
	simple_binary_image_switching.cc
	aggregated_image_switching.cc
	../visual_servoing/visual_servo_manager.cc
    ../visual_servoing/single_visual_servo_manager.cc
	../visual_servoing/ib_eih_visual_servo.cc
    ../visual_servoing/pb_eih_visual_servo.cc
    ../visual_servoing/pb_sac_visual_servo.cc
    ../visual_servoing/pb_visual_servo.cc
    ../visual_servoing/visual_servo.cc
    ../visual_servoing/position_constraint.cc
    ../visual_servoing/cubic_constraint.cc
    ../visual_servoing/object_reached_termination_condition.cc
    ../visual_servoing/timeout_termination_condition.cc
    ../visual_servoing/termination_condition.cc
)

target_link_libraries(image_switching 
	ecp_r_irp6p_m
	ecp ${COMMON_LIBRARIES}
)

install(TARGETS image_switching DESTINATION bin)