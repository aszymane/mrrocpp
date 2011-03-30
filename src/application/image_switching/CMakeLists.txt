add_executable(image_switching
	ecp_t_image_switching.cc
#	ecp_g_image_switching.cc
	simple_binary_image_switching.cc
	aggregated_image_switching.cc
)

target_link_libraries(image_switching
	visual_servoing
	ecp_r_irp6p_m
	ecp ${COMMON_LIBRARIES}
)

install(TARGETS image_switching DESTINATION bin)
