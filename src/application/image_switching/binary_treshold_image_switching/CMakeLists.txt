add_executable(binary_treshold_image_switching
	ecp_t_image_switching.cc
	binary_treshold_image_switching.cc
)

target_link_libraries(binary_treshold_image_switching
	visual_servoing
	ecp_r_irp6p_m
	ecp ${COMMON_LIBRARIES}
)

install(TARGETS binary_treshold_image_switching DESTINATION bin)
