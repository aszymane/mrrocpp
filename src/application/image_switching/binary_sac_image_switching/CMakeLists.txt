add_executable(binary_sac_image_switching
	ecp_t_image_switching.cc
	binary_sac_image_switching.cc
)

target_link_libraries(binary_sac_image_switching
	visual_servoing
	ecp_r_irp6p_m
	ecp ${COMMON_LIBRARIES}
)

install(TARGETS binary_sac_image_switching DESTINATION bin)
