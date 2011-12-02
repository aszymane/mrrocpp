add_executable(aggregated_image_switching
	ecp_t_image_switching.cc
	aggregated_image_switching
)

target_link_libraries(aggregated_image_switching
	visual_servoing
	ecp_r_irp6p_m
	ecp ${COMMON_LIBRARIES}
)

install(TARGETS aggregated_image_switching DESTINATION bin)
