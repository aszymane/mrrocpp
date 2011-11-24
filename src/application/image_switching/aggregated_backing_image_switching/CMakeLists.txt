add_executable(aggregated_backing_image_switching
	ecp_t_image_switching.cc
	aggregated_backing_image_switching.cc
)

target_link_libraries(aggregated_backing_image_switching
	visual_servoing
	ecp_r_irp6p_m
	ecp ${COMMON_LIBRARIES}
)

install(TARGETS aggregated_backing_image_switching DESTINATION bin)
