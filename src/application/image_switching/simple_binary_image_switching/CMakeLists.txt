add_executable(simple_binary_image_switching
	ecp_t_image_switching.cc
	simple_binary_image_switching.cc
)

target_link_libraries(simple_binary_image_switching
	visual_servoing
	ecp_r_irp6p_m
	ecp ${COMMON_LIBRARIES}
)

install(TARGETS simple_binary_image_switching DESTINATION bin)
