add_executable(ternary_changing_image_switching
	ecp_t_image_switching.cc
	ternary_changing_image_switching.cc
)

target_link_libraries(ternary_changing_image_switching
	visual_servoing
	ecp_r_irp6p_m
	ecp ${COMMON_LIBRARIES}
)

install(TARGETS ternary_changing_image_switching DESTINATION bin)
