add_executable(image_switching
	ecp_t_image_switching.cc
	ecp_g_image_switching.cc
)

target_link_libraries(image_switching 
	ecp_r_irp6ot
	ecp ${COMMON_LIBRARIES}
)

install(TARGETS image_switching DESTINATION bin)