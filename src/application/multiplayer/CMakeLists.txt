add_executable(mp_multiplayer
	mp_t_multiplayer.cc
)

target_link_libraries(mp_multiplayer mp)

add_executable(ecp_irp6ot_multiplayer
	ecp_t_multiplayer_irp6ot.cc
)

target_link_libraries(ecp_irp6ot_multiplayer
	ecp_robot
)

install(TARGETS mp_multiplayer ecp_irp6ot_multiplayer DESTINATION bin)
