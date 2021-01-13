set(franka_lwi_proto franka_lwi/franka_lwi_communication_protocol.h)

if(NOT EXISTS ${franka_lwi_proto})
    add_custom_command(OUTPUT ${franka_lwi_proto}
            COMMAND ../scripts/setup.sh
            WORKING_DIRECTORY ../scripts)

    add_custom_target(fetch_franka_lwi_proto ALL
            DEPENDS ${franka_lwi_proto})
endif()