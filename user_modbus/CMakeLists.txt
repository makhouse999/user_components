idf_component_register(SRCS "user_modbus_master.c" "user_modbus_slave.c"
                    INCLUDE_DIRS "include"
					REQUIRES main freertos freemodbus)
