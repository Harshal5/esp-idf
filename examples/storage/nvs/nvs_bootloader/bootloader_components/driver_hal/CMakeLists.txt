idf_build_get_property(target IDF_TARGET)
idf_build_get_property(esp_tee_build ESP_TEE_BUILD)

set(srcs "")
set(includes "include")

if(CONFIG_SOC_GPSPI_SUPPORTED)
    list(APPEND srcs
        "spi_hal.c"
        "spi_hal_iram.c"
    )
endif()

if(CONFIG_SOC_GPIO_PORT)
    list(APPEND srcs "gpio_hal.c")
endif()

if(CONFIG_SOC_CLK_TREE_SUPPORTED)
    list(APPEND srcs "clk_tree_hal.c")
endif()

idf_component_register(SRCS ${srcs}
                       INCLUDE_DIRS ${includes}
                       REQUIRES soc esp_rom hal)

if(CONFIG_HAL_DEFAULT_ASSERTION_LEVEL EQUAL 1)
    target_link_libraries(${COMPONENT_LIB} INTERFACE "-u abort")
elseif(CONFIG_HAL_DEFAULT_ASSERTION_LEVEL EQUAL 2)
    target_link_libraries(${COMPONENT_LIB} INTERFACE "-u __assert_func")
endif()
