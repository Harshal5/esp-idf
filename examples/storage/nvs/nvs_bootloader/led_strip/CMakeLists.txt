include($ENV{IDF_PATH}/tools/cmake/version.cmake)

set(srcs "src/led_strip_api.c")
set(public_requires)

if(CONFIG_SOC_GPSPI_SUPPORTED)
    list(APPEND srcs "src/led_strip_spi_dev.c")
endif()

list(APPEND public_requires "esp_driver_spi")
idf_component_register(SRCS ${srcs}
                       INCLUDE_DIRS "include" "interface"
                       REQUIRES ${public_requires})
