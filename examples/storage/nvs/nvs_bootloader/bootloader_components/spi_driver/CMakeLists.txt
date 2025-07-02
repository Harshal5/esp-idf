idf_build_get_property(target IDF_TARGET)

if(${target} STREQUAL "linux")
    return() # This component is not supported by the POSIX/Linux simulator
endif()

set(srcs "")
set(public_include "include")

if(CONFIG_SOC_GPSPI_SUPPORTED)
    list(APPEND srcs
        "src/gpspi/spi_common.c"
        "src/gpspi/spi_master.c"
    )
endif()

idf_component_register(
    SRCS ${srcs}
    INCLUDE_DIRS ${public_include}
    REQUIRES hal soc
)
