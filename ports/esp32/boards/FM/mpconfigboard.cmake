set(SDKCONFIG_DEFAULTS
    boards/sdkconfig.base
#   boards/sdkconfig.ble
#   boards/sdkconfig.spiram
    boards/FM/sdkconfig.mypartitions
    boards/FM/sdkconfig.fm
)

set(USER_C_MODULES ${MICROPY_PORT_DIR}/ili9342c_mpy/src/micropython.cmake)

if(NOT MICROPY_FROZEN_MANIFEST)
    set(MICROPY_FROZEN_MANIFEST ${MICROPY_PORT_DIR}/boards/FM/manifest-FM.py)
endif()
