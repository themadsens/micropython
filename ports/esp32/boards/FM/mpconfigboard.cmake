set(SDKCONFIG_DEFAULTS
    boards/sdkconfig.base
    boards/sdkconfig.ble
    boards/sdkconfig.spiram
    boards/sdkconfig.mypartitions
)

set(USER_C_MODULES ${MICROPY_PORT_DIR}/ili9342c_mpy/src/micropython.cmake)

if(NOT MICROPY_FROZEN_MANIFEST)
    set(MICROPY_FROZEN_MANIFEST ${MICROPY_PORT_DIR}/boards/manifest-FM.py)
endif()
