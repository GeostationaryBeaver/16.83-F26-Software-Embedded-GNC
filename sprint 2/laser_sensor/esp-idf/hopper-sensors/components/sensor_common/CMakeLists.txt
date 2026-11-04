# ---------------------------------------------------------------------------
# sensor_common -- header-only component.
#
# SRCS is empty because there is nothing to compile; we only publish headers.
# INCLUDE_DIRS "include" is what makes #include "sensor_iface.h" work in any
# component that lists sensor_common under REQUIRES.
# ---------------------------------------------------------------------------
idf_component_register(
    INCLUDE_DIRS "include"
    REQUIRES esp_common
)
