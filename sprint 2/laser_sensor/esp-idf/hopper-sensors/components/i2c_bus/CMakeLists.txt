# ---------------------------------------------------------------------------
# i2c_bus -- owns the single shared I2C master bus.
#
# REQUIRES names components whose headers we expose in OUR public header
# (i2c_bus.h includes driver/i2c_master.h, so "driver" must be public).
# PRIV_REQUIRES would be for things used only inside the .c file.
# ---------------------------------------------------------------------------
idf_component_register(
    SRCS "i2c_bus.c"
    INCLUDE_DIRS "include"
    REQUIRES driver
)
