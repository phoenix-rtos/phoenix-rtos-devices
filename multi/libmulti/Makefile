NAME := libmulti

LOCAL_SRCS := libmulti.c libmulti-generic.c

ifdef LIBMULTI_HAS_GPIO_DRV
LOCAL_SRCS += libmulti-gpio.c
endif

ifdef LIBMULTI_HAS_I2C_DRV
LOCAL_SRCS += libmulti-i2c.c
endif

ifdef LIBMULTI_HAS_SPI_DRV
LOCAL_SRCS += libmulti-spi.c
endif

LOCAL_HEADERS := $(include/*.h)

include $(static-lib.mk)