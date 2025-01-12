set(srcs "ADE7753.c"
        )

idf_component_register(
        SRCS ${srcs}
        INCLUDE_DIRS "."
        REQUIRES 
                "driver"        # Để sử dụng GPIO và SPI master
                "esp_common"    # Để sử dụng ESP log
                "freertos"      # Để sử dụng FreeRTOS API
        )