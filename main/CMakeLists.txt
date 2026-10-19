idf_component_register(
    SRCS "main.c" "sdr.c" "dsp.c" "audio.c" "ui.c" "shot.c"
    INCLUDE_DIRS "."
    REQUIRES bsp librtlsdr usb esp_timer esp_driver_ppa espressif__esp-dsp
             esp_driver_usb_serial_jtag)
