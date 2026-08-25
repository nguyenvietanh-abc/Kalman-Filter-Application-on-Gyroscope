idf_component_register(
    SRCS
        "main.c"
        "mpu6050.c"
        "ahrs.c"
        "kalman.c"
        "imu_calib.c"
    INCLUDE_DIRS
        "."
    REQUIRES
        esp_driver_i2c      # driver/i2c_master.h - API MOI, khong dung driver/i2c.h (EOL)
        esp_driver_gpio     # nut BOOT de buoc hieu chinh lai
        esp_timer           # do dt thuc te
        nvs_flash           # luu ket qua hieu chinh giua cac lan khoi dong
)
