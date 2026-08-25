/*
 * mpu6050.h - Driver MPU6050 viet tren API I2C MOI cua ESP-IDF.
 *
 * Vi sao tu viet thay vi dung component espressif/mpu6050 co san:
 *   Component do (v1.2.1, ban moi nhat) van dung driver/i2c.h - API nay da
 *   END-OF-LIFE tu ESP-IDF v6.0 va SE BI XOA o v7.0. Header cua no in ra
 *   "CRITICAL WARNING" khi bien dich. Driver nay dung driver/i2c_master.h.
 *
 * Vi khong co phan cung de kiem tra, driver co san cac cong cu tu chan doan:
 *   - mpu6050_bus_scan()      : quet bus, liet ke moi dia chi tra loi
 *   - mpu6050_who_am_i()      : doc ID chip, phan biet MPU6050 that voi clone
 *   - mpu6050_verify_config() : DOC LAI moi thanh ghi vua ghi va so sanh
 * Ba buoc nay bat duoc gan het cac loi noi day va loi cau hinh ngay o lan chay
 * dau, thay vi de chung bieu hien thanh "goc bi sai la" kho truy.
 */
#ifndef MPU6050_H
#define MPU6050_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/i2c_master.h"
#include "ahrs.h"   /* imu_sample_t */

/* --- Dia chi I2C: quyet dinh boi chan AD0 tren module --- */
#define MPU6050_ADDR_AD0_LOW   0x68   /* AD0 -> GND (mac dinh cua GY-521) */
#define MPU6050_ADDR_AD0_HIGH  0x69   /* AD0 -> VCC */

/* WHO_AM_I cua MPU6050 that. Cac chip khac tra ve gia tri khac:
 *   0x68 = MPU6050        0x70 = MPU6500 / MPU9250 (rat nhieu module ban ra la
 *   0x71 = MPU9250        loai nay du in nhan "MPU6050")
 *   0x73 = MPU9255        0x12 = ICM20948
 * Driver bao loi thay vi doan, nhung LOG lai gia tri doc duoc de de truy. */
#define MPU6050_WHO_AM_I_VAL   0x68

/* --- Thang do gia toc ke --- */
typedef enum {
    MPU6050_ACCEL_FS_2G  = 0,   /* 16384 LSB/g - do phan giai goc tot nhat */
    MPU6050_ACCEL_FS_4G  = 1,   /*  8192 LSB/g */
    MPU6050_ACCEL_FS_8G  = 2,   /*  4096 LSB/g */
    MPU6050_ACCEL_FS_16G = 3,   /*  2048 LSB/g */
} mpu6050_accel_fs_t;

/* --- Thang do con quay --- */
typedef enum {
    MPU6050_GYRO_FS_250DPS  = 0,   /* 131.0 LSB/(deg/s) */
    MPU6050_GYRO_FS_500DPS  = 1,   /*  65.5 LSB/(deg/s) - MAC DINH cua du an */
    MPU6050_GYRO_FS_1000DPS = 2,   /*  32.8 LSB/(deg/s) */
    MPU6050_GYRO_FS_2000DPS = 3,   /*  16.4 LSB/(deg/s) */
} mpu6050_gyro_fs_t;

/* --- Bo loc thong thap SO tren chip (thanh ghi CONFIG, truong DLPF_CFG) ---
 * Day la lop chong rung DAU TIEN: no cat nhieu rung ngay tren chip, truoc khi
 * du lieu ra khoi cam bien. Danh doi la do tre nhom.
 *
 *   DLPF   Bang thong gia toc ke   Tre     Bang thong con quay   Tre
 *   ----   ---------------------   -----   -------------------   -----
 *   0      260 Hz                  0.0ms   256 Hz               0.98ms
 *   1      184 Hz                  2.0ms   188 Hz               1.9ms
 *   2       94 Hz                  3.0ms    98 Hz               2.8ms
 *   3       44 Hz                  4.9ms    42 Hz               4.8ms   <- mac dinh
 *   4       21 Hz                  8.5ms    20 Hz               8.3ms
 *   5       10 Hz                 13.8ms    10 Hz              13.4ms
 *   6        5 Hz                 19.0ms     5 Hz              18.6ms
 *
 * Chon 3: cat gan het rung tay (8-15 Hz thi khong, nhung cat duoc thanh phan
 * tan cao) ma tre 4.9 ms van du nho de bo loc Kalman @200 Hz khong bi anh huong.
 *
 * LUU Y QUAN TRONG: khi DLPF_CFG khac 0, tan so ra goc cua con quay la 1 kHz.
 * Khi DLPF_CFG = 0, no la 8 kHz. SMPLRT_DIV tinh tren con so do. */
typedef enum {
    MPU6050_DLPF_260HZ = 0,
    MPU6050_DLPF_184HZ = 1,
    MPU6050_DLPF_94HZ  = 2,
    MPU6050_DLPF_44HZ  = 3,
    MPU6050_DLPF_21HZ  = 4,
    MPU6050_DLPF_10HZ  = 5,
    MPU6050_DLPF_5HZ   = 6,
} mpu6050_dlpf_t;

typedef struct {
    /* --- Chan noi day --- */
    i2c_port_num_t i2c_port;        /* -1 = tu chon cong con trong */
    gpio_num_t     sda_io;          /* ESP32 DevKit V1: GPIO21 */
    gpio_num_t     scl_io;          /* ESP32 DevKit V1: GPIO22 */
    uint32_t       scl_speed_hz;    /* 400000 (fast mode). MPU6050 chiu toi 400k */
    bool           internal_pullup; /* Module GY-521 da co pull-up 4.7k onboard,
                                     * nhung bat them khong hai va cuu duoc
                                     * truong hop dung MPU6050 tran */
    uint16_t       dev_addr;        /* MPU6050_ADDR_AD0_LOW / _HIGH */

    /* --- Cau hinh cam bien --- */
    mpu6050_accel_fs_t accel_fs;
    mpu6050_gyro_fs_t  gyro_fs;
    mpu6050_dlpf_t     dlpf;
    uint8_t            smplrt_div;  /* fs = 1000/(1+div) khi dlpf != 0.
                                     * 4 -> 200 Hz */
} mpu6050_config_t;

/* Dien cau hinh mac dinh cho ESP32 DevKit V1 + module GY-521:
 * GPIO21/22, 400 kHz, addr 0x68, +-2g, +-500 deg/s, DLPF 44 Hz, 200 Hz. */
void mpu6050_config_default(mpu6050_config_t *cfg);

typedef struct mpu6050_dev_t *mpu6050_handle_t;

/* So doc tho, chua doi don vi. */
typedef struct {
    int16_t ax, ay, az;
    int16_t temp;
    int16_t gx, gy, gz;
} mpu6050_raw_t;

/* --- Vong doi --- */

/* Tao bus I2C, gan thiet bi, reset chip va nap cau hinh.
 * Da bao gom kiem tra WHO_AM_I va doc lai toan bo thanh ghi de xac nhan. */
esp_err_t mpu6050_create(const mpu6050_config_t *cfg, mpu6050_handle_t *out);

/* Giai phong thiet bi va bus. An toan khi goi voi NULL. */
void mpu6050_delete(mpu6050_handle_t dev);

/* --- Chan doan --- */

/* Quet toan bo dia chi 7-bit hop le (0x08..0x77) va ghi log nhung dia chi tra
 * loi. Goi truoc mpu6050_create khi nghi ngo noi day. n_found co the NULL.
 * Can mot bus da tao - dung mpu6050_bus_scan_standalone neu chua co bus. */
esp_err_t mpu6050_bus_scan(i2c_master_bus_handle_t bus, int *n_found);

/* Tao bus tam, quet, roi huy. Tien cho buoc chan doan dau tien. */
esp_err_t mpu6050_bus_scan_standalone(const mpu6050_config_t *cfg, int *n_found);

/* Doc thanh ghi WHO_AM_I (0x75). */
esp_err_t mpu6050_who_am_i(mpu6050_handle_t dev, uint8_t *out_id);

/* Doc lai TAT CA thanh ghi cau hinh va so voi gia tri mong doi.
 * Bat duoc loi ghi that bai am tham - thu ma chi kiem tra ma tra ve cua ham ghi
 * khong phat hien duoc (vd chip reset lai giua duong do sut nguon). */
esp_err_t mpu6050_verify_config(mpu6050_handle_t dev);

/* In toan bo thanh ghi cau hinh ra log, dang doc duoc. */
esp_err_t mpu6050_dump_config(mpu6050_handle_t dev);

/* --- Doc du lieu --- */

/* Doc lien tuc 14 byte tu 0x3B. Quan trong: mot giao dich duy nhat, nen gia toc
 * ke va con quay den TU CUNG MOT THOI DIEM LAY MAU. Doc rieng hai lan se lam
 * hai nguon lech pha nhau, va bo loc Kalman se hop nhat hai thoi diem khac nhau
 * ma khong he biet. */
esp_err_t mpu6050_read_raw(mpu6050_handle_t dev, mpu6050_raw_t *out);

/* Doc va doi ngay ve don vi vat ly (g va deg/s) - dung nap thang vao ahrs. */
esp_err_t mpu6050_read(mpu6050_handle_t dev, imu_sample_t *out);

/* Nhiet do chip, do C. Huu ich vi bias con quay troi theo nhiet do - co the
 * dung de canh bao nguoi dung hieu chinh lai khi chip am len. */
esp_err_t mpu6050_read_temp(mpu6050_handle_t dev, float *out_celsius);

/* --- Truy van --- */
float mpu6050_accel_sensitivity(mpu6050_handle_t dev);  /* LSB/g */
float mpu6050_gyro_sensitivity(mpu6050_handle_t dev);   /* LSB/(deg/s) */
float mpu6050_sample_rate_hz(mpu6050_handle_t dev);

/* Truy cap bus de dung cho muc dich khac (vd quet lai bus). */
i2c_master_bus_handle_t mpu6050_bus_handle(mpu6050_handle_t dev);

#endif /* MPU6050_H */
