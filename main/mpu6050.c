/*
 * mpu6050.c - Driver MPU6050 tren driver/i2c_master.h. Xem mpu6050.h.
 */
#include "mpu6050.h"

#include <string.h>
#include <stdlib.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "mpu6050";

/* --- Thanh ghi (theo MPU-6000/MPU-6050 Register Map, rev 4.2) --- */
#define REG_SMPLRT_DIV        0x19
#define REG_CONFIG            0x1A
#define REG_GYRO_CONFIG       0x1B
#define REG_ACCEL_CONFIG      0x1C
#define REG_FIFO_EN           0x23
#define REG_INT_PIN_CFG       0x37
#define REG_INT_ENABLE        0x38
#define REG_INT_STATUS        0x3A
#define REG_ACCEL_XOUT_H      0x3B
#define REG_SIGNAL_PATH_RESET 0x68
#define REG_USER_CTRL         0x6A
#define REG_PWR_MGMT_1        0x6B
#define REG_PWR_MGMT_2        0x6C
#define REG_WHO_AM_I          0x75

#define PWR1_DEVICE_RESET     0x80
#define PWR1_SLEEP            0x40
#define PWR1_CLKSEL_PLL_XGYRO 0x01

#define SIGPATH_RESET_ALL     0x07   /* gyro + accel + temp */

#define I2C_TIMEOUT_MS        100

struct mpu6050_dev_t {
    i2c_master_bus_handle_t bus;
    i2c_master_dev_handle_t dev;
    mpu6050_config_t        cfg;
    float                   accel_sens;   /* LSB/g */
    float                   gyro_sens;    /* LSB/(deg/s) */
    bool                    owns_bus;
};

/* ------------------------------------------------------------------------- */
/* Truy cap thanh ghi                                                         */
/* ------------------------------------------------------------------------- */

static esp_err_t reg_write(mpu6050_handle_t d, uint8_t reg, uint8_t val)
{
    const uint8_t buf[2] = { reg, val };
    const esp_err_t err = i2c_master_transmit(d->dev, buf, sizeof buf, I2C_TIMEOUT_MS);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Ghi thanh ghi 0x%02X = 0x%02X that bai: %s",
                 reg, val, esp_err_to_name(err));
    }
    return err;
}

static esp_err_t reg_read(mpu6050_handle_t d, uint8_t reg, uint8_t *buf, size_t len)
{
    /* transmit_receive giu nguyen giao dich: START - ghi dia chi thanh ghi -
     * RESTART - doc. Khong duoc tach thanh hai giao dich roi vi mot master khac
     * (hoac mot lan doc khac) co the chen vao giua. */
    const esp_err_t err = i2c_master_transmit_receive(d->dev, &reg, 1, buf, len,
                                                     I2C_TIMEOUT_MS);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Doc %u byte tu thanh ghi 0x%02X that bai: %s",
                 (unsigned)len, reg, esp_err_to_name(err));
    }
    return err;
}

static esp_err_t reg_read8(mpu6050_handle_t d, uint8_t reg, uint8_t *val)
{
    return reg_read(d, reg, val, 1);
}

/* ------------------------------------------------------------------------- */

void mpu6050_config_default(mpu6050_config_t *cfg)
{
    cfg->i2c_port        = I2C_NUM_0;
    cfg->sda_io          = GPIO_NUM_21;   /* ESP32 DevKit V1 */
    cfg->scl_io          = GPIO_NUM_22;
    cfg->scl_speed_hz    = 400000;
    cfg->internal_pullup = true;
    cfg->dev_addr        = MPU6050_ADDR_AD0_LOW;

    cfg->accel_fs = MPU6050_ACCEL_FS_2G;
    /* +-500 deg/s thay vi +-250: lat thanh cam tay bang tay de dang vuot
     * 250 deg/s, va mot khi con quay bi saturate thi bo loc mat hoan toan thong
     * tin ve doan do - loi khong the phuc hoi. Doi lay do phan giai giam mot
     * nua, van con 65.5 LSB/(deg/s), qua du so voi nhieu nen 0.4 deg/s. */
    cfg->gyro_fs = MPU6050_GYRO_FS_500DPS;
    cfg->dlpf    = MPU6050_DLPF_44HZ;
    cfg->smplrt_div = 4;                  /* 1000/(1+4) = 200 Hz */
}

static float accel_sens_of(mpu6050_accel_fs_t fs)
{
    switch (fs) {
    case MPU6050_ACCEL_FS_2G:  return 16384.0f;
    case MPU6050_ACCEL_FS_4G:  return 8192.0f;
    case MPU6050_ACCEL_FS_8G:  return 4096.0f;
    case MPU6050_ACCEL_FS_16G: return 2048.0f;
    default:                   return 16384.0f;
    }
}

static float gyro_sens_of(mpu6050_gyro_fs_t fs)
{
    switch (fs) {
    case MPU6050_GYRO_FS_250DPS:  return 131.0f;
    case MPU6050_GYRO_FS_500DPS:  return 65.5f;
    case MPU6050_GYRO_FS_1000DPS: return 32.8f;
    case MPU6050_GYRO_FS_2000DPS: return 16.4f;
    default:                      return 65.5f;
    }
}

/* ------------------------------------------------------------------------- */
/* Quet bus                                                                   */
/* ------------------------------------------------------------------------- */

esp_err_t mpu6050_bus_scan(i2c_master_bus_handle_t bus, int *n_found)
{
    if (!bus) {
        return ESP_ERR_INVALID_ARG;
    }

    int found = 0;
    ESP_LOGI(TAG, "Quet bus I2C (dia chi 7-bit 0x08..0x77)...");

    for (uint16_t addr = 0x08; addr <= 0x77; ++addr) {
        if (i2c_master_probe(bus, addr, 50) == ESP_OK) {
            const char *note = "";
            if (addr == MPU6050_ADDR_AD0_LOW)  note = "  <- MPU6050 (AD0 = GND)";
            if (addr == MPU6050_ADDR_AD0_HIGH) note = "  <- MPU6050 (AD0 = VCC)";
            ESP_LOGI(TAG, "  Tim thay thiet bi tai 0x%02X%s", addr, note);
            found++;
        }
    }

    if (found == 0) {
        ESP_LOGE(TAG, "Khong tim thay thiet bi nao tren bus. Kiem tra:");
        ESP_LOGE(TAG, "  - SDA -> GPIO21, SCL -> GPIO22 (co the bi doi cheo)");
        ESP_LOGE(TAG, "  - VCC -> 3V3 (KHONG phai 5V), GND -> GND");
        ESP_LOGE(TAG, "  - Dien tro pull-up: module GY-521 co san, MPU6050 tran thi khong");
        ESP_LOGE(TAG, "  - Moi han / day cam long");
    } else {
        ESP_LOGI(TAG, "Quet xong: %d thiet bi.", found);
    }

    if (n_found) {
        *n_found = found;
    }
    return ESP_OK;
}

esp_err_t mpu6050_bus_scan_standalone(const mpu6050_config_t *cfg, int *n_found)
{
    const i2c_master_bus_config_t bus_cfg = {
        .i2c_port                     = cfg->i2c_port,
        .sda_io_num                   = cfg->sda_io,
        .scl_io_num                   = cfg->scl_io,
        .clk_source                   = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt            = 7,
        .intr_priority                = 0,
        .trans_queue_depth            = 0,
        .flags.enable_internal_pullup = cfg->internal_pullup,
    };

    i2c_master_bus_handle_t bus = NULL;
    esp_err_t err = i2c_new_master_bus(&bus_cfg, &bus);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_new_master_bus that bai: %s", esp_err_to_name(err));
        return err;
    }

    err = mpu6050_bus_scan(bus, n_found);
    i2c_del_master_bus(bus);
    return err;
}

/* ------------------------------------------------------------------------- */
/* Chan doan                                                                  */
/* ------------------------------------------------------------------------- */

esp_err_t mpu6050_who_am_i(mpu6050_handle_t dev, uint8_t *out_id)
{
    if (!dev || !out_id) {
        return ESP_ERR_INVALID_ARG;
    }
    return reg_read8(dev, REG_WHO_AM_I, out_id);
}

static const char *chip_name_of(uint8_t id)
{
    switch (id) {
    case 0x68: return "MPU6050";
    case 0x70: return "MPU6500 hoac MPU9250 (khong phai MPU6050!)";
    case 0x71: return "MPU9250";
    case 0x73: return "MPU9255";
    case 0x12: return "ICM20948";
    case 0x00: return "khong co gi (doc ra 0x00 - thuong la mat SDA)";
    case 0xFF: return "khong co gi (doc ra 0xFF - thuong la thieu pull-up)";
    default:   return "khong ro";
    }
}

esp_err_t mpu6050_verify_config(mpu6050_handle_t dev)
{
    if (!dev) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Doc lai chinh nhung thanh ghi ta da ghi. Kiem tra ma tra ve cua ham ghi
     * la KHONG DU: chip co the ACK roi van khong luu (sut nguon, reset giua
     * duong, hoac ghi vao mot chip khac hoan toan cung dia chi). */
    const struct { uint8_t reg; uint8_t want; const char *name; } expect[] = {
        { REG_SMPLRT_DIV,   dev->cfg.smplrt_div,               "SMPLRT_DIV"   },
        { REG_CONFIG,       (uint8_t)dev->cfg.dlpf,            "CONFIG/DLPF"  },
        { REG_GYRO_CONFIG,  (uint8_t)(dev->cfg.gyro_fs << 3),  "GYRO_CONFIG"  },
        { REG_ACCEL_CONFIG, (uint8_t)(dev->cfg.accel_fs << 3), "ACCEL_CONFIG" },
        { REG_PWR_MGMT_1,   PWR1_CLKSEL_PLL_XGYRO,             "PWR_MGMT_1"   },
        { REG_PWR_MGMT_2,   0x00,                              "PWR_MGMT_2"   },
    };

    esp_err_t result = ESP_OK;
    for (size_t i = 0; i < sizeof expect / sizeof expect[0]; ++i) {
        uint8_t got = 0;
        const esp_err_t err = reg_read8(dev, expect[i].reg, &got);
        if (err != ESP_OK) {
            return err;
        }
        if (got != expect[i].want) {
            ESP_LOGE(TAG, "Thanh ghi %s (0x%02X) khong dung: ghi 0x%02X, doc lai 0x%02X",
                     expect[i].name, expect[i].reg, expect[i].want, got);
            result = ESP_ERR_INVALID_STATE;
        }
    }

    if (result == ESP_OK) {
        ESP_LOGI(TAG, "Doc lai cau hinh: toan bo %u thanh ghi khop.",
                 (unsigned)(sizeof expect / sizeof expect[0]));
    }
    return result;
}

esp_err_t mpu6050_dump_config(mpu6050_handle_t dev)
{
    if (!dev) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t who = 0, smp = 0, cfgr = 0, gyr = 0, acc = 0, p1 = 0, p2 = 0;
    esp_err_t err;
    if ((err = reg_read8(dev, REG_WHO_AM_I,      &who))  != ESP_OK) return err;
    if ((err = reg_read8(dev, REG_SMPLRT_DIV,    &smp))  != ESP_OK) return err;
    if ((err = reg_read8(dev, REG_CONFIG,        &cfgr)) != ESP_OK) return err;
    if ((err = reg_read8(dev, REG_GYRO_CONFIG,   &gyr))  != ESP_OK) return err;
    if ((err = reg_read8(dev, REG_ACCEL_CONFIG,  &acc))  != ESP_OK) return err;
    if ((err = reg_read8(dev, REG_PWR_MGMT_1,    &p1))   != ESP_OK) return err;
    if ((err = reg_read8(dev, REG_PWR_MGMT_2,    &p2))   != ESP_OK) return err;

    ESP_LOGI(TAG, "--- Cau hinh MPU6050 ---");
    ESP_LOGI(TAG, "  WHO_AM_I     0x%02X (%s)", who, chip_name_of(who));
    ESP_LOGI(TAG, "  PWR_MGMT_1   0x%02X  (sleep=%d, clksel=%d)",
             p1, (p1 & PWR1_SLEEP) ? 1 : 0, p1 & 0x07);
    ESP_LOGI(TAG, "  PWR_MGMT_2   0x%02X", p2);
    ESP_LOGI(TAG, "  CONFIG       0x%02X  (DLPF_CFG=%d)", cfgr, cfgr & 0x07);
    ESP_LOGI(TAG, "  SMPLRT_DIV   0x%02X  -> %.1f Hz", smp,
             (double)mpu6050_sample_rate_hz(dev));
    ESP_LOGI(TAG, "  GYRO_CONFIG  0x%02X  (FS_SEL=%d -> %.1f LSB/(deg/s))",
             gyr, (gyr >> 3) & 0x03, (double)dev->gyro_sens);
    ESP_LOGI(TAG, "  ACCEL_CONFIG 0x%02X  (AFS_SEL=%d -> %.0f LSB/g)",
             acc, (acc >> 3) & 0x03, (double)dev->accel_sens);
    ESP_LOGI(TAG, "  SDA=GPIO%d  SCL=GPIO%d  addr=0x%02X  %lu Hz",
             (int)dev->cfg.sda_io, (int)dev->cfg.scl_io, dev->cfg.dev_addr,
             (unsigned long)dev->cfg.scl_speed_hz);
    return ESP_OK;
}

/* ------------------------------------------------------------------------- */
/* Khoi tao                                                                   */
/* ------------------------------------------------------------------------- */

static esp_err_t mpu6050_reset_and_configure(mpu6050_handle_t d)
{
    esp_err_t err;

    /* Buoc 1: reset toan bo thiet bi. Bat buoc de khong ke thua trang thai la
     * tu lan chay truoc (ESP32 reset mem KHONG reset MPU6050). */
    if ((err = reg_write(d, REG_PWR_MGMT_1, PWR1_DEVICE_RESET)) != ESP_OK) return err;

    /* Cho bit DEVICE_RESET tu xoa. Datasheet noi khoang 100 ms; ta poll thay vi
     * cho cung mot khoang, nhung van co tran de khong treo mai. */
    bool cleared = false;
    for (int i = 0; i < 20; ++i) {
        vTaskDelay(pdMS_TO_TICKS(10));
        uint8_t v = 0;
        if (reg_read8(d, REG_PWR_MGMT_1, &v) == ESP_OK && !(v & PWR1_DEVICE_RESET)) {
            cleared = true;
            break;
        }
    }
    if (!cleared) {
        ESP_LOGE(TAG, "Bit DEVICE_RESET khong tu xoa sau 200 ms");
        return ESP_ERR_TIMEOUT;
    }

    /* Buoc 2: reset duong tin hieu. Datasheet khuyen lam sau device reset de
     * xoa sach cac thanh ghi tin hieu tuong tu/so. */
    if ((err = reg_write(d, REG_SIGNAL_PATH_RESET, SIGPATH_RESET_ALL)) != ESP_OK) return err;
    vTaskDelay(pdMS_TO_TICKS(100));

    /* Buoc 3: thoat sleep va chon nguon clock.
     * CLKSEL = 1 (PLL khoa theo con quay truc X) chinh xac hon dao dong noi
     * 8 MHz (CLKSEL = 0) rat nhieu - dao dong noi lech toi +-1%, va sai so tan
     * so lay mau se bien thanh sai so ti le trong tich phan goc. */
    if ((err = reg_write(d, REG_PWR_MGMT_1, PWR1_CLKSEL_PLL_XGYRO)) != ESP_OK) return err;
    vTaskDelay(pdMS_TO_TICKS(10));

    /* Buoc 4: bat toan bo cam bien (khong truc nao o che do standby). */
    if ((err = reg_write(d, REG_PWR_MGMT_2, 0x00)) != ESP_OK) return err;

    /* Buoc 5: bo loc thong thap so tren chip - lop chong rung dau tien. */
    if ((err = reg_write(d, REG_CONFIG, (uint8_t)d->cfg.dlpf)) != ESP_OK) return err;

    /* Buoc 6: tan so lay mau. Phai ghi SAU CONFIG vi y nghia cua SMPLRT_DIV
     * phu thuoc DLPF_CFG (1 kHz khi DLPF != 0, 8 kHz khi DLPF == 0). */
    if ((err = reg_write(d, REG_SMPLRT_DIV, d->cfg.smplrt_div)) != ESP_OK) return err;

    /* Buoc 7: thang do. */
    if ((err = reg_write(d, REG_GYRO_CONFIG,
                         (uint8_t)(d->cfg.gyro_fs << 3))) != ESP_OK) return err;
    if ((err = reg_write(d, REG_ACCEL_CONFIG,
                         (uint8_t)(d->cfg.accel_fs << 3))) != ESP_OK) return err;

    /* Buoc 8: tat FIFO va ngat - du an nay doc bang polling dinh ky.
     * Ghi tuong minh thay vi tin vao gia tri mac dinh sau reset. */
    if ((err = reg_write(d, REG_FIFO_EN,    0x00)) != ESP_OK) return err;
    if ((err = reg_write(d, REG_INT_ENABLE, 0x00)) != ESP_OK) return err;

    vTaskDelay(pdMS_TO_TICKS(50));   /* cho cam bien on dinh sau khi doi cau hinh */
    return ESP_OK;
}

esp_err_t mpu6050_create(const mpu6050_config_t *cfg, mpu6050_handle_t *out)
{
    if (!cfg || !out) {
        return ESP_ERR_INVALID_ARG;
    }
    *out = NULL;

    mpu6050_handle_t d = calloc(1, sizeof(struct mpu6050_dev_t));
    if (!d) {
        return ESP_ERR_NO_MEM;
    }
    d->cfg        = *cfg;
    d->accel_sens = accel_sens_of(cfg->accel_fs);
    d->gyro_sens  = gyro_sens_of(cfg->gyro_fs);

    const i2c_master_bus_config_t bus_cfg = {
        .i2c_port                     = cfg->i2c_port,
        .sda_io_num                   = cfg->sda_io,
        .scl_io_num                   = cfg->scl_io,
        .clk_source                   = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt            = 7,
        .intr_priority                = 0,
        .trans_queue_depth            = 0,
        .flags.enable_internal_pullup = cfg->internal_pullup,
    };

    esp_err_t err = i2c_new_master_bus(&bus_cfg, &d->bus);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_new_master_bus that bai: %s", esp_err_to_name(err));
        free(d);
        return err;
    }
    d->owns_bus = true;

    const i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = cfg->dev_addr,
        .scl_speed_hz    = cfg->scl_speed_hz,
    };
    err = i2c_master_bus_add_device(d->bus, &dev_cfg, &d->dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_master_bus_add_device that bai: %s", esp_err_to_name(err));
        goto fail;
    }

    /* --- Kiem tra co thiet bi o dia chi nay khong, TRUOC khi ghi bat cu gi ---
     * Neu day sai thi ta se biet ngay bang mot thong bao ro rang, thay vi mot
     * chuoi loi ghi thanh ghi kho hieu. */
    err = i2c_master_probe(d->bus, cfg->dev_addr, 100);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Khong co thiet bi nao tra loi tai dia chi 0x%02X (%s)",
                 cfg->dev_addr, esp_err_to_name(err));
        ESP_LOGE(TAG, "Dang quet ca bus de tim:");
        mpu6050_bus_scan(d->bus, NULL);
        goto fail;
    }

    /* --- Kiem tra dung chip khong --- */
    uint8_t who = 0;
    err = reg_read8(d, REG_WHO_AM_I, &who);
    if (err != ESP_OK) {
        goto fail;
    }
    if (who != MPU6050_WHO_AM_I_VAL) {
        ESP_LOGE(TAG, "WHO_AM_I = 0x%02X, mong doi 0x%02X. Chip doc duoc: %s",
                 who, MPU6050_WHO_AM_I_VAL, chip_name_of(who));
        ESP_LOGE(TAG, "Rat nhieu module ban ra duoi ten \"MPU6050\" thuc chat la");
        ESP_LOGE(TAG, "MPU6500/MPU9250 (WHO_AM_I = 0x70/0x71). Ban do thanh ghi");
        ESP_LOGE(TAG, "gan giong nhung khong dong nhat.");
        err = ESP_ERR_NOT_SUPPORTED;
        goto fail;
    }
    ESP_LOGI(TAG, "Tim thay MPU6050 tai 0x%02X (WHO_AM_I = 0x%02X)", cfg->dev_addr, who);

    /* --- Reset va nap cau hinh --- */
    if ((err = mpu6050_reset_and_configure(d)) != ESP_OK) {
        goto fail;
    }

    /* --- Doc lai de xac nhan cau hinh da vao that --- */
    if ((err = mpu6050_verify_config(d)) != ESP_OK) {
        goto fail;
    }

    ESP_LOGI(TAG, "Khoi tao xong: %.1f Hz, gia toc ke +-%dg, con quay +-%d deg/s, DLPF %d",
             (double)mpu6050_sample_rate_hz(d),
             2 << (int)cfg->accel_fs,
             250 << (int)cfg->gyro_fs,
             (int)cfg->dlpf);

    *out = d;
    return ESP_OK;

fail:
    if (d->dev) {
        i2c_master_bus_rm_device(d->dev);
    }
    if (d->bus && d->owns_bus) {
        i2c_del_master_bus(d->bus);
    }
    free(d);
    return err;
}

void mpu6050_delete(mpu6050_handle_t dev)
{
    if (!dev) {
        return;
    }
    /* Cho chip ngu de tiet kien dien - lich su nhung khong bat buoc. */
    reg_write(dev, REG_PWR_MGMT_1, PWR1_SLEEP);

    if (dev->dev) {
        i2c_master_bus_rm_device(dev->dev);
    }
    if (dev->bus && dev->owns_bus) {
        i2c_del_master_bus(dev->bus);
    }
    free(dev);
}

/* ------------------------------------------------------------------------- */
/* Doc du lieu                                                                */
/* ------------------------------------------------------------------------- */

esp_err_t mpu6050_read_raw(mpu6050_handle_t dev, mpu6050_raw_t *out)
{
    if (!dev || !out) {
        return ESP_ERR_INVALID_ARG;
    }

    /* 14 byte lien tiep tu 0x3B:
     *   ACCEL_X/Y/Z (6) | TEMP (2) | GYRO_X/Y/Z (6)
     * Mot giao dich duy nhat - xem ghi chu trong mpu6050.h ve viec tai sao
     * khong duoc doc rieng gia toc ke va con quay. */
    uint8_t b[14];
    const esp_err_t err = reg_read(dev, REG_ACCEL_XOUT_H, b, sizeof b);
    if (err != ESP_OK) {
        return err;
    }

    /* Big-endian, bu hai. Ep sang int16_t truoc khi gan de phep bu hai duoc
     * thuc hien dung, khong phu thuoc do rong cua int tren nen tang. */
    out->ax   = (int16_t)((uint16_t)b[0]  << 8 | b[1]);
    out->ay   = (int16_t)((uint16_t)b[2]  << 8 | b[3]);
    out->az   = (int16_t)((uint16_t)b[4]  << 8 | b[5]);
    out->temp = (int16_t)((uint16_t)b[6]  << 8 | b[7]);
    out->gx   = (int16_t)((uint16_t)b[8]  << 8 | b[9]);
    out->gy   = (int16_t)((uint16_t)b[10] << 8 | b[11]);
    out->gz   = (int16_t)((uint16_t)b[12] << 8 | b[13]);
    return ESP_OK;
}

esp_err_t mpu6050_read(mpu6050_handle_t dev, imu_sample_t *out)
{
    if (!out) {
        return ESP_ERR_INVALID_ARG;
    }

    mpu6050_raw_t r;
    const esp_err_t err = mpu6050_read_raw(dev, &r);
    if (err != ESP_OK) {
        return err;
    }

    const float ka = 1.0f / dev->accel_sens;   /* LSB -> g */
    const float kg = 1.0f / dev->gyro_sens;    /* LSB -> deg/s */

    out->ax = (float)r.ax * ka;
    out->ay = (float)r.ay * ka;
    out->az = (float)r.az * ka;
    out->gx = (float)r.gx * kg;
    out->gy = (float)r.gy * kg;
    out->gz = (float)r.gz * kg;
    return ESP_OK;
}

esp_err_t mpu6050_read_temp(mpu6050_handle_t dev, float *out_celsius)
{
    if (!out_celsius) {
        return ESP_ERR_INVALID_ARG;
    }
    mpu6050_raw_t r;
    const esp_err_t err = mpu6050_read_raw(dev, &r);
    if (err != ESP_OK) {
        return err;
    }
    /* Cong thuc datasheet muc 4.18: T(degC) = raw/340 + 36.53 */
    *out_celsius = (float)r.temp / 340.0f + 36.53f;
    return ESP_OK;
}

/* ------------------------------------------------------------------------- */

float mpu6050_accel_sensitivity(mpu6050_handle_t dev)
{
    return dev ? dev->accel_sens : 0.0f;
}

float mpu6050_gyro_sensitivity(mpu6050_handle_t dev)
{
    return dev ? dev->gyro_sens : 0.0f;
}

float mpu6050_sample_rate_hz(mpu6050_handle_t dev)
{
    if (!dev) {
        return 0.0f;
    }
    /* Tan so ra goc: 8 kHz khi DLPF_CFG == 0, nguoc lai 1 kHz. */
    const float base = (dev->cfg.dlpf == MPU6050_DLPF_260HZ) ? 8000.0f : 1000.0f;
    return base / (float)(1 + dev->cfg.smplrt_div);
}

i2c_master_bus_handle_t mpu6050_bus_handle(mpu6050_handle_t dev)
{
    return dev ? dev->bus : NULL;
}
