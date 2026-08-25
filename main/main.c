/*
 * main.c - Uoc luong goc real-time tu MPU6050 bang bo loc Kalman, tren ESP32.
 *
 * Luong chay:
 *   1. Khoi tao NVS  -> de luu ket qua hieu chinh giua cac lan khoi dong
 *   2. Khoi tao MPU6050 (quet bus + WHO_AM_I + doc lai cau hinh de xac nhan)
 *   3. Nap hieu chinh tu NVS. Neu chua co, hoac nguoi dung giu nut BOOT khi
 *      khoi dong, thi chay hieu chinh moi
 *   4. Task 200 Hz: doc IMU -> tru offset -> hop nhat Kalman -> xuat CSV 50 Hz
 *
 * Chu ky lay mau duoc GIU bang xTaskDelayUntil, nhung dt dua vao bo loc la dt
 * DO THUC TE bang esp_timer. Neu task bi tre (ngat, flash, WiFi) thi bo loc van
 * dung; dung hang so 0.005f o day la mot loi am tham va rat kho truy.
 */
#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "nvs_flash.h"
#include "nvs.h"

#include "mpu6050.h"
#include "ahrs.h"
#include "imu_calib.h"

static const char *TAG = "main";

/* --- Thong so vong chay --- */
#define SAMPLE_RATE_HZ    200
#define SAMPLE_PERIOD_MS  (1000 / SAMPLE_RATE_HZ)       /* 5 ms - CAN FREERTOS_HZ=1000 */
#define CSV_DECIMATE      4                              /* 200/4 = 50 Hz ra UART */
#define CALIB_SAMPLES     1000                           /* 5 s @ 200 Hz */
#define CALIB_MAX_RETRY   3

/* Nut BOOT tren ESP32 DevKit V1. Giu luc khoi dong -> buoc hieu chinh lai. */
#define BOOT_BUTTON_GPIO  GPIO_NUM_0

/* --- Luu tru NVS --- */
#define NVS_NAMESPACE     "imu"
#define NVS_KEY_CALIB     "calib"

typedef struct {
    mpu6050_handle_t imu;
    imu_calib_t      calib;
} app_ctx_t;

/* ------------------------------------------------------------------------- */
/* Luu / nap hieu chinh                                                       */
/* ------------------------------------------------------------------------- */

static esp_err_t calib_load(imu_calib_t *out)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &h);
    if (err != ESP_OK) {
        return err;
    }

    size_t len = sizeof(imu_calib_t);
    err = nvs_get_blob(h, NVS_KEY_CALIB, out, &len);
    nvs_close(h);

    if (err == ESP_OK && len != sizeof(imu_calib_t)) {
        /* Kich thuoc khac -> blob duoc ghi boi mot phien ban struct khac.
         * Bo di thay vi doc nham, vi doc nham se cho bias sai la rat kho truy. */
        ESP_LOGW(TAG, "Blob hieu chinh trong NVS sai kich thuoc (%u, mong doi %u) - bo qua",
                 (unsigned)len, (unsigned)sizeof(imu_calib_t));
        return ESP_ERR_INVALID_SIZE;
    }
    return err;
}

static esp_err_t calib_save(const imu_calib_t *c)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        return err;
    }
    err = nvs_set_blob(h, NVS_KEY_CALIB, c, sizeof(imu_calib_t));
    if (err == ESP_OK) {
        err = nvs_commit(h);
    }
    nvs_close(h);
    return err;
}

/* ------------------------------------------------------------------------- */
/* Hieu chinh offset                                                          */
/* ------------------------------------------------------------------------- */

static esp_err_t run_calibration(mpu6050_handle_t imu, imu_calib_t *out)
{
    for (int attempt = 1; attempt <= CALIB_MAX_RETRY; ++attempt) {
        ESP_LOGW(TAG, "===== HIEU CHINH OFFSET (lan %d/%d) =====",
                 attempt, CALIB_MAX_RETRY);
        ESP_LOGW(TAG, "DAT THIET BI NAM YEN TREN MAT PHANG NGANG.");
        ESP_LOGW(TAG, "Bat dau sau 3 giay, lay mau trong %.1f giay...",
                 (double)CALIB_SAMPLES / SAMPLE_RATE_HZ);
        vTaskDelay(pdMS_TO_TICKS(3000));

        imu_calib_acc_t acc;
        imu_calib_acc_init(&acc, CALIB_SAMPLES);

        TickType_t wake = xTaskGetTickCount();
        bool read_ok = true;

        for (int i = 0; i < CALIB_SAMPLES; ++i) {
            xTaskDelayUntil(&wake, pdMS_TO_TICKS(SAMPLE_PERIOD_MS));

            imu_sample_t s;
            if (mpu6050_read(imu, &s) != ESP_OK) {
                ESP_LOGE(TAG, "Doc IMU that bai o mau %d - huy lan hieu chinh nay", i);
                read_ok = false;
                break;
            }
            imu_calib_acc_add(&acc, &s);

            if ((i % (CALIB_SAMPLES / 5)) == 0) {
                ESP_LOGI(TAG, "  ... %d%%", i * 100 / CALIB_SAMPLES);
            }
        }
        if (!read_ok) {
            continue;
        }

        const imu_calib_status_t st = imu_calib_acc_finish(&acc, out);

        ESP_LOGI(TAG, "Ket qua: %s", imu_calib_status_str(st));
        ESP_LOGI(TAG, "  Bias con quay : %+7.3f %+7.3f %+7.3f deg/s",
                 (double)out->gyro_bias[0], (double)out->gyro_bias[1],
                 (double)out->gyro_bias[2]);
        ESP_LOGI(TAG, "  Do lech chuan : %7.3f %7.3f %7.3f deg/s",
                 (double)acc.gyro_std[0], (double)acc.gyro_std[1],
                 (double)acc.gyro_std[2]);
        ESP_LOGI(TAG, "  |a| trung binh: %7.4f g", (double)acc.acc_norm_mean);
        ESP_LOGI(TAG, "  Offset gia toc: %+7.4f %+7.4f %+7.4f g",
                 (double)out->accel_off[0], (double)out->accel_off[1],
                 (double)out->accel_off[2]);

        if (st == IMU_CALIB_OK) {
            return ESP_OK;
        }
        if (st == IMU_CALIB_ERR_NOT_LEVEL) {
            /* Bias con quay - phan quan trong nhat - van dung. Chi mat phan
             * offset gia toc ke, ma no chi gay lech goc tinh vai phan do. */
            ESP_LOGW(TAG, "Thiet bi khong nam phang: chi hieu chinh duoc con quay.");
            ESP_LOGW(TAG, "Goc co the lech mot chut. Dat phang roi hieu chinh lai de het.");
            return ESP_OK;
        }
        ESP_LOGE(TAG, "Hieu chinh khong hop le - thu lai.");
    }

    ESP_LOGE(TAG, "Hieu chinh that bai sau %d lan. Chay khong hieu chinh:", CALIB_MAX_RETRY);
    ESP_LOGE(TAG, "  bo loc Kalman se tu uoc luong bias, nhung mat vai chuc giay");
    ESP_LOGE(TAG, "  de hoi tu, va yaw se troi nhieu hon.");
    imu_calib_zero(out);
    return ESP_FAIL;
}

/* ------------------------------------------------------------------------- */
/* Task hop nhat 200 Hz                                                       */
/* ------------------------------------------------------------------------- */

static void imu_task(void *arg)
{
    app_ctx_t *ctx = (app_ctx_t *)arg;

    ahrs_t ahrs;
    ahrs_init(&ahrs, NULL);

    /* Tieu de CSV. Bat dau bang '#' de cong cu phia PC bo qua duoc, va de phan
     * biet voi cac dong log cua ESP_LOG lan vao cung mot UART. */
    printf("#t_ms,roll_acc,pitch_acc,roll_gyro,pitch_gyro,roll_comp,pitch_comp,"
           "roll_kf,pitch_kf,yaw_kf,bias_x,bias_y,bias_z,acc_norm,R_used,is_static\n");

    TickType_t  wake      = xTaskGetTickCount();
    int64_t     t_prev_us = esp_timer_get_time();
    uint32_t    n         = 0;
    uint32_t    err_count = 0;
    int64_t     t0_us     = t_prev_us;

    for (;;) {
        xTaskDelayUntil(&wake, pdMS_TO_TICKS(SAMPLE_PERIOD_MS));

        /* dt DO THUC TE, khong dung hang so - xem ghi chu dau file. */
        const int64_t t_now_us = esp_timer_get_time();
        const float   dt       = (float)(t_now_us - t_prev_us) * 1e-6f;

        /* KHONG cap nhat t_prev_us o day. Neu doc that bai roi van nhay moc thoi
         * gian len thi khoang thoi gian vua troi qua bi xoa khoi truc thoi gian
         * cua bo loc: lan doc thanh cong tiep theo se thay dt = 5 ms trong khi
         * thuc te da qua 10 ms, va phan tich phan bi thieu di dung mot chu ky.
         * Voi toc do 100 deg/s do la 0.5 do bi mat cho MOI mau bi rot, va sai so
         * do tich luy mot chieu chu khong tu triet tieu.
         * Cach dung: chi nhay moc sau khi da co du lieu, de dt cua lan sau phu
         * tron khoang trong (giu bac 0 - xap xi tot nhat co the tu mot phep do). */
        imu_sample_t s;
        const esp_err_t err = mpu6050_read(ctx->imu, &s);
        if (err != ESP_OK) {
            /* Mot lan loi I2C don le khong dang de dung ca he thong; nhung loi
             * lien tuc thi phai bao. */
            if (++err_count % 200 == 1) {
                ESP_LOGE(TAG, "Doc IMU that bai (%" PRIu32 " lan): %s",
                         err_count, esp_err_to_name(err));
            }
            continue;
        }

        /* Da co du lieu -> gio moi nhay moc thoi gian. */
        t_prev_us = t_now_us;

        imu_calib_apply(&ctx->calib, &s);

        ahrs_out_t o;
        ahrs_update(&ahrs, &s, dt, &o);

        if (++n % CSV_DECIMATE == 0) {
            printf("%" PRId64 ",%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,"
                   "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.4f,%.5f,%d\n",
                   (t_now_us - t0_us) / 1000,
                   (double)o.roll_acc,  (double)o.pitch_acc,
                   (double)o.roll_gyro, (double)o.pitch_gyro,
                   (double)o.roll_comp, (double)o.pitch_comp,
                   (double)o.roll,      (double)o.pitch, (double)o.yaw,
                   (double)o.bias_x,    (double)o.bias_y, (double)o.bias_z,
                   (double)o.acc_norm,  (double)o.R_used,
                   o.is_static ? 1 : 0);
        }
    }
}

/* ------------------------------------------------------------------------- */

static bool boot_button_held(void)
{
    const gpio_config_t io = {
        .pin_bit_mask = 1ULL << BOOT_BUTTON_GPIO,
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    if (gpio_config(&io) != ESP_OK) {
        return false;
    }
    /* Nut BOOT keo xuong GND khi nhan -> muc thap la dang giu. */
    return gpio_get_level(BOOT_BUTTON_GPIO) == 0;
}

void app_main(void)
{
    static app_ctx_t ctx;

    ESP_LOGI(TAG, "=====================================================");
    ESP_LOGI(TAG, " Uoc luong goc MPU6050 bang bo loc Kalman - ESP32");
    ESP_LOGI(TAG, "=====================================================");
    ESP_LOGI(TAG, "Noi day: SDA->GPIO21  SCL->GPIO22  VCC->3V3  GND->GND");
    ESP_LOGI(TAG, "         AD0->GND (dia chi 0x68).  KHONG dung 5V.");

    /* --- NVS --- */
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS can khoi tao lai (%s) - dang xoa", esp_err_to_name(err));
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    /* --- MPU6050 --- */
    mpu6050_config_t cfg;
    mpu6050_config_default(&cfg);

    err = mpu6050_create(&cfg, &ctx.imu);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Khoi tao MPU6050 that bai: %s", esp_err_to_name(err));
        ESP_LOGE(TAG, "Dung lai. Sua noi day roi reset board.");
        /* Khong reboot lien tuc: vong lap vo tan lam log khong doc duoc va
         * khong giup gi cho viec chan doan. */
        for (;;) {
            vTaskDelay(pdMS_TO_TICKS(5000));
            ESP_LOGE(TAG, "Dang cho - kiem tra noi day, sau do reset board.");
        }
    }

    mpu6050_dump_config(ctx.imu);

    float temp_c = 0.0f;
    if (mpu6050_read_temp(ctx.imu, &temp_c) == ESP_OK) {
        /* Bias con quay troi theo nhiet do, nen ghi lai nhiet do luc hieu chinh
         * la thong tin co ich khi doi chieu ve sau. */
        ESP_LOGI(TAG, "Nhiet do chip: %.1f do C", (double)temp_c);
    }

    /* --- Hieu chinh --- */
    const bool force = boot_button_held();
    if (force) {
        ESP_LOGW(TAG, "Nut BOOT dang duoc giu -> buoc hieu chinh lai.");
    }

    bool need_calib = true;
    if (!force && calib_load(&ctx.calib) == ESP_OK && ctx.calib.valid) {
        ESP_LOGI(TAG, "Nap hieu chinh tu NVS:");
        ESP_LOGI(TAG, "  Bias con quay : %+7.3f %+7.3f %+7.3f deg/s",
                 (double)ctx.calib.gyro_bias[0], (double)ctx.calib.gyro_bias[1],
                 (double)ctx.calib.gyro_bias[2]);
        ESP_LOGI(TAG, "  Offset gia toc: %+7.4f %+7.4f %+7.4f g",
                 (double)ctx.calib.accel_off[0], (double)ctx.calib.accel_off[1],
                 (double)ctx.calib.accel_off[2]);
        ESP_LOGI(TAG, "  (giu nut BOOT luc khoi dong de hieu chinh lai)");
        need_calib = false;
    }

    if (need_calib) {
        run_calibration(ctx.imu, &ctx.calib);
        if (ctx.calib.valid && calib_save(&ctx.calib) == ESP_OK) {
            ESP_LOGI(TAG, "Da luu hieu chinh vao NVS.");
        }
    }

    /* --- Chay --- */
    ESP_LOGI(TAG, "Bat dau hop nhat o %d Hz, xuat CSV o %d Hz.",
             SAMPLE_RATE_HZ, SAMPLE_RATE_HZ / CSV_DECIMATE);

    /* Ghim vao core 1: core 0 chay cac task he thong (WiFi/BT neu bat sau nay),
     * de o day thi jitter chu ky lay mau se tang. */
    xTaskCreatePinnedToCore(imu_task, "imu", 4096, &ctx, 5, NULL, 1);
}
