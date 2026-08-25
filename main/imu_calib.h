/*
 * imu_calib.h - Hieu chinh offset cho MPU6050.
 *
 * File nay chi chua PHAN TOAN, khong include ESP-IDF - nho vay host_test kiem
 * chung duoc chinh thuat toan hieu chinh, khong phai ban copy. Phan luu NVS va
 * phan thu thap mau tu cam bien nam o main.c.
 *
 * Vi sao phai hieu chinh:
 *   - Con quay MPU6050 xuat xuong luon co bias 1-5 deg/s. Tich phan bias nay
 *     len la nguyen nhan so 1 gay drift. Bo loc Kalman CO uoc luong duoc bias
 *     nay (do la trang thai thu hai), nhung neu tru truoc thi filter chi con
 *     phai bam bias con lai rat nho -> hoi tu nhanh hon nhieu.
 *   - Gia toc ke lech vai chuc mg, dich thang goc uoc luong 1-2 do.
 *
 * Diem quan trong: hieu chinh CHI dung neu thiet bi that su nam yen va nam
 * phang luc lay mau. Neu khong kiem tra dieu do, ta se lang le nhan mot bo so
 * rac roi tin no mai mai. Vi vay imu_calib_finish tra ve trang thai chan doan
 * thay vi luon bao thanh cong.
 */
#ifndef IMU_CALIB_H
#define IMU_CALIB_H

#include <stdbool.h>
#include "ahrs.h"   /* imu_sample_t */

typedef struct {
    float gyro_bias[3];   /* deg/s - tru khoi so doc con quay */
    float accel_off[3];   /* g     - tru khoi so doc gia toc ke */
    bool  valid;
} imu_calib_t;

typedef enum {
    IMU_CALIB_OK = 0,
    IMU_CALIB_ERR_FEW_SAMPLES,  /* chua du mau */
    IMU_CALIB_ERR_MOVING,       /* thiet bi khong nam yen -> bias sai */
    IMU_CALIB_ERR_NOT_LEVEL,    /* khong nam phang -> offset gia toc ke vo nghia
                                 * (bias con quay VAN dung va van duoc tra ve) */
} imu_calib_status_t;

typedef struct {
    /* --- tich luy --- */
    int    n;
    double sum_g[3];
    double sum_a[3];
    double sumsq_g[3];

    /* --- nguong kiem tra tinh hop le --- */
    int   n_target;
    float max_gyro_std;      /* deg/s. Vuot -> coi la dang chuyen dong */
    float max_acc_norm_err;  /* g.     Vuot -> dang bi tac dong luc khac */
    float min_level_az;      /* g.     az trung binh phai lon hon nguong nay */

    /* --- so lieu chan doan sau khi finish --- */
    float gyro_std[3];
    float acc_norm_mean;
} imu_calib_acc_t;

/* Dat calib ve khong (khong hieu chinh gi). */
void imu_calib_zero(imu_calib_t *c);

/* Tru offset khoi mot mau tho, tai cho. */
void imu_calib_apply(const imu_calib_t *c, imu_sample_t *s);

/* n_target <= 0 thi dung mac dinh 1000 mau (5 s @ 200 Hz). */
void imu_calib_acc_init(imu_calib_acc_t *a, int n_target);

/* Nap them mot mau THO. Tra ve true khi da du n_target mau. */
bool imu_calib_acc_add(imu_calib_acc_t *a, const imu_sample_t *raw);

/* Chot ket qua. Luon dien gyro_bias neu du mau (ngay ca khi khong nam phang),
 * nhung chi dien accel_off khi thuc su nam phang va nam yen. */
imu_calib_status_t imu_calib_acc_finish(imu_calib_acc_t *a, imu_calib_t *out);

const char *imu_calib_status_str(imu_calib_status_t st);

#endif /* IMU_CALIB_H */
