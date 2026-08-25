/*
 * imu_calib.c - Phan toan cua hieu chinh offset. Xem imu_calib.h.
 */
#include "imu_calib.h"
#include <math.h>

void imu_calib_zero(imu_calib_t *c)
{
    for (int i = 0; i < 3; ++i) {
        c->gyro_bias[i] = 0.0f;
        c->accel_off[i] = 0.0f;
    }
    c->valid = false;
}

void imu_calib_apply(const imu_calib_t *c, imu_sample_t *s)
{
    s->gx -= c->gyro_bias[0];
    s->gy -= c->gyro_bias[1];
    s->gz -= c->gyro_bias[2];

    s->ax -= c->accel_off[0];
    s->ay -= c->accel_off[1];
    s->az -= c->accel_off[2];
}

void imu_calib_acc_init(imu_calib_acc_t *a, int n_target)
{
    a->n = 0;
    for (int i = 0; i < 3; ++i) {
        a->sum_g[i]   = 0.0;
        a->sum_a[i]   = 0.0;
        a->sumsq_g[i] = 0.0;
        a->gyro_std[i] = 0.0f;
    }
    a->n_target = (n_target > 0) ? n_target : 1000;   /* 5 s @ 200 Hz */

    /* 2.0 deg/s long hon nhieu nen (0.4 deg/s RMS) nhung chat hon bat ky chuyen
     * dong tay that nao - nguong nay bat duoc viec nguoi dung cam thiet bi tren
     * tay thay vi dat xuong ban. */
    a->max_gyro_std     = 2.0f;
    a->max_acc_norm_err = 0.10f;
    a->min_level_az     = 0.90f;   /* nghieng qua ~25 do thi coi la khong phang */

    a->acc_norm_mean = 0.0f;
}

bool imu_calib_acc_add(imu_calib_acc_t *a, const imu_sample_t *raw)
{
    if (a->n >= a->n_target) {
        return true;
    }

    const float g[3] = { raw->gx, raw->gy, raw->gz };
    const float ac[3] = { raw->ax, raw->ay, raw->az };

    for (int i = 0; i < 3; ++i) {
        a->sum_g[i]   += (double)g[i];
        a->sumsq_g[i] += (double)g[i] * (double)g[i];
        a->sum_a[i]   += (double)ac[i];
    }
    a->n++;
    return (a->n >= a->n_target);
}

imu_calib_status_t imu_calib_acc_finish(imu_calib_acc_t *a, imu_calib_t *out)
{
    imu_calib_zero(out);

    if (a->n < a->n_target || a->n < 2) {
        return IMU_CALIB_ERR_FEW_SAMPLES;
    }

    const double inv_n = 1.0 / (double)a->n;
    double mean_g[3], mean_a[3];

    for (int i = 0; i < 3; ++i) {
        mean_g[i] = a->sum_g[i] * inv_n;
        mean_a[i] = a->sum_a[i] * inv_n;

        /* Phuong sai = E[x^2] - E[x]^2. Tinh o double vi voi 1000 mau
         * quanh mot gia tri khac 0, dang nay o float se mat het do chinh xac. */
        double var = a->sumsq_g[i] * inv_n - mean_g[i] * mean_g[i];
        if (var < 0.0) var = 0.0;              /* sai so lam tron */
        a->gyro_std[i] = (float)sqrt(var);
    }

    a->acc_norm_mean = (float)sqrt(mean_a[0] * mean_a[0] +
                                   mean_a[1] * mean_a[1] +
                                   mean_a[2] * mean_a[2]);

    /* Bias con quay luon dung duoc: khong phu thuoc huong dat thiet bi. */
    for (int i = 0; i < 3; ++i) {
        out->gyro_bias[i] = (float)mean_g[i];
    }

    /* --- Kiem tra: co dang nam yen khong? --- */
    for (int i = 0; i < 3; ++i) {
        if (a->gyro_std[i] > a->max_gyro_std) {
            return IMU_CALIB_ERR_MOVING;
        }
    }
    if (fabsf(a->acc_norm_mean - 1.0f) > a->max_acc_norm_err) {
        return IMU_CALIB_ERR_MOVING;
    }

    /* --- Kiem tra: co dang nam phang khong? ---
     * Offset gia toc ke chi tach duoc khoi trong luc neu biet truoc huong cua
     * trong luc. Ta gia dinh "dat phang, mat tren huong len" -> trong luc nam
     * tron tren truc Z. Neu khong phang thi khong the phan biet offset voi
     * thanh phan trong luc, nen bo hieu chinh gia toc ke thay vi doan bua. */
    if (mean_a[2] < (double)a->min_level_az) {
        out->valid = true;   /* bias con quay van dung */
        return IMU_CALIB_ERR_NOT_LEVEL;
    }

    out->accel_off[0] = (float)mean_a[0];
    out->accel_off[1] = (float)mean_a[1];
    out->accel_off[2] = (float)(mean_a[2] - 1.0);   /* truc Z con lai dung 1 g */
    out->valid = true;
    return IMU_CALIB_OK;
}

const char *imu_calib_status_str(imu_calib_status_t st)
{
    switch (st) {
    case IMU_CALIB_OK:               return "OK";
    case IMU_CALIB_ERR_FEW_SAMPLES:  return "chua du mau";
    case IMU_CALIB_ERR_MOVING:       return "thiet bi khong nam yen";
    case IMU_CALIB_ERR_NOT_LEVEL:    return "khong nam phang (chi hieu chinh con quay)";
    default:                         return "khong ro";
    }
}
