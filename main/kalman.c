/*
 * kalman.c - Bo loc Kalman 2 trang thai. Xem kalman.h cho mo hinh toan hoc.
 *
 * KHONG include header ESP-IDF nao - file nay bien dich duoc bang ca
 * xtensa-esp-elf-gcc (cho ESP32) va gcc (cho host_test/).
 */
#include "kalman.h"
#include <math.h>

/* ------------------------------------------------------------------------- */

float kalman_wrap180(float deg)
{
    /* fmodf giu dau cua toan hang dau -> can buoc dua ve duong truoc khi tru. */
    deg = fmodf(deg + 180.0f, 360.0f);
    if (deg < 0.0f) {
        deg += 360.0f;
    }
    return deg - 180.0f;
}

void kalman_init(kalman_t *k, float Q_angle, float Q_bias, float R_measure)
{
    k->Q_angle   = (Q_angle   > 0.0f) ? Q_angle   : KALMAN_DEFAULT_Q_ANGLE;
    k->Q_bias    = (Q_bias    > 0.0f) ? Q_bias    : KALMAN_DEFAULT_Q_BIAS;
    k->R_measure = (R_measure > 0.0f) ? R_measure : KALMAN_DEFAULT_R_MEASURE;

    k->angle    = 0.0f;
    k->bias     = 0.0f;
    k->wrap_360 = false;

    /* P ban dau: chua biet gi ve goc lan bias.
     * P00 lon co y do - buoc cap nhat dau tien se cho K0 ~ 1, tuc filter nhan
     * gan tron goc gia toc ke va hoi tu ngay, thay vi bo dan tu 0 mat vai giay. */
    k->P[0][0] = 100.0f;
    k->P[0][1] = 0.0f;
    k->P[1][0] = 0.0f;
    k->P[1][1] = 1.0f;

    k->last_innov = 0.0f;
    k->last_R     = k->R_measure;
    k->last_K[0]  = 0.0f;
    k->last_K[1]  = 0.0f;
}

void kalman_set_angle(kalman_t *k, float angle)
{
    k->angle = k->wrap_360 ? kalman_wrap180(angle) : angle;

    /* Goc vua duoc gan bang mot phep do truc tiep -> do bat dinh cua goc chi
     * con bang nhieu do. Bias thi van chua biet, giu P11 lon.
     * Khong reset k->bias: uoc luong bias da hoc duoc van con gia tri. */
    k->P[0][0] = k->R_measure;
    k->P[0][1] = 0.0f;
    k->P[1][0] = 0.0f;
    k->P[1][1] = 1.0f;
}

/* ------------------------------------------------------------------------- */

/* Buoc du doan: x = F*x + G*u ;  P = F*P*F' + Q
 * F = [[1, -dt], [0, 1]] ,  G = [dt, 0]' ,  Q = diag(Q_angle*dt, Q_bias*dt) */
static void kalman_predict_step(kalman_t *k, float rate, float dt)
{
    /* Toc do goc THUC = toc do do duoc - bias. Day la cho con quay tham gia
     * nhu DAU VAO DIEU KHIEN u, khong phai nhu mot measurement. */
    k->angle += (rate - k->bias) * dt;
    if (k->wrap_360) {
        k->angle = kalman_wrap180(k->angle);
    }
    /* bias khong doi trong mo hinh du doan: coi la hang so + nhieu random-walk,
     * phan "nhieu" the hien qua Q_bias*dt cong vao P11 ben duoi. */

    /* Khai trien tuong F*P*F' cho ma tran 2x2 - re hon nhieu so voi nhan
     * ma tran tong quat, va tranh cap phat tam. Phai doc P vao bien local
     * truoc vi P[0][0] moi phu thuoc ca P01, P10, P11 cu. */
    const float P00 = k->P[0][0];
    const float P01 = k->P[0][1];
    const float P10 = k->P[1][0];
    const float P11 = k->P[1][1];

    k->P[0][0] = P00 + dt * (dt * P11 - P01 - P10) + k->Q_angle * dt;
    k->P[0][1] = P01 - dt * P11;
    k->P[1][0] = P10 - dt * P11;
    k->P[1][1] = P11 + k->Q_bias * dt;
}

/* Chong phan ra so hoc khi chay lien tuc nhieu gio. */
static void kalman_sanitize(kalman_t *k)
{
    /* Sai so lam tron tich luy lam P lech doi xung; lau dan sinh do loi Kalman
     * vo nghia. Cuong che P01 = P10 lai. */
    const float off = 0.5f * (k->P[0][1] + k->P[1][0]);
    k->P[0][1] = off;
    k->P[1][0] = off;

    /* Phuong sai am la vo nghia vat ly. */
    if (k->P[0][0] < 0.0f) k->P[0][0] = 0.0f;
    if (k->P[1][1] < 0.0f) k->P[1][1] = 0.0f;
}

/* ------------------------------------------------------------------------- */

float kalman_update_R(kalman_t *k, float new_angle, float new_rate, float dt, float R)
{
    /* dt phi ly (mau dau tien, timer nhay lui, task bi treo) -> bo qua buoc nay
     * thay vi lam no ma tran P. */
    if (!(dt > 0.0f) || dt > 1.0f) {
        return k->angle;
    }
    if (R < 1e-9f) {
        R = 1e-9f;   /* R = 0 nghia la "tin tuyet doi" -> S co the ve 0 -> chia 0 */
    }

    kalman_predict_step(k, new_rate, dt);

    /* Buoc cap nhat. H = [1 0] nen S la vo huong: khong can nghich dao ma tran,
     * do la ly do bo loc nay chay duoc thoai mai o 200 Hz tren ESP32. */
    float y = new_angle - k->angle;
    if (k->wrap_360) {
        /* Bat buoc cho roll/yaw: khong wrap thi luc vuot +-180 do lech se
         * nhay 360 do va filter giat manh. */
        y = kalman_wrap180(y);
    }

    const float S  = k->P[0][0] + R;
    const float K0 = k->P[0][0] / S;
    const float K1 = k->P[1][0] / S;

    k->angle += K0 * y;
    k->bias  += K1 * y;
    if (k->wrap_360) {
        k->angle = kalman_wrap180(k->angle);
    }

    /* P = (I - K*H) * P , voi H = [1 0] */
    const float P00 = k->P[0][0];
    const float P01 = k->P[0][1];
    k->P[0][0] -= K0 * P00;
    k->P[0][1] -= K0 * P01;
    k->P[1][0] -= K1 * P00;
    k->P[1][1] -= K1 * P01;

    kalman_sanitize(k);

    k->last_innov = y;
    k->last_R     = R;
    k->last_K[0]  = K0;
    k->last_K[1]  = K1;
    return k->angle;
}

float kalman_update(kalman_t *k, float new_angle, float new_rate, float dt)
{
    return kalman_update_R(k, new_angle, new_rate, dt, k->R_measure);
}

float kalman_predict(kalman_t *k, float new_rate, float dt)
{
    if (!(dt > 0.0f) || dt > 1.0f) {
        return k->angle;
    }

    kalman_predict_step(k, new_rate, dt);
    kalman_sanitize(k);

    /* Khong co measurement -> khong co do loi. Ghi 0 de phan chan doan phia
     * tren nhin vao la biet buoc nay da bo qua gia toc ke. */
    k->last_innov = 0.0f;
    k->last_R     = 0.0f;
    k->last_K[0]  = 0.0f;
    k->last_K[1]  = 0.0f;
    return k->angle;
}
