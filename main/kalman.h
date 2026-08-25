/*
 * kalman.h - Bo loc Kalman 2 trang thai cho uoc luong goc nghieng tu IMU 6 truc.
 *
 * KHONG phu thuoc ESP-IDF: chi dung <stdbool.h>. Nho vay file nay bien dich
 * duoc ca cho ESP32 (xtensa-esp-elf-gcc) va cho host PC (gcc) trong host_test/,
 * cho phep kiem chung thuat toan truoc khi co phan cung.
 *
 * Mo hinh:
 *   Trang thai   x = [ theta , b ]^T     theta: goc (deg), b: bias con quay (deg/s)
 *   Dieu khien   u = omega               omega: toc do goc do bang con quay (deg/s)
 *   Do luong     z = goc tinh tu gia toc ke (deg)
 *
 *   Du doan:  theta_k = theta_{k-1} + (omega - b) * dt
 *             b_k     = b_{k-1}
 *   Do luong: z = theta + nhieu,   H = [1  0]
 *
 * Con quay dong vai tro DAU VAO DIEU KHIEN (khong phai measurement) - day la
 * diem khac biet then chot so voi cac trien khai sai thuong gap tren mang.
 */
#ifndef KALMAN_H
#define KALMAN_H

#include <stdbool.h>

typedef struct {
    /* --- trang thai uoc luong --- */
    float angle;        /* x[0]: goc uoc luong (deg)              */
    float bias;         /* x[1]: bias con quay uoc luong (deg/s)  */
    float P[2][2];      /* ma tran hiep phuong sai sai so         */

    /* --- tham so tinh chinh --- */
    float Q_angle;      /* nhieu qua trinh cua goc      (deg^2/s)   */
    float Q_bias;       /* nhieu qua trinh cua bias   ((deg/s)^2/s) */
    float R_measure;    /* phuong sai nhieu do cua gia toc ke (deg^2) */

    /* --- tuy chon --- */
    bool  wrap_360;     /* true: goc quan quanh [-180,180) (dung cho roll) */

    /* --- so lieu chan doan (chi de quan sat, khong tham gia tinh toan) --- */
    float last_innov;   /* y = z - theta_du_doan (deg) */
    float last_R;       /* R thuc su da dung trong buoc cap nhat vua roi */
    float last_K[2];    /* do loi Kalman cua buoc vua roi */
} kalman_t;

/* Gia tri mac dinh tot cho MPU6050 @100-200 Hz. */
#define KALMAN_DEFAULT_Q_ANGLE   0.001f
#define KALMAN_DEFAULT_Q_BIAS    0.003f
#define KALMAN_DEFAULT_R_MEASURE 0.03f

/* Khoi tao. Truyen 0 cho bat ky tham so nao de dung gia tri mac dinh. */
void  kalman_init(kalman_t *k, float Q_angle, float Q_bias, float R_measure);

/* Ep goc ve mot gia tri xac dinh (dung luc khoi dong: seed bang goc gia toc ke,
 * tranh giai doan hoi tu ban dau keo dai). Reset luon P ve trang thai "tin cay". */
void  kalman_set_angle(kalman_t *k, float angle);

/* Mot buoc du doan + cap nhat day du.
 *   new_angle: goc do tu gia toc ke (deg)
 *   new_rate : toc do goc tu con quay (deg/s)
 *   dt       : chu ky lay mau thuc te (s)
 * Tra ve goc uoc luong moi (deg). */
float kalman_update(kalman_t *k, float new_angle, float new_rate, float dt);

/* Nhu kalman_update nhung ghi de R cho rieng buoc nay - dung cho R thich nghi
 * khi phat hien rung/gia toc tuyen tinh lam gia toc ke mat tin cay. */
float kalman_update_R(kalman_t *k, float new_angle, float new_rate, float dt, float R);

/* Chi du doan, khong cap nhat - dung khi gia toc ke hoan toan khong dang tin
 * (dang bi lac manh). P se phinh ra, dung y nghia vat ly. */
float kalman_predict(kalman_t *k, float new_rate, float dt);

/* Quan goc ve [-180, 180). Ham phu tro dung chung. */
float kalman_wrap180(float deg);

#endif /* KALMAN_H */
