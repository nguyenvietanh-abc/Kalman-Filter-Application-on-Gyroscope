/*
 * ahrs.h - Tang hop nhat cam bien: bien accel + gyro thanh goc roll/pitch/yaw.
 *
 * Nhiem vu cua tang nay:
 *   1. Doi gia toc 3 truc -> goc roll/pitch tuyet doi (dua vao vector trong luc)
 *   2. Danh gia do tin cay cua gia toc ke va dieu chinh R theo thoi gian thuc
 *      (R thich nghi) - day la co che xu ly nhieu rung do lac tay
 *   3. Chay 3 bo uoc luong Kalman: roll, pitch, yaw
 *   4. Chay song song 3 baseline (accel-only, gyro-only, complementary) de co
 *      co so so sanh dinh luong - khong co chung thi khong chung minh duoc
 *      Kalman tot hon
 *
 * KHONG include header ESP-IDF nao - bien dich duoc ca tren host bang gcc.
 *
 * GIOI HAN DA BIET (nen ghi ro trong bao cao):
 *   - Bieu dien Euler suy bien khi pitch -> +-90 do (gimbal lock): roll tinh tu
 *     gia toc ke mat y nghia, va he so tan(theta) trong phep bien doi toc do
 *     tien ra vo cuc. Da chan bang CT_MIN, nhung do la chan thiet hai chu khong
 *     phai khac phuc. Muon het thi phai chuyen sang EKF quaternion.
 *   - Yaw KHONG kha quan sat bang IMU 6 truc. Xem phan ZRU trong ahrs.c.
 *   - Trang thai bias cua moi bo Kalman la bias cua TOC DO DA BIEN DOI, khong
 *     phai bias cua tung truc con quay. O goc nho hai thu nay trung nhau; o goc
 *     lon thi bias hieu dung phu thuoc goc, tuc vi pham gia dinh "bias hang so"
 *     cua mo hinh. Trong luong chay binh thuong, hieu chinh offset da tru sach
 *     bias tung truc nen phan du rat nho va anh huong la thu cap.
 */
#ifndef AHRS_H
#define AHRS_H

#include <stdbool.h>
#include "kalman.h"

/* Mot mau IMU da hieu chinh offset va da doi ve don vi vat ly. */
typedef struct {
    float ax, ay, az;   /* gia toc, don vi g   */
    float gx, gy, gz;   /* toc do goc, don vi deg/s */
} imu_sample_t;

typedef struct {
    /* --- tham so Kalman (0 = dung mac dinh cua kalman.h) --- */
    float Q_angle;
    float Q_bias;
    float R_measure;

    /* --- R thich nghi: xu ly nhieu rung / gia toc tuyen tinh --- */
    /* Diem then chot: R duoc tinh tu thanh phan nhieu DA LOC THONG THAP, khong
     * phai tu |a|-1 tuc thoi. Ly do do luong:
     *   - Rung dao dong (trung binh bang 0) lam |a|-1 tuc thoi tang manh, nhung
     *     ban than bo Kalman da triet tieu duoc no qua dac tinh thong thap.
     *     Giam tin gia toc ke luc do CHI THEM TRE, khong loi gi.
     *   - Gia toc tuyen tinh DUY TRI (huong tam khi vung thanh, hoac tang toc
     *     thang) moi lam LECH HE THONG goc tinh tu gia toc ke. Day moi la thu
     *     bat buoc phai giam tin.
     * |a|-1 tuc thoi khong phan biet duoc hai truong hop; loc thong thap thi
     * phan biet duoc. */
    float vib_thresh;       /* g. Duoi nguong nay coi gia toc ke dang tin cay */
    float vib_gain;         /* Do doc tang R theo muc nhieu vuot nguong       */
    float vib_R_max_mult;   /* Tran: R khong vuot R_measure * gia tri nay     */
    float vib_reject;       /* g. Vuot nguong nay (TUC THOI) thi bo han
                             * measurement - danh cho va dap / soc thuc su    */
    float vib_tau;          /* s. Hang so thoi gian loc thanh phan nhieu      */

    /* --- baseline complementary de so sanh --- */
    float comp_alpha;       /* Trong so cho nhanh con quay, dien hinh 0.98    */

    /* true  = dung phep bien doi Euler DAY DU (mac dinh, dung).
     * false = dung xap xi p->roll_rate, q->pitch_rate nhu phan lon cac trien
     *         khai tren mang.
     * Ton tai chi de DO duoc cai gia cua xap xi do, thay vi noi suong. Xem
     * TEST 9 trong host_test/test_kalman.c. */
    bool full_euler;

    /* --- phat hien dung yen, phuc vu ZRU cho yaw ---
     *
     * Tieu chi chinh la DO LECH DONG cua toc do goc, khong phai gia tri tuyet
     * doi. Ly do: bias con quay la HANG SO nen no khong lam tang do lech. Neu
     * do bang gia tri tuyet doi thi mot thiet bi CHUA hieu chinh nam yen voi
     * bias 3 deg/s se bi coi la "dang chuyen dong" mai mai -> ZRU khong bao gio
     * chay -> bias truc Z khong bao gio hoc duoc. Vong lap chet.
     *
     * static_rate_gate la chot phu de mot cu quay DEU va NHANH khong bi nham la
     * dung yen (truong hop do lech dong nho vi toc do gan nhu khong doi). */
    float static_dev_thresh;   /* deg/s. Do lech dong toi da khi coi la tinh    */
    float static_rate_gate;    /* deg/s. Chot tho tren toc do da tru bias       */
    float static_tau;          /* s.     Hang so thoi gian cua bo do do lech   */
    float static_acc_thresh;   /* g                                             */
    float static_hold_s;       /* s. Phai yen lien tuc bao lau moi coi la tinh */
    float yaw_R_zru;           /* Phuong sai cua pseudo-measurement ZRU        */
} ahrs_cfg_t;

typedef struct {
    /* --- ket qua chinh: Kalman --- */
    float roll, pitch, yaw;                 /* deg */

    /* --- baseline de so sanh trong bao cao --- */
    float roll_acc,  pitch_acc;             /* chi gia toc ke: nhieu rang cua */
    float roll_gyro, pitch_gyro, yaw_gyro;  /* chi con quay: troi dan         */
    float roll_comp, pitch_comp;            /* complementary filter           */

    /* --- chan doan --- */
    float bias_x, bias_y, bias_z;   /* bias con quay do Kalman uoc luong, deg/s */
    float acc_norm;                 /* |a| tinh theo g. Bang 1.0 khi dung yen   */
    float R_used;                   /* R thuc su dung cho roll/pitch buoc nay   */
    bool  is_static;                /* dang o trang thai dung yen               */
    bool  acc_rejected;             /* da bo measurement vi rung qua manh       */
} ahrs_out_t;

typedef struct {
    ahrs_cfg_t cfg;

    kalman_t k_roll;
    kalman_t k_pitch;
    kalman_t k_yaw;

    /* trang thai cua cac baseline */
    float roll_gyro, pitch_gyro, yaw_gyro;
    float roll_comp, pitch_comp;

    /* trang thai R thich nghi */
    float acc_dist_lp;  /* thanh phan nhieu gia toc da loc thong thap, don vi g */

    /* trang thai phat hien dung yen */
    float gyro_ema[3];  /* trung binh truot cua toc do goc (gom ca bias)     */
    float gyro_dev[3];  /* do lech dong trung binh so voi ema                */
    float static_timer;
    float yaw_anchor;   /* yaw ghi lai luc bat dau dung yen, lam moc cho ZRU */
    bool  is_static;

    bool  seeded;       /* da gan gia tri khoi dau tu mau dau tien chua */
} ahrs_t;

/* Dien cau hinh mac dinh - da tinh chinh cho MPU6050 quay bang tay o 100-200 Hz. */
void ahrs_cfg_default(ahrs_cfg_t *cfg);

/* Khoi tao. cfg == NULL thi dung mac dinh. */
void ahrs_init(ahrs_t *a, const ahrs_cfg_t *cfg);

/* Mot buoc hop nhat. dt la chu ky lay mau THUC TE (s), khong phai hang so.
 * out co the NULL neu khong can doc ket qua chi tiet. */
void ahrs_update(ahrs_t *a, const imu_sample_t *s, float dt, ahrs_out_t *out);

/* --- Ham phu tro, dung rieng duoc va dung trong host_test --- */

/* Goc tuyet doi tu vector trong luc:
 *   roll  = atan2(ay, az)                  -> dai +-180 do
 *   pitch = atan2(-ax, sqrt(ay^2 + az^2))  -> dai +-90 do
 * Luu y: khong phu thuoc yaw - day chinh la ly do yaw khong kha quan sat. */
void  ahrs_accel_angles(const imu_sample_t *s, float *roll_deg, float *pitch_deg);

/* |a| theo don vi g. Lech khoi 1.0 = dang co gia toc tuyen tinh / rung. */
float ahrs_accel_norm(const imu_sample_t *s);

#endif /* AHRS_H */
