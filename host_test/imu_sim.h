/*
 * imu_sim.h - Sinh du lieu IMU gia lap CO GROUND TRUTH.
 *
 * Ly do ton tai: chua co phan cung. Khong co ground truth thi khong the noi
 * "bo loc chinh xac bao nhieu do" - chi co the noi "duong nhin muot". Simulator
 * nay dung quy trinh nguoc: dinh nghia quy dao goc THAT truoc, roi tu do suy ra
 * dung nhung gi cam bien that se doc duoc, ke ca cac loi cua no.
 *
 * Chuoi suy dien (dung chieu vat ly, khong bat chuoc so lieu):
 *   1. Quy dao Euler that:      roll(t), pitch(t), yaw(t)
 *   2. Dao ham so (sai phan trung tam) -> toc do Euler
 *   3. Kinh hoc nguoc 3-2-1     -> toc do goc trong he than p, q, r
 *   4. Vector trong luc chieu vao he than -> gia toc ly tuong
 *   5. Cong loi cua cam bien that:
 *        - bias con quay (nguyen nhan chinh gay drift)
 *        - nhieu trang Gauss tren ca hai cam bien
 *        - gia toc tuyen tinh do rung tay (thu lam sai goc gia toc ke)
 *
 * Tinh tai lap: PRNG xorshift32 voi seed co dinh. Chay lai cho ket qua y het,
 * nen bang RMSE trong bao cao la con so kiem chung duoc, khong phai ngau nhien.
 */
#ifndef IMU_SIM_H
#define IMU_SIM_H

#include <stdint.h>
#include <stdbool.h>
#include "ahrs.h"   /* dung lai imu_sample_t */

typedef enum {
    /* Dat phang tuyet doi (0,0,0) - dung mo phong CHINH XAC dieu kien can khi
     * hieu chinh offset: thiet bi nam yen tren mat phang. */
    IMU_SIM_LEVEL = 0,

    /* Dung yen o mot goc nghieng khac 0. Dung do DRIFT va do sai so tinh.
     * Chon goc khac 0 co y do: neu de (0,0,0) thi nhieu loi cai dat se bi che
     * lap vi moi thu deu bang 0. */
    IMU_SIM_STATIC,

    /* Ba hinh sin le nhau. Quy dao tron, dao ham lien tuc - dung kiem tra
     * do chinh xac co ban khi dang chuyen dong. */
    IMU_SIM_SMOOTH,

    /* Mo phong dung de bai: quay thanh cam tay bang tay.
     * Gom giu yen - nghieng cham - lat nhanh - giu - quet pitch - quay yaw 90 do.
     * Co ca doan toc do goc cao (~70 deg/s) de bat loi saturate va loi dt. */
    IMU_SIM_HANDHELD,

    /* Chu ky 30 s lap lai: quay yaw +90 do trong 2 s, NGHI 13 s, quay -90 do
     * trong 2 s, NGHI 13 s. Yaw that quay ve dung 0 sau moi chu ky, nen do
     * drift chi can so sanh voi 0.
     *
     * Vi sao can quy dao nay: do drift khi thiet bi dung yen SUOT thoi gian la
     * phep do vo nghia - ZRU ghim yaw vao moc nen ket qua tat nhien bang 0.
     * Cau hoi thuc te la "vua quay vua nghi nhu nguoi dung that thi troi bao
     * nhieu", va chi quy dao xen ke moi tra loi duoc. */
    IMU_SIM_YAW_CYCLE,

    /* Quy dao BIEN: co tinh dua bo loc vao dung nhung duong code ma cac quy dao
     * kia khong bao gio cham toi.
     *   - pitch len +-88 do  -> kich hoat chan CT_MIN (>84.3 do) va chan +-90
     *   - roll quay tron 360 do -> kich hoat toan bo cac nhanh wrap180
     *   - mot cu soc gia toc  -> kich hoat nhanh bo han measurement (>0.5 g)
     *
     * Ly do ton tai: do coverage tren du lieu that cho thay max|pitch| chi 37 do,
     * max|roll| 40 do, max||a|-1| 0.385 g tren MOI quy dao khac. Tuc la ba co che
     * phong ve quan trong nhat cua bo loc chua tung duoc chay mot lan nao, ke ca
     * hai co che vua duoc sua. Test xanh ma khong chay qua code thi khong chung
     * minh duoc gi ve code do. */
    IMU_SIM_EXTREME,
} imu_sim_motion_t;

typedef struct {
    float dt;                 /* chu ky lay mau, s (vd 0.005 = 200 Hz) */
    imu_sim_motion_t motion;

    /* --- loi cua con quay --- */
    float gyro_bias[3];       /* deg/s. MPU6050 that thuong lech 1-5 deg/s */
    float gyro_noise_rms;     /* deg/s */

    /* --- loi cua gia toc ke --- */
    float accel_noise_rms;    /* g */

    /* --- rung dao dong do thao tac bang tay ---
     * Bien do ty le voi toc do goc: quay tay nhanh thi rung manh.
     * Thanh phan nay TRUNG BINH BANG 0 -> lam nhieu phep do nhung khong lam
     * lech he thong. */
    float vib_amp;            /* g, bien do rung khi o muc toi da */
    float vib_freq;           /* Hz, tan so rung tay (dien hinh 8-15 Hz) */
    bool  vib_enable;

    /* --- gia toc huong tam + tiep tuyen do quay quanh khop ---
     * Ban kinh tu khop quay (khuyu/vai) den IMU tren thanh cam tay, don vi m.
     * Thanh phan nay KHONG trung binh bang 0: gia toc huong tam luon huong ve
     * khop trong suot ca cu quay, nen no lam LECH HE THONG goc tinh tu gia toc
     * ke. Day moi la loai nhieu ma R thich nghi sinh ra de chong; mo hinh rung
     * hinh sin o tren tu triet tieu qua dac tinh thong thap cua bo loc nen
     * khong kiem tra duoc co che do.
     * Dat 0 de tat. */
    float arm_radius_m;

    uint32_t seed;
} imu_sim_cfg_t;

typedef struct {
    imu_sim_cfg_t cfg;
    uint32_t rng;
    float    t;            /* thoi gian mo phong hien tai, s */
    bool     have_spare;   /* cache cho Box-Muller (sinh 2 mau moi lan) */
    float    spare;
} imu_sim_t;

/* Cau hinh mac dinh: 200 Hz, quy dao HANDHELD, muc nhieu lay theo datasheet
 * MPU6050 va do rung tay thuc te. */
void imu_sim_cfg_default(imu_sim_cfg_t *cfg);

void imu_sim_init(imu_sim_t *s, const imu_sim_cfg_t *cfg);

/* Sinh mau ke tiep va tang thoi gian len dt.
 *   out       : mau IMU "nhu doc duoc tu cam bien" (da co day du loi)
 *   truth_rpy : goc THAT tai thoi diem do, deg. Co the NULL. */
void imu_sim_step(imu_sim_t *s, imu_sample_t *out, float truth_rpy[3]);

/* Quy dao goc that tai thoi diem t. Tach rieng de test co the truy van truc
 * tiep, va de kiem tra chinh cong thuc quy dao. */
void imu_sim_truth(imu_sim_motion_t motion, float t, float rpy_deg[3]);

static inline float imu_sim_time(const imu_sim_t *s) { return s->t; }

#endif /* IMU_SIM_H */
