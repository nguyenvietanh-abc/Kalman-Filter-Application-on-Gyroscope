/*
 * imu_sim.c - Sinh du lieu IMU gia lap. Xem imu_sim.h cho chuoi suy dien.
 *
 * Toan bo phan quy dao va dao ham tinh o DOUBLE. Ly do: gia toc tiep tuyen can
 * dao ham cap hai cua quy dao goc; lam viec do o float se bi triet tieu chu so
 * (catastrophic cancellation) va cho ra rac.
 */
#include "imu_sim.h"
#include <math.h>

#define DEG2RAD 0.017453292519943295
#define RAD2DEG 57.29577951308232
#define TWO_PI  6.283185307179586
#define G_MS2   9.80665      /* de doi gia toc tuyen tinh tu m/s^2 sang g */

/* ------------------------------------------------------------------------- */
/* PRNG tai lap duoc                                                          */
/* ------------------------------------------------------------------------- */

static uint32_t xorshift32(uint32_t *st)
{
    uint32_t x = *st;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    *st = x;
    return x;
}

/* Uniform trong [0,1). Bo 8 bit thap vi chung yeu nhat o xorshift. */
static float uniform01(imu_sim_t *s)
{
    return (float)(xorshift32(&s->rng) >> 8) * (1.0f / 16777216.0f);
}

/* Gauss(0,1) bang Box-Muller. Sinh 2 mau mot lan nen cache lai mau thu hai. */
static float gauss(imu_sim_t *s)
{
    if (s->have_spare) {
        s->have_spare = false;
        return s->spare;
    }
    float u1 = uniform01(s);
    if (u1 < 1e-7f) {
        u1 = 1e-7f;   /* logf(0) = -inf */
    }
    const float u2  = uniform01(s);
    const float mag = sqrtf(-2.0f * logf(u1));

    s->spare      = mag * sinf((float)TWO_PI * u2);
    s->have_spare = true;
    return mag * cosf((float)TWO_PI * u2);
}

/* ------------------------------------------------------------------------- */
/* Quy dao goc that                                                           */
/* ------------------------------------------------------------------------- */

/* Noi suy smoothstep: muot ca vi tri va dao ham o hai dau doan, nen khong sinh
 * xung toc do goc gia tao (dieu se xay ra neu noi suy tuyen tinh). */
static double seg(double t, double t0, double t1, double delta)
{
    if (t <= t0) return 0.0;
    if (t >= t1) return delta;
    const double u = (t - t0) / (t1 - t0);
    return delta * (u * u * (3.0 - 2.0 * u));
}

static void truth_d(imu_sim_motion_t motion, double t, double rpy[3])
{
    switch (motion) {
    case IMU_SIM_LEVEL:
        rpy[0] = 0.0;
        rpy[1] = 0.0;
        rpy[2] = 0.0;
        break;

    case IMU_SIM_STATIC:
        /* Goc khac 0 co y do - xem ghi chu trong imu_sim.h. */
        rpy[0] = 15.0;
        rpy[1] = -10.0;
        rpy[2] = 0.0;
        break;

    case IMU_SIM_SMOOTH:
        /* Tan so le nhau de ba truc khong bao gio dong pha - neu dong pha thi
         * loi tron truc se bi che lap. */
        rpy[0] = 30.0 * sin(TWO_PI * 0.20 * t);
        rpy[1] = 20.0 * sin(TWO_PI * 0.13 * t + 1.0);
        rpy[2] = 45.0 * sin(TWO_PI * 0.07 * t);
        break;

    case IMU_SIM_YAW_CYCLE: {
        /* Cong don tung chu ky de quy dao lien tuc qua ranh gioi chu ky. Neu
         * dung fmod(t, T) thi yaw se nhay tu 90 ve 0 -> toc do goc vo cuc. */
        const double T = 30.0;
        const int    n = (int)(t / T);
        double yaw = 0.0;
        for (int k = 0; k <= n && k < 1000; ++k) {
            const double t0 = (double)k * T;
            yaw += seg(t, t0 +  1.0, t0 +  3.0,  90.0);
            yaw += seg(t, t0 + 16.0, t0 + 18.0, -90.0);
        }
        rpy[0] = 0.0;
        rpy[1] = 0.0;
        rpy[2] = yaw;
        break;
    }

    case IMU_SIM_EXTREME:
        /* 40 s:
         *   0-2    giu yen
         *   2-6    pitch len +88 do   (cham CT_MIN va chan +-90)
         *   6-9    giu o +88
         *   9-13   pitch ve -88 do
         *   13-16  giu o -88
         *   16-20  pitch ve 0
         *   20-30  roll quay tron 360 do (di qua +-180 hai lan)
         *   30-40  giu yen
         * Roll quay tron dung ham tuyen tinh cong don qua seg de dao ham lien tuc. */
        rpy[0] = seg(t, 20.0, 25.0, 180.0) + seg(t, 25.0, 30.0, 180.0);
        rpy[1] = seg(t, 2.0, 6.0, 88.0) + seg(t, 9.0, 13.0, -176.0)
               + seg(t, 16.0, 20.0, 88.0);
        rpy[2] = 0.0;
        break;

    case IMU_SIM_HANDHELD:
    default:
        /* Kich ban 20 s, cong don tung doan chuyen dong:
         *   0-3 s   giu yen
         *   3-6 s   nghieng cham roll -> +40 deg      (~20 deg/s)
         *   6-8 s   giu yen
         *   8-9 s   LAT NHANH roll -> -30 deg         (~105 deg/s dinh)
         *   9-12 s  giu yen
         *   12-15 s quet pitch -> +35 deg
         *   15-17 s QUAY YAW +90 deg  (dong tac chinh cua de bai)
         *   17-20 s giu yen
         * Cac doan giu yen la co y: do la luc ZRU hoat dong va la luc do drift. */
        rpy[0] = seg(t, 3.0, 6.0, 40.0) + seg(t, 8.0, 9.0, -70.0);
        rpy[1] = seg(t, 12.0, 15.0, 35.0);
        rpy[2] = seg(t, 15.0, 17.0, 90.0);
        break;
    }
}

void imu_sim_truth(imu_sim_motion_t motion, float t, float rpy_deg[3])
{
    double rpy[3];
    truth_d(motion, (double)t, rpy);
    rpy_deg[0] = (float)rpy[0];
    rpy_deg[1] = (float)rpy[1];
    rpy_deg[2] = (float)rpy[2];
}

/* ------------------------------------------------------------------------- */

/* Toc do goc trong he than (rad/s) tai thoi diem t.
 *
 * Kinh hoc nguoc 3-2-1 DAY DU:
 *   p = phi_dot - psi_dot*sin(theta)
 *   q = theta_dot*cos(phi) + psi_dot*cos(theta)*sin(phi)
 *   r = -theta_dot*sin(phi) + psi_dot*cos(theta)*cos(phi)
 *
 * ahrs.c dung dung phep bien doi NGHICH cua cong thuc nay. Vi vay hai ben la
 * nghich dao chinh xac cua nhau, va mot sai dau chung o CA HAI se tu triet tieu
 * qua vong sim -> filter ma khong test RMSE nao bat duoc.
 * Diem mu do duoc bit bang test_convention.c: no neo quy uoc vao vat ly qua mot
 * duong thu ba doc lap (ma tran quay so cap, roi omega tu -R_dot*R^T). */
static void body_rates_d(imu_sim_motion_t m, double t, double pqr[3])
{
    const double h = 1e-3;
    double r0[3], rp[3], rm[3];

    truth_d(m, t, r0);
    truth_d(m, t + h, rp);
    truth_d(m, t - h, rm);

    const double roll_dot  = (rp[0] - rm[0]) / (2.0 * h) * DEG2RAD;   /* rad/s */
    const double pitch_dot = (rp[1] - rm[1]) / (2.0 * h) * DEG2RAD;
    const double yaw_dot   = (rp[2] - rm[2]) / (2.0 * h) * DEG2RAD;

    const double phi   = r0[0] * DEG2RAD;
    const double theta = r0[1] * DEG2RAD;
    const double sp = sin(phi),   cp = cos(phi);
    const double st = sin(theta), ct = cos(theta);

    pqr[0] = roll_dot - yaw_dot * st;
    pqr[1] = pitch_dot * cp + yaw_dot * ct * sp;
    pqr[2] = -pitch_dot * sp + yaw_dot * ct * cp;
}

/* ------------------------------------------------------------------------- */

void imu_sim_cfg_default(imu_sim_cfg_t *cfg)
{
    cfg->dt     = 0.005f;             /* 200 Hz */
    cfg->motion = IMU_SIM_HANDHELD;

    /* Bias khac nhau tren 3 truc va khong tron so - de neu code vo tinh lay
     * bias truc nay ap cho truc khac thi test se phat hien. */
    cfg->gyro_bias[0] = 2.30f;
    cfg->gyro_bias[1] = -1.75f;
    cfg->gyro_bias[2] = 3.10f;

    /* MPU6050 @ DLPF 42 Hz: nhieu con quay khoang 0.4 deg/s RMS,
     * nhieu gia toc ke khoang 4 mg RMS. */
    cfg->gyro_noise_rms  = 0.40f;
    cfg->accel_noise_rms = 0.004f;

    cfg->vib_amp    = 0.35f;   /* rung tay manh co the day |a| lech 0.3-0.4 g */
    cfg->vib_freq   = 11.0f;   /* Hz */
    cfg->vib_enable = true;

    /* Ban kinh quay: IMU nam tren thanh cam tay, cach khop khuyu/vai khoang
     * 30 cm. Dat 0 de tat gia toc huong tam/tiep tuyen. */
    cfg->arm_radius_m = 0.30f;

    cfg->seed = 0x13572468u;
}

void imu_sim_init(imu_sim_t *s, const imu_sim_cfg_t *cfg)
{
    if (cfg) {
        s->cfg = *cfg;
    } else {
        imu_sim_cfg_default(&s->cfg);
    }
    s->rng        = s->cfg.seed ? s->cfg.seed : 1u;   /* xorshift ket voi seed 0 */
    s->t          = 0.0f;
    s->have_spare = false;
    s->spare      = 0.0f;
}

/* ------------------------------------------------------------------------- */

void imu_sim_step(imu_sim_t *s, imu_sample_t *out, float truth_rpy[3])
{
    const imu_sim_cfg_t *c = &s->cfg;
    const double t = (double)s->t;

    /* --- 1. Goc that va toc do goc he than tai t --- */
    double rpy[3], pqr[3];
    truth_d(c->motion, t, rpy);
    body_rates_d(c->motion, t, pqr);

    const double p = pqr[0], q = pqr[1], r = pqr[2];   /* rad/s */

    /* --- 2. Trong luc chieu vao he than (don vi g) ---
     * Dat phang, mat tren huong len: (0,0,+1). Kiem tra nhanh: theta=+90 ->
     * (-1,0,0), va atan2(-ax, hypot) = atan2(1,0) = +90 -> khop cong thuc trong
     * ahrs.c. Luu y: KHONG phu thuoc yaw - day chinh la ly do toan hoc khien
     * yaw khong kha quan sat bang IMU 6 truc. */
    const double phi = rpy[0] * DEG2RAD, theta = rpy[1] * DEG2RAD;
    const double sp = sin(phi), cp = cos(phi);
    const double st = sin(theta), ct = cos(theta);

    double ax = -st;
    double ay = sp * ct;
    double az = cp * ct;

    /* --- 3. Gia toc tuyen tinh do quay thanh cam tay quanh mot khop ---
     * Day la thanh phan QUAN TRONG NHAT de danh gia bo loc, va la thu ma mo
     * hinh "rung hinh sin" khong the hien duoc: no KHONG trung binh bang 0.
     * Gia toc huong tam luon huong ve khop trong suot ca cu quay, nen no lam
     * LECH HE THONG goc tinh tu gia toc ke - khong phai chi lam nhieu.
     *
     * IMU o vi tri rvec = (R, 0, 0) so voi khop quay, trong he than.
     *   a_huong_tam = omega x (omega x rvec) = ( -R(q^2+r^2), R*p*q, R*p*r )
     *   a_tiep_tuyen = alpha x rvec         = ( 0, R*alpha_z, -R*alpha_y )
     *
     * DAU: gia toc ke do SPECIFIC FORCE f = (a_proper - g_field)/|g|.
     *   - Nam yen, dat phang: a_proper = 0, g_field = (0,0,-9.81) trong he Z-huong-len
     *     => f = (0,0,+1). Khop so doc that.
     *   - Phan trong luc -g_field/|g| chieu vao he than DA nam trong ax,ay,az.
     *   - a_proper cua IMU = omega x (omega x r) + alpha x r, vao voi dau CONG.
     * Kiem tra nhanh: quay quanh truc Z than, a_huong_tam theo -X (huong ve khop).
     * Gia toc ke phai doc GIAM tren truc X - dung nhu dau cong cho ra. */
    if (c->arm_radius_m > 0.0f) {
        const double R = (double)c->arm_radius_m;

        /* alpha = domega/dt, sai phan trung tam. Buoc h lon hon buoc dung cho
         * omega de khong khuech dai nhieu so hoc cua dao ham long nhau. */
        const double ha = 5e-3;
        double pqr_p[3], pqr_m[3];
        body_rates_d(c->motion, t + ha, pqr_p);
        body_rates_d(c->motion, t - ha, pqr_m);

        const double alpha_y = (pqr_p[1] - pqr_m[1]) / (2.0 * ha);
        const double alpha_z = (pqr_p[2] - pqr_m[2]) / (2.0 * ha);

        const double lin_x = -R * (q * q + r * r);
        const double lin_y = R * p * q + R * alpha_z;
        const double lin_z = R * p * r - R * alpha_y;

        ax += lin_x / G_MS2;
        ay += lin_y / G_MS2;
        az += lin_z / G_MS2;
    }

    /* --- 3b. Soc gia toc (chi quy dao EXTREME) ---
     * Hai xung ngan bien do lon, mo phong va dap / go vao thanh. Muc dich la dua
     * ||a|-1| vuot 0.5 g de nhanh BO HAN measurement trong ahrs.c thuc su chay.
     * Khong quy dao nao khac dat toi 0.385 g, nen nhanh do chua tung duoc kiem tra. */
    if (c->motion == IMU_SIM_EXTREME) {
        for (int k = 0; k < 2; ++k) {
            const double tc = 32.0 + 3.0 * (double)k;   /* 32 s va 35 s */
            const double dtc = t - tc;
            if (dtc > 0.0 && dtc < 0.15) {
                /* Nua chu ky sin -> len roi ve 0 muot, khong tao buoc nhay. */
                const double env = sin(3.14159265358979 * dtc / 0.15);
                ax += 1.4 * env;
                az += 0.9 * env;
            }
        }
    }

    /* --- 4. Rung tay dao dong ---
     * Bien do ty le voi do lon toc do goc: quay tay nhanh thi rung manh. Day la
     * thanh phan TRUNG BINH BANG 0 - no lam nhieu phep do nhung khong lam lech
     * he thong, khac han thanh phan o buoc 3. */
    if (c->vib_enable) {
        const double rate_mag = sqrt(p * p + q * q + r * r) * RAD2DEG;
        double env = rate_mag / 100.0;          /* day du 100 deg/s -> bien do max */
        if (env > 1.0) env = 1.0;

        const double w = TWO_PI * (double)c->vib_freq * t;
        /* Lech pha 3 truc de rung khong thanh mot huong duy nhat. */
        ax += (double)c->vib_amp * env * sin(w);
        ay += (double)c->vib_amp * env * sin(w + 2.0944);   /* +120 do */
        az += (double)c->vib_amp * env * sin(w + 4.1888);   /* +240 do */
    }

    /* --- 5. Cong bias + nhieu, xuat mau "nhu cam bien that doc duoc" --- */
    out->ax = (float)ax + c->accel_noise_rms * gauss(s);
    out->ay = (float)ay + c->accel_noise_rms * gauss(s);
    out->az = (float)az + c->accel_noise_rms * gauss(s);

    out->gx = (float)(p * RAD2DEG) + c->gyro_bias[0] + c->gyro_noise_rms * gauss(s);
    out->gy = (float)(q * RAD2DEG) + c->gyro_bias[1] + c->gyro_noise_rms * gauss(s);
    out->gz = (float)(r * RAD2DEG) + c->gyro_bias[2] + c->gyro_noise_rms * gauss(s);

    if (truth_rpy) {
        truth_rpy[0] = (float)rpy[0];
        truth_rpy[1] = (float)rpy[1];
        truth_rpy[2] = (float)rpy[2];
    }

    s->t += c->dt;
}
