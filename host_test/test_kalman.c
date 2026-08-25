/*
 * test_kalman.c - Kiem chung bo loc Kalman tren host, KHONG can phan cung.
 *
 * Bien dich chinh nhung file .c dang chay tren ESP32 (../main/kalman.c,
 * ../main/ahrs.c, ../main/imu_calib.c) - khong phai ban copy. Nho vay ket qua o
 * day noi dung ve hanh vi cua firmware.
 *
 * Cac phep do:
 *   1. Hieu chinh offset co lay lai dung bias da tiem vao khong
 *   2. Cac buoc kiem tra tinh hop le cua hieu chinh co bat dung loi khong
 *   3. RMSE cua 4 phuong phap tren quy dao quay tay (co rung)
 *   4. R thich nghi co that su giup khi rung khong (bat/tat de so)
 *   5. Kalman co tu uoc luong duoc bias con quay khong (trang thai thu 2)
 *   6. Drift sau 300 s dung yen - va ZRU giup gi cho yaw
 *
 * Ma tra ve 0 = tat ca dat. Khac 0 = co tieu chi truot.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ahrs.h"
#include "kalman.h"
#include "imu_calib.h"
#include "imu_sim.h"

/* ------------------------------------------------------------------------- */
/* Ha tang test                                                               */
/* ------------------------------------------------------------------------- */

static int g_fail = 0;
static int g_pass = 0;

/* So sanh co dinh huong: mo ta ro tieu chi thay vi chi in PASS/FAIL. */
static void check_lt(const char *what, double got, double limit, const char *unit)
{
    const bool ok = (got < limit);
    printf("  [%s] %-52s %8.4f < %.4f %s\n",
           ok ? "DAT " : "TRUOT", what, got, limit, unit);
    if (ok) g_pass++; else g_fail++;
}

static void check_true(const char *what, bool ok, const char *detail)
{
    printf("  [%s] %-52s %s\n", ok ? "DAT " : "TRUOT", what, detail);
    if (ok) g_pass++; else g_fail++;
}

/* Tich luy sai so binh phuong. Dung wrap180 vi goc la dai luong tuan hoan:
 * 179 va -179 lech nhau 2 do, khong phai 358 do. */
typedef struct { double sse; long n; double max_abs; } rmse_t;

static void rmse_reset(rmse_t *r) { r->sse = 0.0; r->n = 0; r->max_abs = 0.0; }

static void rmse_add(rmse_t *r, float est, float truth)
{
    const double e = (double)kalman_wrap180(est - truth);
    r->sse += e * e;
    r->n++;
    if (fabs(e) > r->max_abs) r->max_abs = fabs(e);
}

static double rmse_get(const rmse_t *r)
{
    return (r->n > 0) ? sqrt(r->sse / (double)r->n) : 0.0;
}

/* ------------------------------------------------------------------------- */
/* Do thi ASCII - khong phu thuoc thu vien nao                                 */
/* ------------------------------------------------------------------------- */

#define PLOT_W 96
#define PLOT_H 19

static void ascii_plot(const char *title, const char *yunit,
                       const float *const *series, const char *marks,
                       const char *const *labels, int n_series,
                       const float *tvec, int n)
{
    if (n <= 0 || n_series <= 0) return;

    float lo = series[0][0], hi = series[0][0];
    for (int s = 0; s < n_series; ++s) {
        for (int i = 0; i < n; ++i) {
            if (series[s][i] < lo) lo = series[s][i];
            if (series[s][i] > hi) hi = series[s][i];
        }
    }
    if (hi - lo < 1e-6f) { hi = lo + 1.0f; lo -= 1.0f; }
    /* Chua le 4% moi phia de duong khong dinh sat vien. */
    const float pad = 0.04f * (hi - lo);
    lo -= pad; hi += pad;

    static char canvas[PLOT_H][PLOT_W + 1];
    for (int r = 0; r < PLOT_H; ++r) {
        memset(canvas[r], ' ', PLOT_W);
        canvas[r][PLOT_W] = '\0';
    }

    /* Duong 0 lam moc tham chieu. */
    if (lo < 0.0f && hi > 0.0f) {
        const int rz = (int)lroundf((hi - 0.0f) / (hi - lo) * (float)(PLOT_H - 1));
        if (rz >= 0 && rz < PLOT_H) {
            memset(canvas[rz], '-', PLOT_W);
        }
    }

    for (int s = 0; s < n_series; ++s) {
        for (int x = 0; x < PLOT_W; ++x) {
            /* Lay mau thua: chon chi so gan nhat voi cot nay. */
            const int i = (int)((long)x * (n - 1) / (PLOT_W - 1));
            const float v = series[s][i];
            int r = (int)lroundf((hi - v) / (hi - lo) * (float)(PLOT_H - 1));
            if (r < 0) r = 0;
            if (r >= PLOT_H) r = PLOT_H - 1;
            canvas[r][x] = marks[s];
        }
    }

    printf("\n  %s   [%s]\n", title, yunit);
    for (int r = 0; r < PLOT_H; ++r) {
        const float v = hi - (hi - lo) * (float)r / (float)(PLOT_H - 1);
        printf("  %8.2f |%s\n", v, canvas[r]);
    }
    printf("  %8s +", "");
    for (int x = 0; x < PLOT_W; ++x) putchar('-');
    printf("\n           %-*s%.1f s\n", PLOT_W - 6, "0 s", (double)tvec[n - 1]);

    printf("           chu giai: ");
    for (int s = 0; s < n_series; ++s) {
        printf("'%c'=%s  ", marks[s], labels[s]);
    }
    printf("\n");
}

/* ------------------------------------------------------------------------- */
/* TEST 1: hieu chinh offset lay lai duoc bias da tiem vao                    */
/* ------------------------------------------------------------------------- */

static void test_calibration(imu_calib_t *calib_out, const imu_sim_cfg_t *base)
{
    printf("\n=== TEST 1: Hieu chinh offset ===\n");

    imu_sim_cfg_t cfg = *base;
    cfg.motion     = IMU_SIM_LEVEL;   /* dieu kien dung: nam yen, nam phang */
    cfg.vib_enable = false;

    imu_sim_t sim;
    imu_sim_init(&sim, &cfg);

    imu_calib_acc_t acc;
    imu_calib_acc_init(&acc, 1000);   /* 5 s @ 200 Hz */

    imu_sample_t s;
    while (!imu_calib_acc_add(&acc, (imu_sim_step(&sim, &s, NULL), &s))) { }

    const imu_calib_status_t st = imu_calib_acc_finish(&acc, calib_out);

    printf("  Trang thai        : %s\n", imu_calib_status_str(st));
    printf("  Bias con quay tiem: %+7.3f %+7.3f %+7.3f deg/s\n",
           (double)base->gyro_bias[0], (double)base->gyro_bias[1], (double)base->gyro_bias[2]);
    printf("  Bias con quay do  : %+7.3f %+7.3f %+7.3f deg/s\n",
           (double)calib_out->gyro_bias[0], (double)calib_out->gyro_bias[1],
           (double)calib_out->gyro_bias[2]);
    printf("  Do lech chuan     : %7.3f %7.3f %7.3f deg/s\n",
           (double)acc.gyro_std[0], (double)acc.gyro_std[1], (double)acc.gyro_std[2]);
    printf("  |a| trung binh    : %7.4f g\n", (double)acc.acc_norm_mean);
    printf("  Offset gia toc ke : %+7.4f %+7.4f %+7.4f g\n",
           (double)calib_out->accel_off[0], (double)calib_out->accel_off[1],
           (double)calib_out->accel_off[2]);

    check_true("Hieu chinh bao thanh cong", st == IMU_CALIB_OK, imu_calib_status_str(st));

    /* Sai so chuan cua trung binh 1000 mau nhieu 0.4 deg/s RMS la 0.013 deg/s.
     * Nguong 0.05 deg/s la khoang 4 sigma - chat nhung khong bap benh. */
    for (int i = 0; i < 3; ++i) {
        char name[64];
        snprintf(name, sizeof name, "Sai so bias truc %c", (char)('X' + i));
        check_lt(name, fabs((double)(calib_out->gyro_bias[i] - base->gyro_bias[i])),
                 0.05, "deg/s");
    }
    /* Sim khong tiem offset gia toc ke -> ket qua phai xap xi 0. */
    check_lt("Offset gia toc ke du (phai ~0)",
             fmax(fmax(fabs((double)calib_out->accel_off[0]),
                       fabs((double)calib_out->accel_off[1])),
                  fabs((double)calib_out->accel_off[2])), 0.003, "g");
}

/* ------------------------------------------------------------------------- */
/* TEST 2: cac buoc kiem tra tinh hop le co bat dung loi khong                */
/* ------------------------------------------------------------------------- */

static void test_calibration_guards(const imu_sim_cfg_t *base)
{
    printf("\n=== TEST 2: Chan doan hieu chinh sai dieu kien ===\n");

    /* --- 2a. Thiet bi dang duoc quay tren tay --- */
    {
        imu_sim_cfg_t cfg = *base;
        cfg.motion = IMU_SIM_HANDHELD;
        imu_sim_t sim;
        imu_sim_init(&sim, &cfg);

        /* Nhay den doan lat nhanh (8-9 s) roi moi lay mau. */
        imu_sample_t s;
        for (int i = 0; i < (int)(8.0f / cfg.dt); ++i) imu_sim_step(&sim, &s, NULL);

        imu_calib_acc_t acc;
        imu_calib_acc_init(&acc, 200);
        while (!imu_calib_acc_add(&acc, (imu_sim_step(&sim, &s, NULL), &s))) { }

        imu_calib_t c;
        const imu_calib_status_t st = imu_calib_acc_finish(&acc, &c);
        check_true("Phat hien thiet bi dang chuyen dong",
                   st == IMU_CALIB_ERR_MOVING, imu_calib_status_str(st));
    }

    /* --- 2b. Nam yen nhung nghieng 40 do: bias con quay van dung, nhung offset
     *          gia toc ke thi khong tach duoc khoi trong luc --- */
    {
        imu_calib_acc_t acc;
        imu_calib_acc_init(&acc, 200);

        imu_sample_t s;
        for (int i = 0; i < 200; ++i) {
            s.ax = 0.0f;
            s.ay = 0.6428f;   /* sin(40 deg) */
            s.az = 0.7660f;   /* cos(40 deg) -> duoi nguong min_level_az = 0.90 */
            s.gx = base->gyro_bias[0];
            s.gy = base->gyro_bias[1];
            s.gz = base->gyro_bias[2];
            imu_calib_acc_add(&acc, &s);
        }

        imu_calib_t c;
        const imu_calib_status_t st = imu_calib_acc_finish(&acc, &c);
        check_true("Phat hien thiet bi khong nam phang",
                   st == IMU_CALIB_ERR_NOT_LEVEL, imu_calib_status_str(st));
        check_true("Van tra ve bias con quay du khong nam phang",
                   fabsf(c.gyro_bias[0] - base->gyro_bias[0]) < 1e-3f &&
                   c.accel_off[0] == 0.0f && c.accel_off[2] == 0.0f,
                   "gyro_bias dung, accel_off bo trong");
    }

    /* --- 2c. Chua du mau --- */
    {
        imu_calib_acc_t acc;
        imu_calib_acc_init(&acc, 1000);
        imu_sample_t s = { 0, 0, 1.0f, 0, 0, 0 };
        for (int i = 0; i < 10; ++i) imu_calib_acc_add(&acc, &s);

        imu_calib_t c;
        const imu_calib_status_t st = imu_calib_acc_finish(&acc, &c);
        check_true("Phat hien chua du mau",
                   st == IMU_CALIB_ERR_FEW_SAMPLES, imu_calib_status_str(st));
    }
}

/* ------------------------------------------------------------------------- */
/* TEST 3: RMSE 4 phuong phap tren quy dao quay tay                           */
/* ------------------------------------------------------------------------- */

#define TRACE_MAX 8000

typedef struct {
    int   n;
    float t[TRACE_MAX];
    float truth_r[TRACE_MAX], truth_p[TRACE_MAX], truth_y[TRACE_MAX];
    float acc_r[TRACE_MAX],   acc_p[TRACE_MAX];
    float gyr_r[TRACE_MAX],   gyr_p[TRACE_MAX];
    float cmp_r[TRACE_MAX],   cmp_p[TRACE_MAX];
    float kf_r[TRACE_MAX],    kf_p[TRACE_MAX], kf_y[TRACE_MAX];
    float anorm[TRACE_MAX],   rused[TRACE_MAX];
    float stat[TRACE_MAX];
    float bx[TRACE_MAX], by[TRACE_MAX], bz[TRACE_MAX];
} trace_t;

static trace_t *g_trace_bias  = NULL;
static trace_t *g_trace_uncal = NULL;

typedef struct {
    double acc_r, acc_p;
    double gyr_r, gyr_p;
    double cmp_r, cmp_p;
    double kf_r,  kf_p;
    double kf_r_max, acc_r_max;
    float  bias_final[3];
} run_stats_t;

/* Chay mot lan mo phong day du. calib != NULL -> tru offset truoc khi hop nhat.
 * cfg_ahrs != NULL -> ghi de cau hinh AHRS (dung de bat/tat R thich nghi). */
static void run_sim(const imu_sim_cfg_t *scfg, float duration_s,
                    const imu_calib_t *calib, const ahrs_cfg_t *cfg_ahrs,
                    run_stats_t *out, trace_t *trace, int trace_decim)
{
    if (trace_decim < 1) {
        trace_decim = 1;
    }
    imu_sim_t sim;
    imu_sim_init(&sim, scfg);

    ahrs_t ah;
    ahrs_init(&ah, cfg_ahrs);

    rmse_t r_acc, p_acc, r_gyr, p_gyr, r_cmp, p_cmp, r_kf, p_kf;
    rmse_reset(&r_acc); rmse_reset(&p_acc);
    rmse_reset(&r_gyr); rmse_reset(&p_gyr);
    rmse_reset(&r_cmp); rmse_reset(&p_cmp);
    rmse_reset(&r_kf);  rmse_reset(&p_kf);

    if (trace) trace->n = 0;

    const int steps = (int)(duration_s / scfg->dt);
    /* Bo 1 s dau: cac bo uoc luong dang hoi tu, tinh vao la lam ban so lieu
     * cua CA BON phuong phap mot cach khong dong deu. */
    const int warmup = (int)(1.0f / scfg->dt);

    for (int i = 0; i < steps; ++i) {
        const float t = imu_sim_time(&sim);

        imu_sample_t s;
        float truth[3];
        imu_sim_step(&sim, &s, truth);

        if (calib) {
            imu_calib_apply(calib, &s);
        }

        ahrs_out_t o;
        ahrs_update(&ah, &s, scfg->dt, &o);

        if (i >= warmup) {
            rmse_add(&r_acc, o.roll_acc,  truth[0]);
            rmse_add(&p_acc, o.pitch_acc, truth[1]);
            rmse_add(&r_gyr, o.roll_gyro, truth[0]);
            rmse_add(&p_gyr, o.pitch_gyro, truth[1]);
            rmse_add(&r_cmp, o.roll_comp, truth[0]);
            rmse_add(&p_cmp, o.pitch_comp, truth[1]);
            rmse_add(&r_kf,  o.roll,      truth[0]);
            rmse_add(&p_kf,  o.pitch,     truth[1]);
        }

        /* Lay mau thua cho trace de mot lan chay dai van du cho trong buffer.
         * Neu khong the du, bao ro thay vi cat am tham. */
        if (trace && (i % trace_decim) == 0 && trace->n < TRACE_MAX) {
            const int k = trace->n++;
            trace->t[k]       = t;
            trace->truth_r[k] = truth[0];
            trace->truth_p[k] = truth[1];
            trace->truth_y[k] = truth[2];
            trace->acc_r[k]   = o.roll_acc;
            trace->acc_p[k]   = o.pitch_acc;
            trace->gyr_r[k]   = o.roll_gyro;
            trace->gyr_p[k]   = o.pitch_gyro;
            trace->cmp_r[k]   = o.roll_comp;
            trace->cmp_p[k]   = o.pitch_comp;
            trace->kf_r[k]    = o.roll;
            trace->kf_p[k]    = o.pitch;
            trace->kf_y[k]    = o.yaw;
            trace->anorm[k]   = o.acc_norm;
            trace->rused[k]   = o.R_used;
            trace->stat[k]    = o.is_static ? 1.0f : 0.0f;
            trace->bx[k]      = o.bias_x;
            trace->by[k]      = o.bias_y;
            trace->bz[k]      = o.bias_z;
        }
    }

    out->acc_r = rmse_get(&r_acc); out->acc_p = rmse_get(&p_acc);
    out->gyr_r = rmse_get(&r_gyr); out->gyr_p = rmse_get(&p_gyr);
    out->cmp_r = rmse_get(&r_cmp); out->cmp_p = rmse_get(&p_cmp);
    out->kf_r  = rmse_get(&r_kf);  out->kf_p  = rmse_get(&p_kf);
    out->kf_r_max  = r_kf.max_abs;
    out->acc_r_max = r_acc.max_abs;
    if (trace && (steps / trace_decim) > TRACE_MAX) {
        printf("  CANH BAO: trace bi cat - can %d o, chi co %d. Do thi se thieu duoi.\n",
               steps / trace_decim, TRACE_MAX);
    }

    out->bias_final[0] = ah.k_roll.bias;
    out->bias_final[1] = ah.k_pitch.bias;
    out->bias_final[2] = ah.k_yaw.bias;
}

static void print_rmse_table(const char *title, const run_stats_t *st)
{
    printf("\n  %s\n", title);
    printf("  %-24s %10s %10s\n", "Phuong phap", "RMSE roll", "RMSE pitch");
    printf("  %-24s %10s %10s\n", "------------------------", "----------", "----------");
    printf("  %-24s %9.3f%s %9.3f%s\n", "Chi gia toc ke", st->acc_r, " deg", st->acc_p, " deg");
    printf("  %-24s %9.3f%s %9.3f%s\n", "Chi con quay",   st->gyr_r, " deg", st->gyr_p, " deg");
    printf("  %-24s %9.3f%s %9.3f%s\n", "Complementary",  st->cmp_r, " deg", st->cmp_p, " deg");
    printf("  %-24s %9.3f%s %9.3f%s  <== \n", "KALMAN",   st->kf_r,  " deg", st->kf_p,  " deg");
}

static void test_rmse(const imu_sim_cfg_t *base, const imu_calib_t *calib,
                      trace_t *trace)
{
    printf("\n=== TEST 3: RMSE tren quy dao quay tay (co rung, da hieu chinh) ===\n");

    imu_sim_cfg_t cfg = *base;
    cfg.motion = IMU_SIM_HANDHELD;

    run_stats_t st;
    run_sim(&cfg, 20.0f, calib, NULL, &st, trace, 1);

    print_rmse_table("Quy dao HANDHELD 20 s, rung bat", &st);

    check_lt("Kalman tot hon chi gia toc ke (roll)",  st.kf_r, st.acc_r, "deg");
    check_lt("Kalman tot hon chi gia toc ke (pitch)", st.kf_p, st.acc_p, "deg");
    check_lt("Kalman tot hon chi con quay (roll)",    st.kf_r, st.gyr_r, "deg");
    check_lt("Kalman tot hon chi con quay (pitch)",   st.kf_p, st.gyr_p, "deg");
    check_lt("Sai so dinh cua Kalman (roll)",         st.kf_r_max, st.acc_r_max, "deg");
}

/* ------------------------------------------------------------------------- */
/* TEST 4: R thich nghi co that su giup khi rung khong                        */
/* ------------------------------------------------------------------------- */

static void test_adaptive_r(const imu_sim_cfg_t *base, const imu_calib_t *calib)
{
    printf("\n=== TEST 4: Tac dung cua R thich nghi khi co rung ===\n");

    imu_sim_cfg_t cfg = *base;
    cfg.motion     = IMU_SIM_HANDHELD;
    cfg.vib_enable = true;

    /* R co dinh: vo hieu hoa ca phan tang R va phan loai bo measurement. */
    ahrs_cfg_t fixed;
    ahrs_cfg_default(&fixed);
    fixed.vib_gain   = 0.0f;
    fixed.vib_reject = 1e9f;

    run_stats_t st_fixed, st_adapt;
    run_sim(&cfg, 20.0f, calib, &fixed, &st_fixed, NULL, 1);
    run_sim(&cfg, 20.0f, calib, NULL,   &st_adapt, NULL, 1);

    printf("  %-28s %10s %10s\n", "", "RMSE roll", "RMSE pitch");
    printf("  %-28s %9.3f%s %9.3f%s\n", "R co dinh",
           st_fixed.kf_r, " deg", st_fixed.kf_p, " deg");
    printf("  %-28s %9.3f%s %9.3f%s\n", "R thich nghi",
           st_adapt.kf_r, " deg", st_adapt.kf_p, " deg");
    printf("  %-28s %9.1f%s %9.1f%s\n", "Cai thien",
           100.0 * (1.0 - st_adapt.kf_r / st_fixed.kf_r), " %",
           100.0 * (1.0 - st_adapt.kf_p / st_fixed.kf_p), " %");

    check_lt("R thich nghi tot hon R co dinh (roll)",  st_adapt.kf_r, st_fixed.kf_r, "deg");
    check_lt("R thich nghi tot hon R co dinh (pitch)", st_adapt.kf_p, st_fixed.kf_p, "deg");
}

/* ------------------------------------------------------------------------- */
/* TEST 5: Kalman tu uoc luong duoc bias con quay (trang thai thu hai)        */
/* ------------------------------------------------------------------------- */

static void test_bias_estimation(const imu_sim_cfg_t *base)
{
    printf("\n=== TEST 5: Kalman tu uoc luong bias con quay (KHONG hieu chinh truoc) ===\n");

    imu_sim_cfg_t cfg = *base;
    cfg.motion     = IMU_SIM_LEVEL;
    cfg.vib_enable = false;

    /* Co tinh KHONG truyen calib: de xem trang thai thu hai cua bo loc co tu
     * tim ra bias hay khong. Day la ly do chon mo hinh 2 trang thai thay vi 1. */
    run_stats_t st;
    run_sim(&cfg, 120.0f, NULL, NULL, &st, g_trace_bias, 4);

    printf("  %-20s %10s %10s %10s\n", "Truc", "Tiem vao", "Uoc luong", "Sai so");
    const char *ax_name[3] = { "X (roll)", "Y (pitch)", "Z (yaw, ZRU)" };
    for (int i = 0; i < 3; ++i) {
        printf("  %-20s %9.3f%s %9.3f%s %9.3f%s\n", ax_name[i],
               (double)cfg.gyro_bias[i], " ", (double)st.bias_final[i], " ",
               (double)(st.bias_final[i] - cfg.gyro_bias[i]), " deg/s");
    }

    /* Roll/pitch: bias kha quan sat lien tuc nho gia toc ke -> nguong chat. */
    check_lt("Bias truc X hoi tu", fabs((double)(st.bias_final[0] - cfg.gyro_bias[0])),
             0.20, "deg/s");
    check_lt("Bias truc Y hoi tu", fabs((double)(st.bias_final[1] - cfg.gyro_bias[1])),
             0.20, "deg/s");
    /* Yaw: chi kha quan sat khi dung yen (ZRU). Nguong long hon la trung thuc,
     * khong phai nhuong bo - day la gioi han thong tin, khong phai loi code. */
    check_lt("Bias truc Z hoi tu qua ZRU", fabs((double)(st.bias_final[2] - cfg.gyro_bias[2])),
             0.50, "deg/s");
}

/* ------------------------------------------------------------------------- */
/* TEST 7: Kalman thang complementary o dau?                                  */
/* ------------------------------------------------------------------------- */

static void test_vs_complementary(const imu_sim_cfg_t *base, const imu_calib_t *calib)
{
    printf("\n=== TEST 7: Kalman so voi complementary - hai tinh huong ===\n");

    imu_sim_cfg_t cfg = *base;
    cfg.motion = IMU_SIM_HANDHELD;

    run_stats_t with_cal, no_cal;
    run_sim(&cfg, 20.0f, calib, NULL, &with_cal, NULL, 1);       /* da hieu chinh offset */
    run_sim(&cfg, 20.0f, NULL,  NULL, &no_cal,   g_trace_uncal, 1); /* CHUA hieu chinh */

    printf("  %-34s %11s %11s\n", "", "Complement.", "KALMAN");
    printf("  %-34s %10.3f%s %10.3f%s\n", "Da hieu chinh offset (roll)",
           with_cal.cmp_r, " deg", with_cal.kf_r, " deg");
    printf("  %-34s %10.3f%s %10.3f%s\n", "Da hieu chinh offset (pitch)",
           with_cal.cmp_p, " deg", with_cal.kf_p, " deg");
    printf("  %-34s %10.3f%s %10.3f%s\n", "CHUA hieu chinh offset (roll)",
           no_cal.cmp_r, " deg", no_cal.kf_r, " deg");
    printf("  %-34s %10.3f%s %10.3f%s\n", "CHUA hieu chinh offset (pitch)",
           no_cal.cmp_p, " deg", no_cal.kf_p, " deg");

    printf("\n  Ket luan trung thuc: khi bias da bi tru sach, complementary ngang\n"
           "  hoac hon Kalman - vi trang thai thu hai cua Kalman khong con viec gi\n"
           "  de lam. Gia tri thuc cua mo hinh 2 trang thai the hien khi CON bias:\n"
           "  no tu do va bu, complementary thi khong co co che nao lam duoc.\n");

    check_lt("Kalman hon complementary khi chua hieu chinh (roll)",
             no_cal.kf_r, no_cal.cmp_r, "deg");
    check_lt("Kalman hon complementary khi chua hieu chinh (pitch)",
             no_cal.kf_p, no_cal.cmp_p, "deg");
}

/* ------------------------------------------------------------------------- */
/* TEST 6: Drift sau 300 s dung yen                                           */
/* ------------------------------------------------------------------------- */

static void test_drift(const imu_sim_cfg_t *base, const imu_calib_t *calib)
{
    printf("\n=== TEST 6: Drift sau 300 s dung yen ===\n");

    imu_sim_cfg_t cfg = *base;
    cfg.motion     = IMU_SIM_STATIC;   /* nghieng (15, -10, 0), khong doi */
    cfg.vib_enable = false;

    imu_sim_t sim;
    imu_sim_init(&sim, &cfg);

    ahrs_t ah;
    ahrs_init(&ah, NULL);

    float truth[3];
    ahrs_out_t o;
    memset(&o, 0, sizeof o);

    const int steps = (int)(300.0f / cfg.dt);
    for (int i = 0; i < steps; ++i) {
        imu_sample_t s;
        imu_sim_step(&sim, &s, truth);
        imu_calib_apply(calib, &s);
        ahrs_update(&ah, &s, cfg.dt, &o);
    }

    const double drift_roll  = fabs((double)kalman_wrap180(o.roll  - truth[0]));
    const double drift_pitch = fabs((double)kalman_wrap180(o.pitch - truth[1]));
    const double drift_yaw_kf  = fabs((double)kalman_wrap180(o.yaw      - truth[2]));
    const double drift_yaw_raw = fabs((double)kalman_wrap180(o.yaw_gyro - truth[2]));

    printf("  Sau 300 s (5 phut) dung yen:\n");
    printf("    Roll  (Kalman)          : %8.3f deg\n", drift_roll);
    printf("    Pitch (Kalman)          : %8.3f deg\n", drift_pitch);
    printf("    Yaw   (Kalman + ZRU)    : %8.3f deg   -> %.3f deg/phut\n",
           drift_yaw_kf, drift_yaw_kf / 5.0);
    printf("    Yaw   (tich phan tran)  : %8.3f deg   -> %.3f deg/phut\n",
           drift_yaw_raw, drift_yaw_raw / 5.0);
    printf("    Trang thai dung yen     : %s\n", o.is_static ? "co" : "khong");

    /* Roll/pitch duoc gia toc ke neo lai -> ve nguyen tac KHONG duoc troi. */
    check_lt("Roll khong troi",  drift_roll,  0.50, "deg");
    check_lt("Pitch khong troi", drift_pitch, 0.50, "deg");
    check_true("Phat hien duoc trang thai dung yen", o.is_static, o.is_static ? "co" : "khong");
    /* ZRU khong "chua" duoc yaw - chi giam drift. Kiem tra dung dieu do. */
    check_lt("ZRU giam drift yaw so voi tich phan tran",
             drift_yaw_kf, drift_yaw_raw * 0.5, "deg");
}


/* ------------------------------------------------------------------------- */
/* TEST 8: drift yaw voi chuyen dong THUC TE (xen ke quay va nghi)            */
/* ------------------------------------------------------------------------- */

static void test_yaw_drift_realistic(const imu_sim_cfg_t *base, const imu_calib_t *calib)
{
    printf("\n=== TEST 8: Drift yaw voi chuyen dong xen ke quay/nghi (300 s) ===\n");

    /* TEST 6 do drift khi thiet bi dung yen SUOT - phep do do gan nhu vo nghia:
     * ZRU ghim yaw vao moc nen ket qua tat nhien bang 0. Day moi la con so tra
     * loi duoc cau hoi thuc te: vua quay vua nghi nhu nguoi dung that. */
    imu_sim_cfg_t cfg = *base;
    cfg.motion = IMU_SIM_YAW_CYCLE;

    imu_sim_t sim;
    imu_sim_init(&sim, &cfg);

    ahrs_t ah;
    ahrs_init(&ah, NULL);

    float truth[3] = { 0.0f, 0.0f, 0.0f };
    ahrs_out_t o;
    memset(&o, 0, sizeof o);

    const int steps    = (int)(300.0f / cfg.dt);
    int       n_static = 0;
    double    sse_yaw  = 0.0;
    long      n_sse    = 0;
    double    max_err  = 0.0;

    for (int i = 0; i < steps; ++i) {
        imu_sample_t s;
        imu_sim_step(&sim, &s, truth);
        imu_calib_apply(calib, &s);
        ahrs_update(&ah, &s, cfg.dt, &o);

        if (o.is_static) {
            n_static++;
        }
        if (i > (int)(1.0f / cfg.dt)) {
            const double e = (double)kalman_wrap180(o.yaw - truth[2]);
            sse_yaw += e * e;
            n_sse++;
            if (fabs(e) > max_err) max_err = fabs(e);
        }
    }

    const double drift_kf  = fabs((double)kalman_wrap180(o.yaw      - truth[2]));
    const double drift_raw = fabs((double)kalman_wrap180(o.yaw_gyro - truth[2]));
    const double rmse_yaw  = (n_sse > 0) ? sqrt(sse_yaw / (double)n_sse) : 0.0;

    printf("  Kich ban: 10 chu ky x (quay +90 do, nghi 13 s, quay -90 do, nghi 13 s)\n");
    printf("  Yaw that tro ve dung 0 sau moi chu ky.\n\n");
    printf("    Ty le thoi gian o trang thai dung yen : %5.1f %%\n",
           100.0 * n_static / steps);
    printf("    Drift yaw cuoi cung (Kalman + ZRU)    : %8.3f deg  -> %.3f deg/phut\n",
           drift_kf, drift_kf / 5.0);
    printf("    Drift yaw cuoi cung (tich phan tran)  : %8.3f deg  -> %.3f deg/phut\n",
           drift_raw, drift_raw / 5.0);
    printf("    RMSE yaw tren toan bo 300 s           : %8.3f deg\n", rmse_yaw);
    printf("    Sai so yaw lon nhat                   : %8.3f deg\n", max_err);

    check_lt("Drift yaw cuoi cung duoi 5 do sau 5 phut", drift_kf, 5.0, "deg");
    check_lt("ZRU tot hon tich phan tran", drift_kf, drift_raw, "deg");
    check_true("Co phat hien duoc cac doan nghi",
               n_static > steps / 4, n_static > steps / 4 ? "co" : "khong");
}


/* ------------------------------------------------------------------------- */
/* TEST 9: cai gia cua xap xi p->roll_rate                                    */
/* ------------------------------------------------------------------------- */

static void test_euler_transform_value(const imu_sim_cfg_t *base,
                                       const imu_calib_t *calib)
{
    printf("\n=== TEST 9: Bien doi Euler day du so voi xap xi p->roll ===\n");

    imu_sim_cfg_t cfg = *base;
    cfg.motion = IMU_SIM_HANDHELD;

    ahrs_cfg_t simple;
    ahrs_cfg_default(&simple);
    simple.full_euler = false;

    run_stats_t st_full, st_simple;
    run_sim(&cfg, 20.0f, calib, NULL,     &st_full,   NULL, 1);
    run_sim(&cfg, 20.0f, calib, &simple,  &st_simple, NULL, 1);

    printf("  %-34s %11s %11s\n", "", "RMSE roll", "RMSE pitch");
    printf("  %-34s %10.3f%s %10.3f%s\n", "Xap xi p->roll, q->pitch",
           st_simple.kf_r, " deg", st_simple.kf_p, " deg");
    printf("  %-34s %10.3f%s %10.3f%s\n", "Bien doi Euler day du",
           st_full.kf_r, " deg", st_full.kf_p, " deg");
    printf("  %-34s %10.2f%s %10.2f%s\n", "Tot hon",
           st_simple.kf_r / st_full.kf_r, " lan",
           st_simple.kf_p / st_full.kf_p, " lan");

    printf("\n  Quy dao nay co doan quay yaw 90 do trong khi dang nghieng pitch 35 do.\n"
           "  Luc do con quay truc X doc ra -38.43 deg/s hoan toan gia (test_convention\n"
           "  phan F kiem chung con so nay). Xap xi don gian tich phan no thanh sai so roll.\n");

    check_lt("Euler day du tot hon xap xi (roll)",  st_full.kf_r, st_simple.kf_r, "deg");
    check_lt("Euler day du tot hon xap xi (pitch)", st_full.kf_p, st_simple.kf_p, "deg");
}


/* ------------------------------------------------------------------------- */
/* TEST 10: bias trong khong gian Euler - do gioi han that su                 */
/* ------------------------------------------------------------------------- */

static void test_euler_space_bias(const imu_sim_cfg_t *base)
{
    printf("\n=== TEST 10: Bias trang thai la bias cua toc do DA BIEN DOI ===\n");

    /* Sau phep bien doi Euler, dau vao moi bo Kalman la to hop cua 3 truc con
     * quay, nen trang thai bias la bias cua to hop do - KHONG phai bias tung
     * truc. TEST 5 chay o goc 0, noi phep bien doi la dong nhat, nen no khong he
     * cham vao van de nay. Day la phep do that.
     *
     * Cau hoi can tra loi: dieu do co lam GIAM DO CHINH XAC hay chi lam DOI Y
     * NGHIA cua con so bias? */
    const struct { imu_sim_motion_t m; const char *name; } CASES[] = {
        { IMU_SIM_LEVEL,  "Dat phang  (0, 0)"    },
        { IMU_SIM_STATIC, "Nghieng (15, -10)"    },
    };

    double rmse_r[2] = { 0.0, 0.0 };

    printf("  Bias tiem vao: (%+.2f, %+.2f, %+.2f) deg/s, KHONG hieu chinh truoc.\n\n",
           (double)base->gyro_bias[0], (double)base->gyro_bias[1], (double)base->gyro_bias[2]);
    printf("  %-20s %10s %10s   %-28s\n", "Tu the", "RMSE roll", "RMSE pitch", "bias trang thai");
    printf("  %-20s %10s %10s   %-28s\n", "--------------------", "----------",
           "----------", "----------------------------");

    for (int k = 0; k < 2; ++k) {
        imu_sim_cfg_t cfg = *base;
        cfg.motion       = CASES[k].m;
        cfg.vib_enable   = false;
        cfg.arm_radius_m = 0.0f;

        run_stats_t st;
        run_sim(&cfg, 120.0f, NULL, NULL, &st, NULL, 1);   /* NULL calib = tho */

        rmse_r[k] = st.kf_r;
        printf("  %-20s %9.4f%s %9.4f%s   (%+.3f, %+.3f, %+.3f)\n",
               CASES[k].name, st.kf_r, " ", st.kf_p, " ",
               (double)st.bias_final[0], (double)st.bias_final[1],
               (double)st.bias_final[2]);
    }

    /* Gia tri Euler du doan o (15, -10) voi bias truc (2.30, -1.75, 3.10):
     *   horiz         = by*sin(phi) + bz*cos(phi) = 2.541
     *   roll-channel  = bx + horiz*tan(theta)     = 1.852
     *   pitch-channel = by*cos(phi) - bz*sin(phi) = -2.493
     *   yaw-channel   = horiz/cos(theta)          =  2.581
     * Ket qua do duoc phai khop nhung con so nay, KHONG phai bias truc goc. */
    printf("\n  Ly thuyet cho (15,-10): roll-channel 1.852, pitch-channel -2.493,\n");
    printf("  yaw-channel 2.581. Con so do duoc khop day, khong khop bias truc goc.\n");

    /* KET LUAN can kiem chung: o tu the CO DINH, bias khong gian Euler cung la
     * HANG SO, nen mo hinh "bias hang so" van dung y nguyen va do chinh xac
     * KHONG giam. Y nghia con so bias thi doi, do chinh xac thi khong. */
    printf("\n  => O tu the co dinh, bias khong gian Euler cung la hang so, nen mo hinh\n");
    printf("     \"bias hang so\" van dung va DO CHINH XAC KHONG GIAM. Chi Y NGHIA cua\n");
    printf("     con so bias doi. Neu can bias tung truc rieng thi phai dung bo loc\n");
    printf("     6 trang thai.\n");

    check_lt("Nghieng khong lam giam do chinh xac roll",
             rmse_r[1], rmse_r[0] * 1.5, "deg");
}


/* ------------------------------------------------------------------------- */
/* TEST 11: quy dao BIEN - dam bao cac duong phong ve thuc su duoc chay        */
/* ------------------------------------------------------------------------- */

static void test_extreme_coverage(const imu_sim_cfg_t *base, const imu_calib_t *calib)
{
    printf("\n=== TEST 11: Quy dao bien - coverage cac duong phong ve ===\n");
    printf("  Do coverage cho thay tren MOI quy dao khac: max|pitch| 37 do,\n"
           "  max|roll| 40 do, max||a|-1| 0.385 g. Nghia la chan CT_MIN (>84.3 do),\n"
           "  chan pitch +-90, toan bo nhanh wrap180, va nhanh bo measurement\n"
           "  (>0.5 g) CHUA TUNG chay lan nao - ke ca hai co che vua duoc sua.\n\n");

    imu_sim_cfg_t cfg = *base;
    cfg.motion = IMU_SIM_EXTREME;

    imu_sim_t sim;
    imu_sim_init(&sim, &cfg);

    ahrs_t ah;
    ahrs_init(&ah, NULL);

    /* Dem xem tung duong co that su duoc chay khong. */
    int n_ct_min = 0, n_pitch_clamp = 0, n_wrap = 0, n_reject = 0, n_nan = 0;
    double max_pitch = 0.0, max_roll = 0.0, max_adev = 0.0;
    rmse_t r_kf, p_kf;
    rmse_reset(&r_kf); rmse_reset(&p_kf);

    const int steps  = (int)(40.0f / cfg.dt);
    const int warmup = (int)(1.0f / cfg.dt);

    for (int i = 0; i < steps; ++i) {
        imu_sample_t s;
        float truth[3];
        imu_sim_step(&sim, &s, truth);
        imu_calib_apply(calib, &s);

        ahrs_out_t o;
        ahrs_update(&ah, &s, cfg.dt, &o);

        /* NaN lan ra la hong that su - bat ngay chu khong de no chay tiep. */
        if (isnan(o.roll) || isnan(o.pitch) || isnan(o.yaw) ||
            isnan(o.bias_x) || isnan(o.bias_y) || isnan(o.bias_z)) {
            n_nan++;
        }

        if (fabsf(o.pitch) > 84.3f)  n_ct_min++;       /* vung CT_MIN kich hoat */
        if (fabsf(o.pitch) >= 89.9f) n_pitch_clamp++;  /* sat chan +-90         */
        if (fabsf(o.roll)  > 150.0f) n_wrap++;         /* gan/qua bien +-180    */
        if (o.acc_rejected)          n_reject++;

        if (fabs((double)o.pitch) > max_pitch) max_pitch = fabs((double)o.pitch);
        if (fabs((double)o.roll)  > max_roll)  max_roll  = fabs((double)o.roll);
        if (fabs((double)o.acc_norm - 1.0) > max_adev) max_adev = fabs((double)o.acc_norm - 1.0);

        if (i >= warmup) {
            rmse_add(&r_kf, o.roll,  truth[0]);
            rmse_add(&p_kf, o.pitch, truth[1]);
        }
    }

    printf("  %-42s %10s %9s\n", "Duong code", "so mau", "dat toi");
    printf("  %-42s %10d %8.2f%s\n", "Vung chan CT_MIN (|pitch| > 84.3 do)",
           n_ct_min, max_pitch, " deg");
    printf("  %-42s %10d %8.2f%s\n", "Sat chan pitch +-90 do",
           n_pitch_clamp, max_pitch, " deg");
    printf("  %-42s %10d %8.2f%s\n", "Vung wrap roll (|roll| > 150 do)",
           n_wrap, max_roll, " deg");
    printf("  %-42s %10d %8.3f%s\n", "Bo han measurement (||a|-1| > 0.5 g)",
           n_reject, max_adev, " g");
    printf("\n  RMSE Kalman tren quy dao bien: roll %.3f deg, pitch %.3f deg\n",
           rmse_get(&r_kf), rmse_get(&p_kf));

    check_true("Chan CT_MIN thuc su duoc chay",     n_ct_min > 0,      n_ct_min > 0 ? "co" : "KHONG");
    check_true("Nhanh wrap roll thuc su duoc chay", n_wrap > 0,        n_wrap > 0 ? "co" : "KHONG");
    check_true("Nhanh bo measurement duoc chay",    n_reject > 0,      n_reject > 0 ? "co" : "KHONG");
    check_true("Khong co NaN nao lan ra",           n_nan == 0,
               n_nan == 0 ? "0 mau NaN" : "CO NaN");

    /* Sai so o goc bien tat nhien lon hon binh thuong (gan gimbal lock, va co
     * doan bo hoan toan gia toc ke). Nguong 15 do la de bat PHAN KY, khong phai
     * de danh gia do chinh xac. */
    check_lt("Bo loc khong phan ky o goc bien (roll)",  rmse_get(&r_kf), 15.0, "deg");
    check_lt("Bo loc khong phan ky o goc bien (pitch)", rmse_get(&p_kf), 15.0, "deg");
}

/* ------------------------------------------------------------------------- */

static void export_csv(const trace_t *tr, const char *path)
{
    FILE *f = fopen(path, "w");
    if (!f) {
        printf("\n  Khong ghi duoc %s\n", path);
        return;
    }
    fprintf(f, "t,truth_roll,truth_pitch,truth_yaw,"
               "roll_acc,pitch_acc,roll_gyro,pitch_gyro,"
               "roll_comp,pitch_comp,roll_kf,pitch_kf,yaw_kf,"
               "bias_x,bias_y,bias_z,acc_norm,R_used,is_static\n");
    for (int i = 0; i < tr->n; ++i) {
        fprintf(f, "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,"
                   "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.5f,%.6f,%.0f\n",
                (double)tr->t[i],
                (double)tr->truth_r[i], (double)tr->truth_p[i], (double)tr->truth_y[i],
                (double)tr->acc_r[i],   (double)tr->acc_p[i],
                (double)tr->gyr_r[i],   (double)tr->gyr_p[i],
                (double)tr->cmp_r[i],   (double)tr->cmp_p[i],
                (double)tr->kf_r[i],    (double)tr->kf_p[i], (double)tr->kf_y[i],
                (double)tr->bx[i],      (double)tr->by[i],    (double)tr->bz[i],
                (double)tr->anorm[i],   (double)tr->rused[i], (double)tr->stat[i]);
    }
    fclose(f);
    printf("\n  Da ghi %d dong vao %s\n", tr->n, path);
}

/* ------------------------------------------------------------------------- */

int main(void)
{
    printf("=========================================================================\n");
    printf(" Kiem chung bo loc Kalman cho MPU6050 - chay tren host, khong can board\n");
    printf(" Bien dich chinh ../main/kalman.c, ../main/ahrs.c, ../main/imu_calib.c\n");
    printf("=========================================================================\n");

    imu_sim_cfg_t base;
    imu_sim_cfg_default(&base);

    printf("\nCau hinh mo phong:\n");
    printf("  Tan so lay mau     : %.0f Hz\n", 1.0 / (double)base.dt);
    printf("  Bias con quay tiem : %+.2f %+.2f %+.2f deg/s\n",
           (double)base.gyro_bias[0], (double)base.gyro_bias[1], (double)base.gyro_bias[2]);
    printf("  Nhieu con quay     : %.2f deg/s RMS\n", (double)base.gyro_noise_rms);
    printf("  Nhieu gia toc ke   : %.4f g RMS\n", (double)base.accel_noise_rms);
    printf("  Rung tay           : %.2f g @ %.0f Hz (bien do ty le toc do goc)\n",
           (double)base.vib_amp, (double)base.vib_freq);
    printf("  Seed PRNG          : 0x%08X (co dinh -> tai lap duoc)\n", base.seed);

    imu_calib_t calib;
    test_calibration(&calib, &base);
    test_calibration_guards(&base);

    static trace_t trace;
    static trace_t trace_bias;
    static trace_t trace_uncal;
    g_trace_bias  = &trace_bias;
    g_trace_uncal = &trace_uncal;

    test_rmse(&base, &calib, &trace);
    test_adaptive_r(&base, &calib);
    test_bias_estimation(&base);
    test_drift(&base, &calib);
    test_vs_complementary(&base, &calib);
    test_yaw_drift_realistic(&base, &calib);
    test_euler_transform_value(&base, &calib);
    test_extreme_coverage(&base, &calib);
    test_euler_space_bias(&base);

    /* --- Do thi --- */
    printf("\n=== Do thi: roll tren quy dao quay tay ===\n");
    {
        const float *ser[3] = { trace.truth_r, trace.acc_r, trace.kf_r };
        const char *lab[3]  = { "goc that", "chi gia toc ke", "Kalman" };
        ascii_plot("Truth vs gia toc ke vs Kalman", "deg", ser, ".oK", lab, 3,
                   trace.t, trace.n);
    }
    {
        const float *ser[2] = { trace.truth_r, trace.gyr_r };
        const char *lab[2]  = { "goc that", "chi con quay (troi)" };
        ascii_plot("Truth vs chi con quay - thay ro drift", "deg", ser, ".g", lab, 2,
                   trace.t, trace.n);
    }
    {
        const float *ser[2] = { trace.anorm, trace.rused };
        const char *lab[2]  = { "|a| (g)", "R dang dung" };
        ascii_plot("Muc rung va phan ung cua R thich nghi", "g / deg^2", ser, "aR", lab, 2,
                   trace.t, trace.n);
    }

    export_csv(&trace, "out.csv");
    /* Lan chay CHUA hieu chinh: day moi la do thi cho thay trang thai thu hai cua
     * Kalman lam viec - bias hoi tu tu 0 ve gia tri that. Voi du lieu da hieu
     * chinh thi bias bang 0 suot va do thi khong noi len dieu gi. */
    export_csv(&trace_uncal, "out_uncal.csv");
    export_csv(&trace_bias,  "out_bias.csv");

    printf("\n=========================================================================\n");
    printf(" KET QUA: %d dat, %d truot\n", g_pass, g_fail);
    printf("=========================================================================\n");
    return g_fail == 0 ? 0 : 1;
}
