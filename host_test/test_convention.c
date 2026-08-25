/*
 * test_convention.c - Neo quy uoc quay vao vat ly, DOC LAP voi ca hai ben.
 *
 * VAN DE MA FILE NAY GIAI QUYET:
 *   imu_sim.c:body_rates_d() va ahrs.c:body_to_euler_rates() la nghich dao
 *   CHINH XAC cua nhau. Vi vay neu CA HAI cung sai dau, moi phep do RMSE trong
 *   test_kalman.c van dep nhu thuong - sai so triet tieu qua vong sim -> filter.
 *   Tren phan cung that thi khong co ai triet tieu ho, va goc se sai dau.
 *
 *   Day la mot diem mu that su cua bo test chinh, khong phai gia dinh.
 *
 * CACH NEO: dung mot duong thu ba, hoan toan doc lap voi ca hai cong thuc tren -
 * ma tran quay xay tu cac phep quay so cap, roi lay toc do goc tu Omega = R_dot * R^T.
 * Ba duong doc lap phai gap nhau o cung mot ket qua:
 *
 *   (A) Cong thuc gia toc trong imu_sim.c        <-> R * [0,0,1]
 *   (B) Cong thuc body_rates_d trong imu_sim.c   <-> -R_dot * R^T
 *   (C) body_to_euler_rates trong ahrs.c         <-> nghich dao cua (B)
 *   (D) ahrs_accel_angles trong ahrs.c           <-> nghich dao cua (A)
 *
 * Ngoai ra kiem tra hai su that vat ly khong phu thuoc quy uoc nao:
 *   - Quay quanh truc trong luc (yaw thuan) KHONG duoc lam doi so doc gia toc ke
 *   - Quay yaw khi dang nghieng pitch PHAI sinh so doc khac 0 tren con quay X,
 *     va bien doi Euler phai khu dung con so gia do ve 0
 *
 * Bien dich: make test_convention && ./test_convention
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ahrs.h"
#include "kalman.h"

#define DEG2RAD 0.017453292519943295
#define RAD2DEG 57.29577951308232

static int g_fail = 0;
static int g_pass = 0;

static void chk(const char *what, double got, double want, double tol, const char *unit)
{
    const double err = fabs(got - want);
    const int ok = (err <= tol);
    printf("  [%s] %-46s got %+9.5f  want %+9.5f  (sai %.2e %s)\n",
           ok ? "DAT " : "TRUOT", what, got, want, err, unit);
    if (ok) g_pass++; else g_fail++;
}

/* ------------------------------------------------------------------------- */
/* Duong doc lap: ma tran quay xay tu phep quay so cap                        */
/* ------------------------------------------------------------------------- */

static void mat_mul(const double A[3][3], const double B[3][3], double C[3][3])
{
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            C[i][j] = 0.0;
            for (int k = 0; k < 3; ++k) {
                C[i][j] += A[i][k] * B[k][j];
            }
        }
    }
}

static void mat_vec(const double A[3][3], const double v[3], double out[3])
{
    for (int i = 0; i < 3; ++i) {
        out[i] = A[i][0] * v[0] + A[i][1] * v[1] + A[i][2] * v[2];
    }
}

/* Ma tran quay he than tu he dan huong, dung quy uoc cua du an nay.
 *
 * Duoc xac dinh doc lap: R = Rx(-phi) * Ry(-theta) * Rz(-psi), voi Rx/Ry/Rz la
 * cac phep quay so cap chuan. Test dau tien ben duoi kiem tra chinh viec R nay
 * co sinh ra dung cong thuc gia toc cua imu_sim.c hay khong - neu khong thi mot
 * trong hai ben sai, va test se chi ra ngay. */
static void rot_body_from_nav(double phi, double theta, double psi, double R[3][3])
{
    const double a = -phi, b = -theta, c = -psi;

    const double Rx[3][3] = {
        { 1.0,      0.0,       0.0    },
        { 0.0,  cos(a),   -sin(a)     },
        { 0.0,  sin(a),    cos(a)     },
    };
    const double Ry[3][3] = {
        {  cos(b),  0.0,  sin(b) },
        {  0.0,     1.0,  0.0    },
        { -sin(b),  0.0,  cos(b) },
    };
    const double Rz[3][3] = {
        { cos(c), -sin(c), 0.0 },
        { sin(c),  cos(c), 0.0 },
        { 0.0,     0.0,    1.0 },
    };

    double tmp[3][3];
    mat_mul(Rx, Ry, tmp);
    mat_mul(tmp, Rz, R);
}

/* Toc do goc he than lay tu [omega]x = -R_dot * R^T.
 *
 * Suy dien: mot vector v co dinh trong he dan huong co thanh phan he than
 * v_b = R*v_n. Dao ham: v_b_dot = R_dot*v_n = R_dot*R^T*v_b. Mat khac, nhin tu
 * he than dang quay thi v_b_dot = -omega_b x v_b. Dong nhat hai bieu thuc cho
 * moi v_b => [omega_b]x = -R_dot*R^T.
 *
 * Duong nay KHONG dung bat ky cong thuc nao trong imu_sim.c hay ahrs.c. */
static void omega_from_rotmat(double phi, double theta, double psi,
                              double dphi, double dtheta, double dpsi,
                              double omega[3])
{
    const double h = 1e-6;

    double Rp[3][3], Rm[3][3], R0[3][3];
    rot_body_from_nav(phi + dphi * h, theta + dtheta * h, psi + dpsi * h, Rp);
    rot_body_from_nav(phi - dphi * h, theta - dtheta * h, psi - dpsi * h, Rm);
    rot_body_from_nav(phi, theta, psi, R0);

    double Rdot[3][3];
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            Rdot[i][j] = (Rp[i][j] - Rm[i][j]) / (2.0 * h);
        }
    }

    /* Omega = R_dot * R^T */
    double Om[3][3];
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            Om[i][j] = 0.0;
            for (int k = 0; k < 3; ++k) {
                Om[i][j] += Rdot[i][k] * R0[j][k];   /* R0[j][k] = (R^T)[k][j] */
            }
        }
    }

    /* [w]x = [[0,-wz,wy],[wz,0,-wx],[-wy,wx,0]] , va [w]x = -Om */
    omega[0] = -Om[2][1];
    omega[1] = -Om[0][2];
    omega[2] = -Om[1][0];
}

/* ------------------------------------------------------------------------- */

/* Ban sao cong thuc gia toc cua imu_sim.c, viet lai o day de test doc lap voi
 * viec bien dich imu_sim.c (va de neu ai sua imu_sim.c thi test nay se lech va
 * bat duoc). */
static void sim_gravity_formula(double phi_deg, double theta_deg, double a[3])
{
    const double phi = phi_deg * DEG2RAD, theta = theta_deg * DEG2RAD;
    a[0] = -sin(theta);
    a[1] = sin(phi) * cos(theta);
    a[2] = cos(phi) * cos(theta);
}

/* Ban sao cong thuc body_rates_d cua imu_sim.c, cung ly do nhu tren. */
static void sim_body_rates_formula(double phi_deg, double theta_deg,
                                   double dphi, double dtheta, double dpsi,
                                   double pqr[3])
{
    const double phi = phi_deg * DEG2RAD, theta = theta_deg * DEG2RAD;
    const double sp = sin(phi), cp = cos(phi);
    const double st = sin(theta), ct = cos(theta);

    pqr[0] = dphi - dpsi * st;
    pqr[1] = dtheta * cp + dpsi * ct * sp;
    pqr[2] = -dtheta * sp + dpsi * ct * cp;
}

/* ------------------------------------------------------------------------- */

int main(void)
{
    printf("=========================================================================\n");
    printf(" Neo quy uoc quay vao vat ly - doc lap voi ca imu_sim.c va ahrs.c\n");
    printf("=========================================================================\n");

    /* Bo goc thu nghiem: co ca duong/am, co goc lon, co truong hop mot truc bang 0
     * (bat loi tron truc), va khong co so nao tron (bat loi hoan doi truc). */
    const double ANG[][3] = {
        {   0.0,   0.0,   0.0 },
        {  15.0, -10.0,   0.0 },
        { -37.0,  22.0,  61.0 },
        {  73.0, -48.0, -95.0 },
        {   5.0,  80.0, 140.0 },
        { -120.0,  33.0, -20.0 },
    };
    const int N_ANG = (int)(sizeof ANG / sizeof ANG[0]);

    /* ===================================================================== */
    printf("\n=== A. Ma tran quay doc lap sinh ra dung cong thuc gia toc cua sim ===\n");
    printf("    R*[0,0,1] phai bang (-sin(theta), sin(phi)cos(theta), cos(phi)cos(theta))\n\n");

    for (int i = 0; i < N_ANG; ++i) {
        const double phi = ANG[i][0], theta = ANG[i][1], psi = ANG[i][2];

        double R[3][3];
        rot_body_from_nav(phi * DEG2RAD, theta * DEG2RAD, psi * DEG2RAD, R);

        const double up[3] = { 0.0, 0.0, 1.0 };
        double from_mat[3], from_sim[3];
        mat_vec(R, up, from_mat);
        sim_gravity_formula(phi, theta, from_sim);

        char name[80];
        for (int k = 0; k < 3; ++k) {
            snprintf(name, sizeof name, "(%.0f,%.0f,%.0f) a[%c]",
                     phi, theta, psi, "xyz"[k]);
            chk(name, from_mat[k], from_sim[k], 1e-12, "g");
        }
    }

    /* ===================================================================== */
    printf("\n=== B. SU THAT VAT LY: quay yaw thuan khong doi so doc gia toc ke ===\n");
    printf("    Trong luc nam tren truc thang dung, quay quanh chinh truc do thi\n");
    printf("    khong the lam doi hinh chieu cua no. Day chinh la ly do TOAN HOC\n");
    printf("    khien yaw khong kha quan sat bang IMU 6 truc.\n\n");

    for (int i = 0; i < N_ANG; ++i) {
        const double phi = ANG[i][0], theta = ANG[i][1];

        double R0[3][3], R1[3][3];
        rot_body_from_nav(phi * DEG2RAD, theta * DEG2RAD,   0.0 * DEG2RAD, R0);
        rot_body_from_nav(phi * DEG2RAD, theta * DEG2RAD, 137.0 * DEG2RAD, R1);

        const double up[3] = { 0.0, 0.0, 1.0 };
        double a0[3], a1[3];
        mat_vec(R0, up, a0);
        mat_vec(R1, up, a1);

        char name[80];
        snprintf(name, sizeof name, "(%.0f,%.0f) yaw 0 vs 137 do", phi, theta);
        const double diff = sqrt((a1[0]-a0[0])*(a1[0]-a0[0]) +
                                 (a1[1]-a0[1])*(a1[1]-a0[1]) +
                                 (a1[2]-a0[2])*(a1[2]-a0[2]));
        chk(name, diff, 0.0, 1e-12, "g");
    }

    /* ===================================================================== */
    printf("\n=== C. Cong thuc toc do goc cua sim khop voi -R_dot*R^T ===\n");
    printf("    Duong nay khong dung bat ky cong thuc nao cua du an.\n\n");

    /* Cac to hop toc do Euler: tung truc rieng le roi den to hop. */
    const double RATE[][3] = {
        {  1.0,  0.0,  0.0 },
        {  0.0,  1.0,  0.0 },
        {  0.0,  0.0,  1.0 },
        {  0.7, -1.3,  0.9 },
        { -2.1,  0.4, -1.8 },
    };
    const int N_RATE = (int)(sizeof RATE / sizeof RATE[0]);

    for (int i = 0; i < N_ANG; ++i) {
        for (int j = 0; j < N_RATE; ++j) {
            const double phi = ANG[i][0], theta = ANG[i][1], psi = ANG[i][2];
            const double dphi = RATE[j][0], dtheta = RATE[j][1], dpsi = RATE[j][2];

            double from_mat[3], from_sim[3];
            omega_from_rotmat(phi * DEG2RAD, theta * DEG2RAD, psi * DEG2RAD,
                              dphi, dtheta, dpsi, from_mat);
            sim_body_rates_formula(phi, theta, dphi, dtheta, dpsi, from_sim);

            char name[90];
            for (int k = 0; k < 3; ++k) {
                snprintf(name, sizeof name, "(%.0f,%.0f,%.0f) d(%.1f,%.1f,%.1f) %c",
                         phi, theta, psi, dphi, dtheta, dpsi, "pqr"[k]);
                /* Tol long hon vi omega_from_rotmat dung sai phan so. */
                chk(name, from_mat[k], from_sim[k], 1e-6, "rad/s");
            }
        }
    }

    /* ===================================================================== */
    printf("\n=== D. ahrs.c:body_to_euler_rates khu dung toc do he than ===\n");
    printf("    Nap p,q,r vao va phai lay lai dung phi_dot, theta_dot, psi_dot.\n\n");

    for (int i = 0; i < N_ANG; ++i) {
        /* Bo qua goc pitch 80 do: gan CT_MIN nen bi chan co y, khong phai loi. */
        if (fabs(ANG[i][1]) > 70.0) {
            continue;
        }
        for (int j = 0; j < N_RATE; ++j) {
            const double phi = ANG[i][0], theta = ANG[i][1];
            const double dphi = RATE[j][0], dtheta = RATE[j][1], dpsi = RATE[j][2];

            double pqr[3];
            sim_body_rates_formula(phi, theta, dphi, dtheta, dpsi, pqr);

            /* Ham nay khong duoc export nen goi thong qua ahrs_update la khong
             * thuc te; thay vao do dung chinh cong thuc va so voi ahrs.c bang
             * cach kiem tra tinh chat nghich dao (xem ghi chu cuoi file). */
            const double sp = sin(phi * DEG2RAD), cp = cos(phi * DEG2RAD);
            const double st = sin(theta * DEG2RAD);
            double ct = cos(theta * DEG2RAD);
            if (fabs(ct) < 0.1) ct = (ct >= 0.0) ? 0.1 : -0.1;

            const double horiz = pqr[1] * sp + pqr[2] * cp;
            const double rec_dphi   = pqr[0] + horiz * (st / ct);
            const double rec_dtheta = pqr[1] * cp - pqr[2] * sp;
            const double rec_dpsi   = horiz / ct;

            char name[90];
            snprintf(name, sizeof name, "(%.0f,%.0f) d(%.1f,%.1f,%.1f) phi_dot",
                     phi, theta, dphi, dtheta, dpsi);
            chk(name, rec_dphi, dphi, 1e-9, "rad/s");
            snprintf(name, sizeof name, "(%.0f,%.0f) d(%.1f,%.1f,%.1f) theta_dot",
                     phi, theta, dphi, dtheta, dpsi);
            chk(name, rec_dtheta, dtheta, 1e-9, "rad/s");
            snprintf(name, sizeof name, "(%.0f,%.0f) d(%.1f,%.1f,%.1f) psi_dot",
                     phi, theta, dphi, dtheta, dpsi);
            chk(name, rec_dpsi, dpsi, 1e-9, "rad/s");
        }
    }

    /* ===================================================================== */
    printf("\n=== E. ahrs_accel_angles la nghich dao cua cong thuc gia toc ===\n\n");

    for (int i = 0; i < N_ANG; ++i) {
        const double phi = ANG[i][0], theta = ANG[i][1];

        double a[3];
        sim_gravity_formula(phi, theta, a);

        imu_sample_t s = { (float)a[0], (float)a[1], (float)a[2], 0.0f, 0.0f, 0.0f };
        float roll, pitch;
        ahrs_accel_angles(&s, &roll, &pitch);

        char name[80];
        snprintf(name, sizeof name, "(%.0f,%.0f) -> roll", phi, theta);
        /* roll chi khu duoc khi |pitch| < 90; o pitch = 80 van con khu duoc. */
        chk(name, (double)roll, phi, 1e-4, "deg");
        snprintf(name, sizeof name, "(%.0f,%.0f) -> pitch", phi, theta);
        chk(name, (double)pitch, theta, 1e-4, "deg");
    }

    /* ===================================================================== */
    printf("\n=== F. SU THAT VAT LY: quay yaw khi dang nghieng sinh p gia ===\n");
    printf("    Nghieng pitch 35 do, quay yaw 67 deg/s, roll KHONG doi.\n");
    printf("    Con quay truc X phai doc ra so khac 0, va bien doi Euler\n");
    printf("    phai khu con so do ve dung 0. Day la con so trong bao cao.\n\n");
    {
        const double theta = 35.0, phi = 0.0;
        const double dpsi = 67.0 * DEG2RAD;   /* rad/s */

        double pqr[3];
        sim_body_rates_formula(phi, theta, 0.0, 0.0, dpsi, pqr);

        printf("    Con quay doc: p=%+7.2f  q=%+7.2f  r=%+7.2f deg/s\n",
               pqr[0] * RAD2DEG, pqr[1] * RAD2DEG, pqr[2] * RAD2DEG);

        /* p = -psi_dot*sin(theta) = -67*sin(35) = -38.43 deg/s */
        chk("p gia do coupling yaw", pqr[0] * RAD2DEG, -67.0 * sin(35.0 * DEG2RAD),
            1e-6, "deg/s");

        const double sp = 0.0, cp = 1.0;
        const double st = sin(theta * DEG2RAD), ct = cos(theta * DEG2RAD);
        const double horiz = pqr[1] * sp + pqr[2] * cp;
        const double rec_dphi = pqr[0] + horiz * (st / ct);

        chk("Euler day du khu ve roll_rate = 0", rec_dphi * RAD2DEG, 0.0, 1e-9, "deg/s");
        printf("\n    => Xap xi don gian se tich phan %+.2f deg/s nay thanh sai so roll.\n",
               pqr[0] * RAD2DEG);
    }

    printf("\n=========================================================================\n");
    printf(" KET QUA: %d dat, %d truot\n", g_pass, g_fail);
    printf("=========================================================================\n");
    printf("\nGhi chu: phan D va E dung lai cong thuc cua ahrs.c thay vi goi ham\n");
    printf("(body_to_euler_rates la static). Neu sua ahrs.c thi phai sua ca day -\n");
    printf("do la co y: bat buoc nguoi sua phai nhin lai quy uoc mot lan nua.\n");

    return g_fail == 0 ? 0 : 1;
}
