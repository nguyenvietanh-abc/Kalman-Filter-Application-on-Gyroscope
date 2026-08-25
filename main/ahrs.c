/*
 * ahrs.c - Hop nhat accel + gyro thanh goc. Xem ahrs.h cho tong quan.
 */
#include "ahrs.h"
#include <math.h>

#define RAD2DEG 57.29577951308232f
#define DEG2RAD 0.017453292519943295f

/* Chan suy bien khi pitch -> +-90 do: tan(theta) va 1/cos(theta) tien ra vo cuc.
 * 0.1 tuong ung pitch ~ +-84.3 do. */
#define CT_MIN  0.1f

/* ------------------------------------------------------------------------- */

float ahrs_accel_norm(const imu_sample_t *s)
{
    return sqrtf(s->ax * s->ax + s->ay * s->ay + s->az * s->az);
}

void ahrs_accel_angles(const imu_sample_t *s, float *roll_deg, float *pitch_deg)
{
    /* Dat phang, mat tren huong len: (ax,ay,az) = (0,0,+1)g -> roll = pitch = 0.
     * roll dung atan2 hai doi so nen phu duoc ca dai +-180 do.
     * pitch chia cho do lon hinh chieu tren mat phang YZ nen chi dat +-90 do -
     * day la gioi han ban chat cua bieu dien Euler, khong phai loi cai dat. */
    if (roll_deg) {
        *roll_deg = atan2f(s->ay, s->az) * RAD2DEG;
    }
    if (pitch_deg) {
        *pitch_deg = atan2f(-s->ax, sqrtf(s->ay * s->ay + s->az * s->az)) * RAD2DEG;
    }
}

void ahrs_cfg_default(ahrs_cfg_t *cfg)
{
    cfg->Q_angle   = KALMAN_DEFAULT_Q_ANGLE;
    cfg->Q_bias    = KALMAN_DEFAULT_Q_BIAS;
    cfg->R_measure = KALMAN_DEFAULT_R_MEASURE;

    /* Rung tay thuong lam |a| lech 0.1 - 0.4 g. Duoi 0.1 g coi nhu nhieu do
     * binh thuong; tren do giam tin dan; tren 0.5 g thi gia toc ke gan nhu vo
     * dung cho viec do goc nen bo hoan toan. */
    cfg->vib_thresh     = 0.10f;
    cfg->vib_gain       = 40.0f;
    cfg->vib_R_max_mult = 100.0f;
    cfg->vib_reject     = 0.50f;
    /* 0.30 s: du dai de triet rung tay 8-15 Hz, du ngan de bat kip mot cu vung
     * thanh keo dai ~1 s. */
    cfg->vib_tau        = 0.30f;

    cfg->comp_alpha = 0.98f;
    cfg->full_euler = true;

    /* Nguong do lech dong phai long hon nhieu do cua con quay (MPU6050 nhieu
     * khoang 0.4 deg/s RMS sau DLPF, do lech tuyet doi trung binh ~0.8*sigma
     * = 0.32 deg/s) nhung chat hon chuyen dong tay that. */
    cfg->static_dev_thresh = 1.5f;
    /* 20 deg/s la sai so zero-rate output toi da theo datasheet MPU6050 - chot
     * phai long hon con so do de mot con chip xau nhat van bootstrap duoc khi
     * chua hieu chinh, nhung du chat de chan mot cu quay tay co y (thuong
     * >40 deg/s). */
    cfg->static_rate_gate  = 20.0f;
    cfg->static_tau        = 0.15f;
    cfg->static_acc_thresh = 0.05f;
    cfg->static_hold_s     = 0.25f;
    cfg->yaw_R_zru         = 0.10f;
}

void ahrs_init(ahrs_t *a, const ahrs_cfg_t *cfg)
{
    if (cfg) {
        a->cfg = *cfg;
    } else {
        ahrs_cfg_default(&a->cfg);
    }
    const ahrs_cfg_t *c = &a->cfg;

    kalman_init(&a->k_roll,  c->Q_angle, c->Q_bias, c->R_measure);
    kalman_init(&a->k_pitch, c->Q_angle, c->Q_bias, c->R_measure);
    kalman_init(&a->k_yaw,   c->Q_angle, c->Q_bias, c->yaw_R_zru);

    /* roll va yaw chay tren dai +-180 nen phai bat wrap. pitch chi +-90,
     * khong bao gio vuot bien nen de nguyen. */
    a->k_roll.wrap_360  = true;
    a->k_yaw.wrap_360   = true;
    a->k_pitch.wrap_360 = false;

    a->roll_gyro  = a->pitch_gyro = a->yaw_gyro = 0.0f;
    a->roll_comp  = a->pitch_comp = 0.0f;
    a->acc_dist_lp = 0.0f;
    for (int i = 0; i < 3; ++i) {
        a->gyro_ema[i] = 0.0f;
        a->gyro_dev[i] = 0.0f;
    }
    a->static_timer = 0.0f;
    a->yaw_anchor   = 0.0f;
    a->is_static    = false;
    a->seeded       = false;
}

/* ------------------------------------------------------------------------- */

/* R thich nghi. Y tuong: gia toc ke chi do dung goc khi luc duy nhat tac dung
 * len no la trong luc. Khi lac tay, gia toc tuyen tinh cong vao lam |a| lech
 * khoi 1 g - do chinh la thu do duoc de biet PHEP DO DANG XAU DEN MUC NAO, va
 * tu do noi cho Kalman biet nen giam tin gia toc ke bao nhieu. */
static float adaptive_R(const ahrs_cfg_t *c, float *dist_lp,
                        float acc_norm, float dt, bool *reject)
{
    const float e_inst = fabsf(acc_norm - 1.0f);

    /* Loai bo measurement: dung gia tri TUC THOI. Mot cu va dap thuc su lam mau
     * do vo dung ngay tai mau do, khong can cho loc. */
    *reject = (e_inst > c->vib_reject);

    /* Muc do giam tin: dung gia tri DA LOC. Xem giai thich trong ahrs.h.
     * O tau = 0.3 s, rung 11 Hz bi suy giam khoang 20 lan, con gia toc huong
     * tam duy tri thi di qua gan nguyen ven. */
    float lambda = dt / c->vib_tau;
    if (lambda > 1.0f) {
        lambda = 1.0f;
    }
    *dist_lp += lambda * (e_inst - *dist_lp);

    if (*dist_lp <= c->vib_thresh) {
        return c->R_measure;
    }

    float mult = 1.0f + c->vib_gain * (*dist_lp - c->vib_thresh);
    if (mult > c->vib_R_max_mult) {
        mult = c->vib_R_max_mult;
    }
    return c->R_measure * mult;
}

/* Doi toc do goc he than (p,q,r tu con quay) sang toc do Euler (phi/theta/psi).
 *
 *   phi_dot   = p + (q*sin(phi) + r*cos(phi)) * tan(theta)
 *   theta_dot = q*cos(phi) - r*sin(phi)
 *   psi_dot   = (q*sin(phi) + r*cos(phi)) / cos(theta)
 *
 * VI SAO PHAI LAM DAY DU thay vi xap xi p->roll, q->pitch:
 *   Khi thanh cam tay dang nghieng va ta quay YAW, con quay truc X doc ra mot
 *   toc do khac 0 (p = -psi_dot*sin(theta)) MAC DU roll khong he doi. Vi du
 *   thuc: nghieng pitch 35 do roi quay yaw 67 deg/s -> truc X doc -38 deg/s
 *   hoan toan gia. Xap xi don gian se tich phan con so gia do thanh sai so roll.
 *
 *   Sai so nay lon hon han sai so do rung, va no lam co che R thich nghi phan
 *   tac dung: dung luc con quay sai nhat thi R thich nghi lai giam tin gia toc
 *   ke - thu duy nhat con dung. Sua nhanh nay moi la dieu kien de R thich nghi
 *   phat huy.
 *
 * Goc phi/theta lay tu chinh uoc luong hien tai cua bo loc (cach lam chuan). */
static void body_to_euler_rates(float roll_deg, float pitch_deg,
                                float p, float q, float r,
                                float *roll_rate, float *pitch_rate, float *yaw_rate)
{
    const float phi   = roll_deg  * DEG2RAD;
    const float theta = pitch_deg * DEG2RAD;

    const float sp = sinf(phi),   cp = cosf(phi);
    const float st = sinf(theta);

    /* Trong mien hop le cua bieu dien Euler, |pitch| <= 90 do nen cos(theta) >= 0.
     * Vi vay chan xuong CT_MIN va GIU DAU DUONG.
     *
     * KHONG duoc chan kieu "giu nguyen dau" (ct = ct>=0 ? CT_MIN : -CT_MIN): neu
     * uoc luong pitch vuot qua 90 do - dieu co the xay ra thoang qua khi lat
     * nhanh, vi buoc du doan tich phan tu do trong khi phep do gia toc ke bi
     * chan cung o +-90 - thi cos(theta) doi dau, ct nhay tu +0.1 sang -0.1, va
     * CA HAI he so st/ct lan 1/ct DOI DAU. Luc do so hang coupling roll bi dao
     * chieu, thanh phan hoi tiep duong, va bo loc co the mat on dinh. Chan
     * duong bien no thanh gioi han do loi thay vi mot buoc nhay dau. */
    float ct = cosf(theta);
    if (ct < CT_MIN) {
        ct = CT_MIN;
    }

    const float horiz = q * sp + r * cp;   /* thanh phan dung chung */

    *roll_rate  = p + horiz * (st / ct);
    *pitch_rate = q * cp - r * sp;
    *yaw_rate   = horiz / ct;
}

/* Tron kieu complementary nhung an toan khi vuot bien +-180.
 * Dang thong thuong a*(x+w*dt) + (1-a)*z sai khi x va z nam hai ben bien
 * (vd x=179, z=-179 se cho ket qua ~0 thay vi ~180). */
static float comp_blend_wrapped(float prev, float rate, float dt, float meas, float alpha)
{
    const float predicted = prev + rate * dt;
    const float err = kalman_wrap180(meas - predicted);
    return kalman_wrap180(predicted + (1.0f - alpha) * err);
}

static void ahrs_fill_out(const ahrs_t *a, ahrs_out_t *out,
                          float roll_acc, float pitch_acc,
                          float acc_norm, float R_used, bool rejected)
{
    if (!out) {
        return;
    }
    out->roll  = a->k_roll.angle;
    out->pitch = a->k_pitch.angle;
    out->yaw   = a->k_yaw.angle;

    out->roll_acc   = roll_acc;
    out->pitch_acc  = pitch_acc;
    out->roll_gyro  = a->roll_gyro;
    out->pitch_gyro = a->pitch_gyro;
    out->yaw_gyro   = a->yaw_gyro;
    out->roll_comp  = a->roll_comp;
    out->pitch_comp = a->pitch_comp;

    out->bias_x = a->k_roll.bias;
    out->bias_y = a->k_pitch.bias;
    out->bias_z = a->k_yaw.bias;

    out->acc_norm     = acc_norm;
    out->R_used       = R_used;
    out->is_static    = a->is_static;
    out->acc_rejected = rejected;
}

/* ------------------------------------------------------------------------- */

void ahrs_update(ahrs_t *a, const imu_sample_t *s, float dt, ahrs_out_t *out)
{
    const ahrs_cfg_t *c = &a->cfg;

    float roll_acc, pitch_acc;
    ahrs_accel_angles(s, &roll_acc, &pitch_acc);
    const float acc_norm = ahrs_accel_norm(s);

    /* --- Mau dau tien: gan truc tiep goc gia toc ke cho moi bo uoc luong ---
     * Neu khong seed, filter phai bo tu 0 den goc that, mat mot doan thoi gian
     * hoi tu vo ich va lam bang RMSE xau di mot cach gia tao. */
    if (!a->seeded) {
        kalman_set_angle(&a->k_roll,  roll_acc);
        kalman_set_angle(&a->k_pitch, pitch_acc);
        kalman_set_angle(&a->k_yaw,   0.0f);   /* yaw khong co goc tham chieu: quy uoc 0 */

        a->roll_gyro  = a->roll_comp  = roll_acc;
        a->pitch_gyro = a->pitch_comp = pitch_acc;
        a->yaw_gyro   = 0.0f;
        a->yaw_anchor = 0.0f;

        /* Seed bo do do lech bang chinh mau dau tien. Neu de ema = 0 thi mau dau
         * tien (chua tru bias) se tao mot xung do lech gia tao, lam tri hoan
         * viec phat hien dung yen them mot doan bang static_tau. */
        a->gyro_ema[0] = s->gx;
        a->gyro_ema[1] = s->gy;
        a->gyro_ema[2] = s->gz;
        a->gyro_dev[0] = a->gyro_dev[1] = a->gyro_dev[2] = 0.0f;

        a->seeded     = true;

        /* Chua co moc thoi gian truoc do -> khong tich phan mau nay. */
        ahrs_fill_out(a, out, roll_acc, pitch_acc, acc_norm, c->R_measure, false);
        return;
    }

    if (!(dt > 0.0f) || dt > 1.0f) {
        /* dt phi ly: giu nguyen trang thai, chi bao cao lai. */
        ahrs_fill_out(a, out, roll_acc, pitch_acc, acc_norm, 0.0f, false);
        return;
    }

    /* --- Phat hien dung yen (phuc vu ZRU cho yaw) ---
     *
     * Tieu chi 1 (chinh): DO LECH DONG cua toc do goc nho.
     *   Bias con quay la hang so -> khong lam tang do lech. Nho vay tieu chi nay
     *   hoat dong ngay ca khi CHUA hieu chinh offset, la dieu kien can de bias
     *   truc Z bootstrap duoc qua ZRU. Neu thay bang |gx| < nguong thi mot chip
     *   chua hieu chinh voi bias 3 deg/s se mai mai bi coi la dang chuyen dong.
     *
     * Tieu chi 2 (chot tho): toc do DA TRU BIAS phai nho.
     *   Chan truong hop quay deu toc do cao - luc do do lech dong cung nho vi
     *   toc do gan nhu khong doi, tieu chi 1 mot minh se bi lua.
     *
     * Tieu chi 3: |a| xap xi 1 g -> khong co gia toc tuyen tinh dang tac dong.
     *
     * GIOI HAN DA BIET: mot cu quay HOAN TOAN DEU, cham hon static_rate_gate,
     * quanh dung truc trong luc, keo dai hon static_hold_s se bi nham la dung
     * yen. Chuyen dong tay that luon co rung va bien thien toc do nen tieu chi 1
     * bat duoc; truong hop nay chi xay ra voi ban quay co dong co. */
    float lambda = dt / c->static_tau;
    if (lambda > 1.0f) {
        lambda = 1.0f;
    }
    const float g_in[3]   = { s->gx, s->gy, s->gz };
    const float bias_est[3] = { a->k_roll.bias, a->k_pitch.bias, a->k_yaw.bias };

    bool gyro_quiet = true;
    for (int i = 0; i < 3; ++i) {
        a->gyro_ema[i] += lambda * (g_in[i] - a->gyro_ema[i]);

        const float d = fabsf(g_in[i] - a->gyro_ema[i]);
        a->gyro_dev[i] += lambda * (d - a->gyro_dev[i]);

        if (a->gyro_dev[i] > c->static_dev_thresh) {
            gyro_quiet = false;
        }
        if (fabsf(g_in[i] - bias_est[i]) > c->static_rate_gate) {
            gyro_quiet = false;
        }
    }

    const bool acc_quiet = fabsf(acc_norm - 1.0f) < c->static_acc_thresh;

    if (gyro_quiet && acc_quiet) {
        a->static_timer += dt;
    } else {
        a->static_timer = 0.0f;
        a->is_static    = false;
    }

    const bool was_static = a->is_static;
    if (a->static_timer >= c->static_hold_s) {
        a->is_static = true;
    }
    /* Vua chuyen tu chuyen dong sang dung yen: chot moc yaw hien tai lai. */
    if (a->is_static && !was_static) {
        a->yaw_anchor = a->k_yaw.angle;
    }

    /* --- Doi toc do goc he than sang toc do Euler ---
     * Dung goc uoc luong hien tai cua bo loc lam diem tuyen tinh hoa. */
    float roll_rate, pitch_rate, yaw_rate;
    if (c->full_euler) {
        body_to_euler_rates(a->k_roll.angle, a->k_pitch.angle,
                            s->gx, s->gy, s->gz,
                            &roll_rate, &pitch_rate, &yaw_rate);
    } else {
        /* Xap xi thuong gap: coi toc do truc than la toc do Euler. Chi dung de
         * DO cai gia cua no - xem ghi chu tai truong full_euler trong ahrs.h. */
        roll_rate  = s->gx;
        pitch_rate = s->gy;
        yaw_rate   = s->gz;
    }

    /* --- Roll / Pitch: Kalman voi R thich nghi --- */
    bool rejected;
    const float R = adaptive_R(c, &a->acc_dist_lp, acc_norm, dt, &rejected);

    if (rejected) {
        /* Rung qua manh: gia toc ke khong con noi duoc gi ve goc. Chi du doan.
         * P se phinh ra - dung y nghia vat ly: filter tu biet no dang mat tin. */
        kalman_predict(&a->k_roll,  roll_rate,  dt);
        kalman_predict(&a->k_pitch, pitch_rate, dt);
    } else {
        kalman_update_R(&a->k_roll,  roll_acc,  roll_rate,  dt, R);
        kalman_update_R(&a->k_pitch, pitch_acc, pitch_rate, dt, R);
    }

    /* Pitch duoc DINH NGHIA tren [-90, 90] trong bieu dien Euler 3-2-1; mot uoc
     * luong ngoai khoang do la vo nghia. Buoc du doan tich phan tu do nen no
     * VUOT duoc, nhat la khi lat nhanh hoac khi measurement bi bo vi rung. Chan
     * lai de trang thai luon nam trong mien hop le cua chinh bieu dien do. */
    if (a->k_pitch.angle >  90.0f) a->k_pitch.angle =  90.0f;
    if (a->k_pitch.angle < -90.0f) a->k_pitch.angle = -90.0f;

    /* --- Yaw: ZRU (zero-rate update) ---
     * Khong co tu ke nen khong co nguon do yaw tuyet doi. Nhung khi thiet bi
     * DUNG YEN, ta biet chac mot dieu: yaw that khong thay doi. Do chinh la mot
     * phep do hop le - dua vao Kalman duoi dang "yaw = yaw_anchor". Nho vay
     * bias truc Z tro nen kha quan sat va duoc uoc luong, giup giam manh drift.
     *
     * Luc dang chuyen dong thi khong co phep do nao -> chi du doan, tuc yaw la
     * tich phan cua (gz - bias). Drift con lai la GIOI HAN PHAN CUNG, khong the
     * khu het bang IMU 6 truc. */
    if (a->is_static) {
        kalman_update_R(&a->k_yaw, a->yaw_anchor, yaw_rate, dt, c->yaw_R_zru);
    } else {
        kalman_predict(&a->k_yaw, yaw_rate, dt);
    }

    /* --- Baseline 1: chi con quay. Tich phan tran tren truc tho, khong tru
     *     bias, khong doi he toa do, khong hop nhat. Duong nay PHAI troi - do
     *     la thu can chung minh. --- */
    a->roll_gyro   = kalman_wrap180(a->roll_gyro + s->gx * dt);
    a->pitch_gyro += s->gy * dt;
    a->yaw_gyro    = kalman_wrap180(a->yaw_gyro + s->gz * dt);

    /* --- Baseline 2: complementary filter ---
     * Duoc cap CUNG toc do Euler da doi he nhu Kalman. So sanh phai cong bang:
     * khac nhau o THUAT TOAN HOP NHAT, khong phai o chat luong dau vao. */
    a->roll_comp = comp_blend_wrapped(a->roll_comp, roll_rate, dt, roll_acc, c->comp_alpha);
    /* pitch nam trong +-90, khong bao gio vuot bien nen tron truc tiep. */
    a->pitch_comp = c->comp_alpha * (a->pitch_comp + pitch_rate * dt) +
                    (1.0f - c->comp_alpha) * pitch_acc;

    /* Baseline 3 (chi gia toc ke) khong can trang thai - chinh la roll_acc/pitch_acc. */

    ahrs_fill_out(a, out, roll_acc, pitch_acc, acc_norm, rejected ? 0.0f : R, rejected);
}
