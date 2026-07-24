#include "params.h"
#include <Preferences.h>
#include <string.h>

// ビルド時デフォルト。現行の実機チューニング値(2026-07-24時点、
// 閉路パターンで物理ずれ2mm・1°)と一致させてある。
const Params kDefaultParams = {
    // 車輪速度PID
    .speed_kp = 300.0f,
    .speed_ki = 3000.0f,
    .speed_kd = 0.0f,
    .kf_duty_per_mps = 250.0f,
    .vbatt_nom = 8.0f,
    .speed_lpf_alpha = 0.12f,

    // 旋回中の左右速度同期
    .turn_sync_kp = 0.3f,
    .turn_sync_ki = 1.0f,
    .sync_max_corr = 0.05f,
    .sync_lpf_alpha = 0.02f,

    // 直進停止プロファイル
    .final_appr_mmps = 50.0f,
    .stop_min_mmps = 40.0f,
    .stop_timeout_s = 4.0f,
    .backoff_dist_mm = 30.0f,
    .backoff_mps = 0.12f,
    .stop_hold_sec = 0.5f,

    // 低速連続動作
    .latch_mps = 0.05f,
    .latch_turn_mps = 0.06f,
    .jog_mps = 0.05f,

    // 急停止
    .qstp_decel = 1000.0f,

    // その場旋回
    .turn_kp = 0.35f,
    .turn_ki = 0.3f,
    .turn_kd = 0.05f,
    .turn_fine_rad = 0.2f,
    .turn_max_mps = 0.15f,
    .turn_min_mps = 0.06f,
    .turn_tol_rad = 0.01f,
    .turn_settle_s = 0.08f,
    .turn_max_retry = 3.0f,
    .turn_accel = 4.0f,

    // 直進の角度・角速度フィードバック
    .angle_fb_gain = 0.5f,
    .rate_fb_gain = 0.05f,

    // 壁センサフィードバック
    .wall_threshold = 100.0f,
    .wall_target_ls = 348.0f,
    .wall_target_rs = 241.0f,
    .wall_tilt_gain = 0.0015f,
    .wall_tilt_max = 0.12f,
    .wall_cutoff_mm = 30.0f,
};

Params params = kDefaultParams;

const ParamDef PARAM_TABLE[] = {
    {"speed_kp", offsetof(Params, speed_kp)},
    {"speed_ki", offsetof(Params, speed_ki)},
    {"speed_kd", offsetof(Params, speed_kd)},
    {"kf_duty_per_mps", offsetof(Params, kf_duty_per_mps)},
    {"vbatt_nom", offsetof(Params, vbatt_nom)},
    {"speed_lpf_alpha", offsetof(Params, speed_lpf_alpha)},

    {"turn_sync_kp", offsetof(Params, turn_sync_kp)},
    {"turn_sync_ki", offsetof(Params, turn_sync_ki)},
    {"sync_max_corr", offsetof(Params, sync_max_corr)},
    {"sync_lpf_alpha", offsetof(Params, sync_lpf_alpha)},

    {"final_appr_mmps", offsetof(Params, final_appr_mmps)},
    {"stop_min_mmps", offsetof(Params, stop_min_mmps)},
    {"stop_timeout_s", offsetof(Params, stop_timeout_s)},
    {"backoff_dist_mm", offsetof(Params, backoff_dist_mm)},
    {"backoff_mps", offsetof(Params, backoff_mps)},
    {"stop_hold_sec", offsetof(Params, stop_hold_sec)},

    {"latch_mps", offsetof(Params, latch_mps)},
    {"latch_turn_mps", offsetof(Params, latch_turn_mps)},
    {"jog_mps", offsetof(Params, jog_mps)},

    {"qstp_decel", offsetof(Params, qstp_decel)},

    {"turn_kp", offsetof(Params, turn_kp)},
    {"turn_ki", offsetof(Params, turn_ki)},
    {"turn_kd", offsetof(Params, turn_kd)},
    {"turn_fine_rad", offsetof(Params, turn_fine_rad)},
    {"turn_max_mps", offsetof(Params, turn_max_mps)},
    {"turn_min_mps", offsetof(Params, turn_min_mps)},
    {"turn_tol_rad", offsetof(Params, turn_tol_rad)},
    {"turn_settle_s", offsetof(Params, turn_settle_s)},
    {"turn_max_retry", offsetof(Params, turn_max_retry)},
    {"turn_accel", offsetof(Params, turn_accel)},

    {"angle_fb_gain", offsetof(Params, angle_fb_gain)},
    {"rate_fb_gain", offsetof(Params, rate_fb_gain)},

    {"wall_threshold", offsetof(Params, wall_threshold)},
    {"wall_target_ls", offsetof(Params, wall_target_ls)},
    {"wall_target_rs", offsetof(Params, wall_target_rs)},
    {"wall_tilt_gain", offsetof(Params, wall_tilt_gain)},
    {"wall_tilt_max", offsetof(Params, wall_tilt_max)},
    {"wall_cutoff_mm", offsetof(Params, wall_cutoff_mm)},
};

const size_t PARAM_COUNT = sizeof(PARAM_TABLE) / sizeof(PARAM_TABLE[0]);

namespace {
// Preferences のネームスペース名(15文字以内)
constexpr const char* kNamespace = "mobparams";

float* field_ptr(const ParamDef& def) {
    return reinterpret_cast<float*>(reinterpret_cast<char*>(&params) + def.offset);
}
}  // namespace

void params_reset_to_defaults() {
    params = kDefaultParams;
}

bool params_get(const char* name, float& out_value) {
    for (size_t i = 0; i < PARAM_COUNT; i++) {
        if (strcmp(PARAM_TABLE[i].name, name) == 0) {
            out_value = *field_ptr(PARAM_TABLE[i]);
            return true;
        }
    }
    return false;
}

bool params_set(const char* name, float value) {
    for (size_t i = 0; i < PARAM_COUNT; i++) {
        if (strcmp(PARAM_TABLE[i].name, name) == 0) {
            *field_ptr(PARAM_TABLE[i]) = value;
            return true;
        }
    }
    return false;
}

void params_begin() {
    params_load();
}

bool params_save() {
    Preferences prefs;
    if (!prefs.begin(kNamespace, /*readOnly=*/false)) {
        return false;
    }
    for (size_t i = 0; i < PARAM_COUNT; i++) {
        prefs.putFloat(PARAM_TABLE[i].name, *field_ptr(PARAM_TABLE[i]));
    }
    prefs.end();
    return true;
}

bool params_load() {
    Preferences prefs;
    if (!prefs.begin(kNamespace, /*readOnly=*/true)) {
        // ネームスペース未作成(一度もPSAVEしていない): 現在値のまま
        return false;
    }
    for (size_t i = 0; i < PARAM_COUNT; i++) {
        float* p = field_ptr(PARAM_TABLE[i]);
        // 保存されていないキーは現在値(=デフォルト)を維持する。
        // 新しく追加したパラメータが既存の保存データに無くても
        // 他の項目に影響しないのはこの仕組みのため。
        *p = prefs.getFloat(PARAM_TABLE[i].name, *p);
    }
    prefs.end();
    return true;
}
