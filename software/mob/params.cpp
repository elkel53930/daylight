#include "params.h"
#include <Preferences.h>
#include <string.h>

// ビルド時デフォルト。現行の実機チューニング値(2026-07-24時点、
// 閉路パターンで物理ずれ2mm・1°)と一致させてある。
const Params kDefaultParams = {
    // 車輪速度PID(MOT/DUTYコマンド用)
    .speed_kp = 300.0f,
    .speed_ki = 3000.0f,
    .speed_kd = 0.0f,
    .kf_duty_per_mps = 250.0f,
    .vbatt_nom = 8.0f,
    .speed_lpf_alpha = 0.12f,

    // その場静止制御(2026-08-02実機チューニング: kp/kiともspeed_kp/kiの
    // 4倍まで上げても発散せず安定。振動幅がエンコーダのノイズフロアで
    // 頭打ちになったためここで確定)
    .place_kp = 1200.0f,
    .place_ki = 12000.0f,
    .place_kd = 0.0f,
    .place_out_max = 400.0f,
    .place_lpf_alpha = 0.12f,

    // その場旋回(2026-08-02実機チューニング: 角度追従は誤差0.22°まで収束、
    // 振動なし。pivot_kpを当初見積もり(旧turn_kp*kf_duty_per_mps=90)から
    // 40倍の3600まで上げることで大幅改善した。旋回中の並進側drift(vsum)は
    // まだ詰め切れていない(HOLD単体では±2mm/s以内だが旋回中は±10mm/s
    // 程度残る、要継続チューニング)。
    .pivot_max_radps = 3.9f,
    .pivot_accel = 20.0f,
    .pivot_kf = 19.0f,
    .pivot_kp = 3600.0f,
    .pivot_ki = 75.0f,
    .pivot_kd = 80.0f,
    .pivot_out_max = 500.0f,
};

Params params = kDefaultParams;

const ParamDef PARAM_TABLE[] = {
    {"speed_kp", offsetof(Params, speed_kp)},
    {"speed_ki", offsetof(Params, speed_ki)},
    {"speed_kd", offsetof(Params, speed_kd)},
    {"kf_duty_per_mps", offsetof(Params, kf_duty_per_mps)},
    {"vbatt_nom", offsetof(Params, vbatt_nom)},
    {"speed_lpf_alpha", offsetof(Params, speed_lpf_alpha)},

    {"place_kp", offsetof(Params, place_kp)},
    {"place_ki", offsetof(Params, place_ki)},
    {"place_kd", offsetof(Params, place_kd)},
    {"place_out_max", offsetof(Params, place_out_max)},
    {"place_lpf_alpha", offsetof(Params, place_lpf_alpha)},

    {"pivot_max_radps", offsetof(Params, pivot_max_radps)},
    {"pivot_accel", offsetof(Params, pivot_accel)},
    {"pivot_kf", offsetof(Params, pivot_kf)},
    {"pivot_kp", offsetof(Params, pivot_kp)},
    {"pivot_ki", offsetof(Params, pivot_ki)},
    {"pivot_kd", offsetof(Params, pivot_kd)},
    {"pivot_out_max", offsetof(Params, pivot_out_max)},
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
