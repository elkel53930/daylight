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

    // 位置ホールド(外側ループ、2026-08-02追加)。速度(左右輪速度の和)の
    // 平均をゼロにするだけでは位置の復元力が無くドリフトしうるため追加。
    // target_v_sum は左右輪速度の「和」なので機体前後速度はこの半分。
    // 2026-08-05: JOGFWD/JOGBACK が本体≈25mm/sと遅く低速で制御性が悪い
    // (静止摩擦・バックラッシで動きがぎこちない)ため速度を引き上げ:
    // kp 2.0→6.0、max 0.05→0.14(和=0.14 → 本体巡航≈70mm/s、従来25mm/sの
    // 約3倍)。PSETライブ掃引(4.0/0.10・6.0/0.14・8.0/0.18)で60mm往復を比較し、
    // 6.0/0.14 を選定: 60mm往復が総2.3s→1.6s、巡航≈75mm/s、オーバーシュート無し・
    // 方向逆行0。HOLD の位置復元にも共用されるため静止保持(p2p 0.11mm)と
    // 旋回後保持(並進 p2p 0.26mm)も発振しないことを実機確認済み。
    .place_pos_kp = 6.0f,
    .place_pos_max_mps = 0.14f,

    // IMU Y軸(ロボット前後方向、+=前進)加速度フィードフォワード
    // (2026-08-02追加)。静止時(HOLD)のみ有効、旋回中は無効
    // (place_controller.cpp参照: IMUが回転中心からオフセットしており、
    // 旋回中は向心・接線加速度が混入して大暴走した実機事象があるため)。
    // ゲイン自体は初期値のまま静止時に発散しないことのみ確認済み。
    .place_accel_kd = 20.0f,
    .place_accel_lpf_alpha = 0.3f,

    // 左右輪速度同期(P制御のみ、2026-08-02実機チューニング)。300では
    // 旋回中の急激な速度変化に対して逆に外乱を増やした(vsum -24〜+19mm/s)
    // ため、100まで下げて改善を確認(-10〜+11mm/s、sync無し同等以上)。
    .place_sync_kp = 100.0f,
    .place_sync_out_max = 200.0f,
    .place_sync_err_lpf_alpha = 0.02f,  // ≈50ms。車輪PID(0.12)と周波数帯を分離

    // その場旋回(2026-08-02実機チューニング: 角度追従は誤差0.22°まで収束、
    // 振動なし。pivot_kpを当初見積もり(旧turn_kp*kf_duty_per_mps=90)から
    // 40倍の3600まで上げることで大幅改善した。旋回中の並進側drift(vsum)は
    // まだ詰め切れていない(HOLD単体では±2mm/s以内だが旋回中は±10mm/s
    // 程度残る、要継続チューニング)。
    // pivot_max_radpsは当初3.9だったが2倍の7.8に変更(90度旋回では元々
    // 90度分の角度でこの上限に到達しない=ピーク角速度5.4rad/s程度に
    // 留まるため実質的な影響は緩やか、収束誤差0.24°・vsumも悪化なしを
    // 実機確認済み)。pivot_accelは1.5倍の30を試したが、実機で目視すると
    // 振動気味だったため20に戻した(2026-08-02)。
    .pivot_max_radps = 7.8f,
    .pivot_accel = 20.0f,
    .pivot_kf = 19.0f,
    .pivot_kp = 3600.0f,
    .pivot_ki = 75.0f,
    .pivot_kd = 80.0f,
    .pivot_out_max = 500.0f,

    // 仮想ターゲット追従パス走行。角度誤差は2026-08-02にベアリング角
    // (ロボットから見たターゲットの方向)からターゲット自身の向きとの
    // 差へ変更し、方位誤差の最大値が21°→5°程度まで大幅改善(旧
    // kp_ang=1000/kf_ang=19/kd_ang=80のデフォルト値のままで達成)。
    // それでも実機でガタつきが残るとのことで角度系ゲインを段階的に
    // 0.75倍ずつ下げている(1000/19/80→750/14.25/60→562.5/10.6875/45→
    // 421.875/8.015625/33.75、計0.421875倍、2026-08-02、要検証)。
    // 0.75倍(2段目)の時点では位置精度が悪化(誤差6mm/1mm→16mm/19mm)
    // したが、さらに0.75倍(3段目)したところ位置精度は7mm/7mmまで回復
    // した(ノイズの可能性もあり要再検証)。
    .path_follow_mm = 30.0f,
    .path_accel = 1000.0f,  // 2026-08-02: PATTERNの巡航速度300mm/s(実機で高精度を確認)に合わせて2倍のまま
    .path_kp_fwd = 3.0f,
    .path_kp_ang = 421.875f,
    .path_kf_ang = 8.015625f,
    .path_kd_ang = 33.75f,
    .path_out_max = 500.0f,
    // 位置復元力(2026-08-02追加)。90°ターンは追従できるが、機体の向きが
    // 進行方向から90°を超えて回る(180°/Uターン)とtarget_heading制御だけでは
    // 位置を回復できず発散する実機事象への対策。dist_errorがpath_blend_mmに
    // 達するとベアリング角主体に切り替え、path_gate_mmを超えたらターゲットを
    // 止めて追いつくのを待つ。正常追従(dist~30、直進の加減速で最大~55)では
    // ブレンドはごく僅か・ゲートは非発動。実機で要チューニング。
    .path_blend_mm = 40.0f,
    .path_gate_mm = 90.0f,
    .path_wall_kp = 0.0f,        // 既定は無効。PSETでライブ調整して有効化する。
    .path_wall_present = 150.0f,
    .path_wall_bias_max = 0.10f, // ≈5.7° まで
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

    {"place_pos_kp", offsetof(Params, place_pos_kp)},
    {"place_pos_max_mps", offsetof(Params, place_pos_max_mps)},

    {"place_accel_kd", offsetof(Params, place_accel_kd)},
    {"place_accel_lpf_alpha", offsetof(Params, place_accel_lpf_alpha)},

    {"place_sync_kp", offsetof(Params, place_sync_kp)},
    {"place_sync_out_max", offsetof(Params, place_sync_out_max)},
    {"place_sync_err_lpf_alpha", offsetof(Params, place_sync_err_lpf_alpha)},

    {"pivot_max_radps", offsetof(Params, pivot_max_radps)},
    {"pivot_accel", offsetof(Params, pivot_accel)},
    {"pivot_kf", offsetof(Params, pivot_kf)},
    {"pivot_kp", offsetof(Params, pivot_kp)},
    {"pivot_ki", offsetof(Params, pivot_ki)},
    {"pivot_kd", offsetof(Params, pivot_kd)},
    {"pivot_out_max", offsetof(Params, pivot_out_max)},

    {"path_follow_mm", offsetof(Params, path_follow_mm)},
    {"path_accel", offsetof(Params, path_accel)},
    {"path_kp_fwd", offsetof(Params, path_kp_fwd)},
    {"path_kp_ang", offsetof(Params, path_kp_ang)},
    {"path_kf_ang", offsetof(Params, path_kf_ang)},
    {"path_kd_ang", offsetof(Params, path_kd_ang)},
    {"path_out_max", offsetof(Params, path_out_max)},
    {"path_blend_mm", offsetof(Params, path_blend_mm)},
    {"path_gate_mm", offsetof(Params, path_gate_mm)},
    {"path_wall_kp", offsetof(Params, path_wall_kp)},
    {"path_wall_present", offsetof(Params, path_wall_present)},
    {"path_wall_bias_max", offsetof(Params, path_wall_bias_max)},
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
