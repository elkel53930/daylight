#ifndef PARAMS_H
#define PARAMS_H

#include <Arduino.h>
#include <stddef.h>

// 機体ごとに調整するチューニングパラメータ一式。
//
// ビルド時デフォルトはこのファイルの kDefaultParams にまとめてあり、
// 実行時に PSET で変更・PSAVE で ESP32 内蔵フラッシュ(NVS)へ個体別に
// 永続化できる(software/mob/README.md のシリアルコマンド一覧参照)。
// パラメータごとに個別の NVS キーで保存するため、将来 Params に
// フィールドを追加/削除しても、他の調整済み値には影響しない
// (新フィールドは単にキーが無いのでデフォルトのまま動く)。
//
// フィールドを追加する場合は、この struct と下の PARAM_TABLE
// (params.cpp)の両方に追記すること。名前は ESP32 Preferences の
// キー長制限(15文字)以内の snake_case にする。
struct Params {
    // 車輪速度PID(motion_controller.cpp)
    float speed_kp;
    float speed_ki;
    float speed_kd;
    float kf_duty_per_mps;
    float vbatt_nom;
    float speed_lpf_alpha;

    // 旋回中の左右速度同期(motion_controller.cpp)
    float turn_sync_kp;
    float turn_sync_ki;
    float sync_max_corr;
    float sync_lpf_alpha;

    // 直進停止プロファイル(mob.ino updateStop)
    float final_appr_mmps;
    float stop_min_mmps;
    float stop_timeout_s;
    float backoff_dist_mm;
    float backoff_mps;
    float stop_hold_sec;

    // 低速連続動作(mob.ino latch/jog)
    float latch_mps;
    float latch_turn_mps;
    float jog_mps;
    float jog_turn_mps;

    // 急停止(mob.ino QSTP)
    float qstp_decel;

    // その場旋回(mob.ino updateTurn)
    float turn_kp;
    float turn_ki;
    float turn_kd;
    float turn_fine_rad;
    float turn_max_mps;
    float turn_min_mps;
    float turn_tol_rad;
    float turn_settle_s;
    float turn_max_retry;
    float turn_accel;

    // 直進の角度・角速度フィードバック(mob.ino updateForward等)
    float angle_fb_gain;
    float rate_fb_gain;

    // 壁センサフィードバック(mob.ino calculate_wall_*)
    float wall_threshold;
    float wall_target_ls;
    float wall_target_rs;
    float wall_tilt_gain;
    float wall_tilt_max;
    float wall_cutoff_mm;
};

// ビルド時デフォルト値(現行の実機チューニング値と一致させること)
extern const Params kDefaultParams;

// 実行時に使われるパラメータの実体
extern Params params;

struct ParamDef {
    const char* name;   // シリアルコマンドで使う名前(15文字以内)
    size_t offset;       // Params 構造体内のオフセット(offsetof)
};

extern const ParamDef PARAM_TABLE[];
extern const size_t PARAM_COUNT;

// params をビルド時デフォルトに戻す(NVSには触れない)
void params_reset_to_defaults();

// 名前で1件取得。見つからなければ false。
bool params_get(const char* name, float& out_value);

// 名前で1件を即座に変更(RAM上のみ、NVSには保存しない)。
// 未知の名前なら false。
bool params_set(const char* name, float value);

// 起動時に呼ぶ: NVSに保存済みの値があれば読み込み、無い項目は現在値
// (＝ビルド時デフォルト)のまま。
void params_begin();

// 現在の params を丸ごとNVSへ保存(パラメータ名ごとに個別キー)。
bool params_save();

// NVSから読み込みRAMへ反映(保存されていない項目は変更しない)。
bool params_load();

#endif
