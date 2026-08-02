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
    // 車輪速度PID(motion_controller.cpp、MOT/DUTYコマンド用)
    float speed_kp;
    float speed_ki;
    float speed_kd;
    float kf_duty_per_mps;
    float vbatt_nom;
    float speed_lpf_alpha;

    // その場静止制御(place_controller.cpp)。左右輪速度の和(並進成分)を
    // ゼロへ追い込むPID。速度PID(speed_kp/ki)と誤差の単位・出力スケール
    // (m/s→duty)は同じだがフィードフォワードが無いため、経路が異なる
    // (2026-08-02新規追加、実機チューニング済み)。
    float place_kp;
    float place_ki;
    float place_kd;
    float place_out_max;
    float place_lpf_alpha;
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
