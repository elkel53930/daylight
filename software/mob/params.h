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

    // JOGFWD/JOGBACK の並進制御(place_controller.cpp)。左右輪速度の和
    // (並進成分)を目標へ追い込むPID。速度PID(speed_kp/ki)と誤差の単位・
    // 出力スケール(m/s→duty)は同じだがフィードフォワードが無いため、
    // 経路が異なる(2026-08-02新規追加、実機チューニング済み)。
    float place_kp;
    float place_ki;
    float place_kd;
    float place_out_max;
    float place_lpf_alpha;

    // JOGFWD/JOGBACK の位置制御(外側ループ)。sensors.get_distance()を
    // 目標位置へ寄せるP制御で目標並進速度を作り、上記の速度PIDへ渡す。
    float place_pos_kp;       // 位置誤差[m] → 目標速度[m/s]
    float place_pos_max_mps;  // 目標速度のクランプ

    // HOLD/TURN/JOGTURN の「並進抑制」専用パラメータ。旋回時のガタつきを
    // 抑えるため、JOG移動用とは分離して個別に弱めて調整できるようにする。
    float place_hold_kp;
    float place_hold_ki;
    float place_hold_kd;
    float place_hold_out_max;
    float place_hold_pos_kp;       // 位置誤差[m] → 目標速度[m/s]
    float place_hold_pos_max_mps;  // 目標速度のクランプ

    // その場静止制御のIMU加速度フィードフォワード。エンコーダ差分ベースの
    // 速度PID(10ms窓)より速く外乱に反応させる狙いで、IMU Y軸(ロボット
    // 前後方向、+=前進側、実機確認済み)の加速度に比例した制動を毎ms
    // 直接加える(2026-08-02新規追加)。
    float place_accel_kd;         // 加速度[m/s^2] → duty(符号は反転して加える)
    float place_accel_lpf_alpha;  // 加速度生値のLPF係数(EMA)

    // 左右輪速度同期(P制御のみ)。|vr|と|vl|の差 → duty補正。速い方を
    // 弱め遅い方を強める(2026-08-02新規追加、ユーザー指摘)。
    float place_sync_kp;       // 速度差[m/s] → duty
    float place_sync_out_max;  // 補正duty(片輪あたり)のクランプ
    // 同期誤差(mag_r-mag_l)専用の緩いLPF係数(2026-08-03追加)。車輪速度PID
    // (place_lpf_alpha)より十分小さくし、同期ループと車輪PIDの反応周波数帯を
    // 分離して結合振動(旋回中の前後発振→横drift)を防ぐ。旧motion_controllerの
    // SYNC_ERR_LPF_ALPHA=0.02(≈50ms)に倣う(c816154)。
    float place_sync_err_lpf_alpha;

    // その場旋回(place_controller.cpp、TURNコマンド)。目標角度プロファイル
    // (台形速度、pivot_max_radps/pivot_accel)とその追従PID+FF。
    // (2026-08-02新規追加。角度追従は実機チューニング済み、旋回中の並進側
    // driftは調整継続中)。
    float pivot_max_radps;  // プロファイル巡航角速度 [rad/s]
    float pivot_accel;      // プロファイル角加速度(加速・減速とも一定) [rad/s^2]
    float pivot_kf;         // FF: 目標角速度[rad/s] → duty
    float pivot_kp;         // 角度誤差[rad] → duty
    float pivot_ki;         // 角度誤差積分[rad*s] → duty
    float pivot_kd;         // 角速度誤差(ジャイロ実測との差)[rad/s] → duty
    float pivot_out_max;    // 旋回側補正duty(duty_diff)のクランプ

    // 仮想ターゲット追従によるパス走行(path_controller.cpp、PATTERNコマンド)。
    // ロボットは常に仮想ターゲットの方向を向き(角度)、path_follow_mmの
    // 距離を保つ(速度)よう追従する(2026-08-02新規追加、実機未チューニング)。
    float path_follow_mm;  // ターゲットとの目標追従距離 [mm]
    float path_accel;      // 直進セグメントの加減速度(一定) [mm/s^2]
    float path_kp_fwd;     // 距離誤差[mm] → duty(前進側)
    float path_kp_ang;     // 方位誤差[rad] → duty(旋回側)
    float path_kf_ang;     // FF: スラローム区間の幾何学的角速度[rad/s] → duty
    float path_kd_ang;     // 角速度誤差(ジャイロ実測との差)[rad/s] → duty
    float path_out_max;    // duty_common/duty_diffそれぞれのクランプ
    // 位置復元力(2026-08-02追加)。角度誤差を、追従できているときは
    // target_heading(高精度)、distが開くほどベアリング角(ロボット→ターゲット
    // 位置の方向、位置復元力)へブレンドする。path_blend_mmはdist_errorが
    // この値に達したとき完全にベアリング主体になる幅。
    float path_blend_mm;   // ベアリングへのブレンド幅 [mm]
    float path_gate_mm;    // dist_to_targetがこの値を超えたらターゲットの前進を止めて待つ [mm]
    // Kanayama式の横位置復元力(2026-08-07追加、Phase 2)。機体フレームでの
    // ターゲットの横ずれ e_y[mm] に比例する heading バイアスを常時重畳し、
    // スラローム中の横driftを向き制御で戻す(path_controller.cpp参照)。
    // path_ky>0 で有効(既定)、0以下なら従来のベアリングブレンド方式へ
    // フォールバックして後方互換を保つ。
    float path_ky;         // 横誤差[mm] → headingバイアス [rad]（Kanayama復元力ゲイン）
    float path_ky_max;     // 横誤差バイアスのクランプ [rad]
    // 側壁センサによる横位置補正(壁追従、2026-08-03追加)。直進セグメントで
    // 両側に壁があるとき、ls/rs差から機体を迷路中心へ寄せる微小な heading
    // バイアスを重畳する。側壁センサは角度に敏感なので near-straight のときだけ
    // 信頼でき、ゲインは小さくバイアスはクランプする。既定 path_wall_kp=0(無効)。
    float path_wall_kp;       // (rs-ls)[sensor unit] → heading バイアス [rad]
    float path_wall_present;  // ls,rs がこれ以上で「側壁あり」とみなす
    float path_wall_bias_max; // heading バイアスのクランプ [rad]
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
