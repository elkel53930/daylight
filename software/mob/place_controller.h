#ifndef PLACE_CONTROLLER_H
#define PLACE_CONTROLLER_H

#include <Arduino.h>
#include "motor.h"
#include "sensors.h"

// 「その場」(=並進せずに同じ位置にとどまる)制御をまとめるクラス。
// start() はその場に静止するだけ、start_turn() はその場旋回(超信地旋回)
// する。並進側は位置制御(外側ループ)+速度制御(内側ループ)のカスケード
// 構成: sensors.get_distance()(1ms毎に常時更新されるオドメトリ、
// place_controller自身の10ms窓推定とは別の独立した積分値)を開始時の値に
// 保つP制御が目標並進速度を作り、それを2026-08-02実機チューニング済みの
// 速度PID(place_kp/ki/kd)が追従する。当初は速度(左右輪速度の和)だけを
// ゼロへ追い込む制御だったが、瞬間速度の平均がゼロでも位置がゆっくり
// ドリフトしうる(復元力が無い)ため、位置フィードバックを追加した
// (2026-08-02)。start_turn() ではこれに加えて、目標角度(角加速度一定の
// 台形速度プロファイルで滑らかに変化させる)への追従制御を重ね合わせる。
// 並進側の制御が旋回中も動き続けることで、旋回によって機体の位置が
// ずれるのを防ぐ狙い。
//
// 出力(duty)は「並進側の補正(duty_common、左右同じ値)」「旋回側の補正
// (duty_diff、左右逆符号)」「IMU加速度FF(duty_accel_ff、左右同じ値)」
// 「左右輪速度の大きさを揃えるP補正(sync_r/sync_l、左右で別々の値)」を
// 独立に計算してから単純加算する:
//   duty_r = duty_common + duty_diff + duty_accel_ff + sync_r
//   duty_l = duty_common - duty_diff + duty_accel_ff + sync_l
// (duty_diff > 0 で左/CCW方向に回転する。turn_in_place() 等、旧実装と
// 同じ符号規約)。duty_common/duty_diff/syncはエンコーダ差分ベースで
// 10ms窓でしか更新されないのに対し、duty_accel_ffはIMU加速度(ロボット
// 前後方向、+=前進側)を使い毎ms計算するため、外乱への反応が速い
// (2026-08-02追加)。
//
// sync_r/sync_lは、duty_common・duty_diffが左右へ「同じ値」または
// 「符号だけ逆の値」を与える前提(モーターが左右対称に応答する前提)を
// 補うもの。実際はモーター個体差で同じdutyでも実速度が揃わないことが
// あり、たとえvsum(和)がゼロでも機体がゆっくり回頭してしまう
// (並進側の制御からは検出できない位置ズレの原因になる、2026-08-02
// ユーザー指摘)。左右輪速度の大きさ|vr|・|vl|をP制御で揃え、速い方を
// 弱め遅い方を強める形で常時(HOLD中もTURN中も)補正する。
//
// motion_controller.cpp とは独立の、根本から作り直す新しい制御系列。
class PlaceController {
public:
    PlaceController(Motor& motor, Sensors& sensors);

    // その場静止を開始する(旋回はしない)。
    void start();

    // その場旋回を開始する: 現在角度を基準に delta_angle_rad(正=左/CCW、
    // 単位rad)だけ回転する目標角度プロファイルを生成して追従する。
    // 並進速度ゼロを保つ制御(start()と同じ)も同時に動く。
    // as_jog=true のとき「JOGTURN」として扱い、目標角へ到達・整定したら
    // 一度だけ take_jog_arrived() が true を返す(=完了通知に使う。到達後も
    // 角度・位置は保持し続ける)。既定 false は従来のTURN(完了通知なし)。
    void start_turn(float delta_angle_rad, bool as_jog = false);

    // JOGFWD/JOGBACK: 現在のヨー(向き)を保持したまま、前後に delta_dist_mm
    // (正=前進、負=後退)だけ並進する。並進側の位置P制御(place_pos_kp/
    // place_pos_max_mpsで低速クランプ)が pos_ref を delta ぶんずらして
    // 滑らかに到達させ、旋回制御は開始時の向きを保持する。到達・整定で
    // 一度だけ take_jog_arrived() が true を返す(到達後も位置・向きを保持)。
    void start_move(float delta_dist_mm);

    // JOG(start_move/start_turn as_jog)が目標へ到達・整定した瞬間に一度だけ
    // true を返す(ラッチをクリアして返す)。mob.ino が毎tickポーリングして
    // DONE を送るのに使う。到達後も制御は保持を続ける(停止はMOT,0,0)。
    bool take_jog_arrived();

    // 1kHzループから呼ぶ(motion_state == PLACE_HOLD の間のみ)。
    void update(float dt_s);

    // モーターを止めて終了する。
    void stop();

    bool is_turning() const { return turning_; }

    // テレメトリ用
    float get_vr_mps() const { return vr_filt_mps_; }
    float get_vl_mps() const { return vl_filt_mps_; }
    float get_v_sum_mps() const { return vr_filt_mps_ + vl_filt_mps_; }
    int16_t get_duty() const { return last_duty_; }
    float get_target_angle_rad() const { return turn_target_angle_rad_; }
    float get_omega_target_radps() const { return turn_omega_signed_radps_; }
    int16_t get_duty_diff() const { return last_duty_diff_; }
    float get_pos_error_mm() const { return pos_ref_mm_ - sensors_.get_distance(); }

private:
    // 速度推定を1msごとの生エンコーダ差分から行うと、AS5047の分解能
    // (16384カウント/回転、23.4mm径ホイールで1カウント≈0.0045mm)に対し
    // 1msという時間窓が短すぎ、量子化ノイズ1カウントだけで見かけ上
    // ±4.5mm/s相当の速度ノイズになる(2026-08-02実機、vsumが±15〜20mm/s
    // で振動しゲインを上げても縮まらなかった原因)。速度推定と制御更新の
    // 周期を10ms窓にまとめ、同じ量子化誤差1カウントの影響を約1/10に
    // 圧縮する(制御対象の物理的な時定数に対して10ms=100Hzは十分速い)。
    static constexpr uint8_t VELOCITY_WINDOW_TICKS = 10;
    uint8_t window_tick_count_ = 0;
    float accum_dt_s_ = 0.0f;

    Motor& motor_;
    Sensors& sensors_;

    uint16_t prev_r_angle_ = 0;
    uint16_t prev_l_angle_ = 0;
    bool have_prev_ = false;

    // LPF後の測定車輪速度 [m/s](符号規約はmotion_controller.cppと同じ:
    // 前進方向が正になるよう左輪のみエンコーダ差分を反転して使う)。
    float vr_filt_mps_ = 0.0f;
    float vl_filt_mps_ = 0.0f;

    // 位置ホールドの基準(start()/start_turn()実行時のsensors.get_distance()、mm)
    float pos_ref_mm_ = 0.0f;

    // 並進速度(vr+vl)を目標値(位置P制御の出力)へ追い込むPID
    float integ_ = 0.0f;
    float prev_err_ = 0.0f;

    int16_t last_duty_ = 0;       // 並進側の最終duty(テレメトリ用)
    int16_t last_duty_diff_ = 0;  // 旋回側の最終duty差分(テレメトリ用)
    float last_duty_common_f_ = 0.0f;  // 並進側duty(10ms窓、次tickまで保持してduty_diff/加速度FFと合成)
    float last_duty_diff_f_ = 0.0f;    // 旋回側duty(10ms窓、同上)
    float last_sync_r_f_ = 0.0f;       // 左右輪速度同期の右輪側補正(10ms窓、同上)
    float last_sync_l_f_ = 0.0f;       // 左右輪速度同期の左輪側補正(10ms窓、同上)
    // wheel_sync誤差(mag_r-mag_l)の専用LPF値(2026-08-03追加)。車輪速度PIDと
    // 同じ生の速度信号(place_lpf_alpha≈0.12=速い)に反応すると結合振動し、
    // 旋回中に前後発振→横drift になる(旧motion_controllerで実証・解決済、
    // c816154)。同期ループ専用にさらに緩いLPFをかけ低周波の定常ずれのみに
    // 反応させ、車輪PIDと周波数帯を分離する。
    float sync_err_filt_ = 0.0f;

    // IMU Y軸(前後方向)加速度のLPF後の値 [m/s^2](2026-08-02追加)
    float accel_filt_mps2_ = 0.0f;

    // その場旋回の状態(start_turn()で初期化)
    bool turning_ = false;
    float turn_dir_ = 1.0f;               // +1: 左(CCW)へ回転中, -1: 右(CW)
    float turn_start_angle_rad_ = 0.0f;   // プロファイル開始時の角度
    float turn_goal_angle_rad_ = 0.0f;    // プロファイルの最終到達角度
    float turn_target_angle_rad_ = 0.0f;  // 現在の目標角度(台形プロファイルで進行)
    float turn_omega_mag_ = 0.0f;         // 現在の目標角速度の大きさ[rad/s]
    float turn_omega_signed_radps_ = 0.0f; // 符号付き目標角速度(テレメトリ用)
    float turn_integ_ = 0.0f;             // 角度誤差の積分項

    // JOG(距離/角度指定で到達したら完了通知する)状態。NONE=通常のHOLD/TURN
    // (完了通知しない)。MOVE=start_move(位置到達で完了)。TURN=start_turn
    // (as_jog、角度到達で完了)。到達を検出したら kind を NONE に戻して
    // 保持を継続しつつ arrived ラッチを立てる。
    enum class JogKind { NONE, MOVE, TURN };
    JogKind jog_kind_ = JogKind::NONE;
    bool jog_arrived_latch_ = false;

    static int16_t calc_delta_14bit(uint16_t now, uint16_t prev);
    static float counts_to_m(int16_t delta_counts);

    void reset_common();
    float update_translational(float dt_s);
    float update_turn_profile_and_track(float dt_s);
    void update_wheel_sync(float dt_s);
};

#endif
