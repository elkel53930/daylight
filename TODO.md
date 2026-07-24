# TODO

## mob: ball_sensor の ADC 読み取りエラー(散発)

SEN コマンド要求時に稀に以下のエラーが出る(2026-07-24 に2回観測)。

```
E (171881) adc_oneshot: adc_oneshot_get_calibrated_result(330): adc oneshot read fail
```

- ボールセンサ(IO14、ADC2)の読み取りで一過性に失敗している模様
- SEN 応答自体は返る(ball_raw=0 になる)ので実害は今のところ小さい
- 2回ともアイドル状態からの単発 SEN 要求時に発生
- 再発頻度が上がるようなら ball_sensor.cpp の ADC 設定
  (ADC2 と battery/壁センサとの共存、キャリブレーション設定)を調査する
