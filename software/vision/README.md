# vision

カメラ画像(Picamera2 の **BGR** `numpy.ndarray`)から

1. 黄色いボールの存在検出(高速な前段判定)
2. 黄色いボールの中心座標・直径・自信度の推定(RANSAC 円フィット)
3. 赤色の壁上面(マイクロマウス)の**下端**エッジ検出

を行うライブラリ。仕様は リポジトリ直下 `vision_algorithm.md`。

## 方針

- **cv2(OpenCV)使用**: BGR→HSV 変換・マスク生成を cv2 で行い高速化
  (純 numpy 実装比で約 10 倍)。輪郭抽出・RANSAC 円/直線フィットは numpy。
- **解像度非依存**: 画像サイズは `image.shape[:2]` から取得。しきい値は
  割合ベース。
- **色判定は HSV**: RGB 値の直接比較はしない。HSV レンジは OpenCV 準拠
  (H:0-179, S:0-255, V:0-255)なので実機画像を GIMP 等で調べた値を
  そのまま `ColorRange` に使える。
- **ピクセルループ禁止**: 全て cv2 / numpy のベクトル化。

## セットアップ(cv2 の導入)

cv2 は Raspberry Pi では apt で入れる(venv は `--system-site-packages`
なので apt で入れれば venv からも見える。picamera2 と同じ方式)。

```bash
sudo apt install -y python3-opencv
software/venv/bin/python3 -c "import cv2; print(cv2.__version__)"  # 確認
```

## モジュール

| ファイル | 内容 |
|----------|------|
| `vision_types.py` | `ColorRange` / `BallEstimationResult` / 各 Config / 既定色レンジ |
| `color.py` | `bgr_to_hsv` / `make_mask` / `mask_ratio` |
| `ball.py` | `detect_yellow_ball` / `estimate_yellow_ball` / `boundary_points` |
| `wall.py` | `detect_nearest_red_wall_edge` / `lower_edge_points` |
| `vision_http_test.py` | 実機確認用 HTTP ビューア(撮影+検出結果オーバーレイ配信) |

## API

```python
from vision_types import (ColorRange, BallEstimationConfig,
                          WallEdgeDetectionConfig, DEFAULT_YELLOW, DEFAULT_RED)
from ball import detect_yellow_ball, estimate_yellow_ball
from wall import detect_nearest_red_wall_edge

# 1. 存在検出(bool)。黄色ピクセル割合 > threshold
present = detect_yellow_ball(bgr, DEFAULT_YELLOW, threshold=0.003)

# 2. 中心・直径・自信度(推定失敗なら None)
res = estimate_yellow_ball(bgr, DEFAULT_YELLOW, BallEstimationConfig(seed=0))
# res.center_x, res.center_y, res.diameter, res.confidence(=RANSAC インライア率)

# 3. 最も手前(画像最下)の赤壁下端エッジ y = a*x + b(失敗なら None)
edge = detect_nearest_red_wall_edge(bgr, DEFAULT_RED, WallEdgeDetectionConfig(seed=0))
# a, b = edge   # x=列, y=行
```

- 座標系: 原点=画像左上、x=右+、y=下+。
- 赤エッジは各列の赤色最大 y(下端)を点群にし、複数の赤領域があれば
  最も下側のクラスタを選んで RANSAC 直線フィット。垂直に近い(列幅が
  `min_column_span_px` 未満)エッジは表現不能なので `None`。
- 既定色レンジ(`DEFAULT_YELLOW` / `DEFAULT_RED`)は照明依存の暫定値。
  実機で撮った画像を見ながら調整すること。

## テスト

```bash
software/venv/bin/python3 -m unittest discover -s software/vision/tests -q
```

人工画像のみで動く(カメラ・cv2 不要)。

## 実機動作確認(HTTP ビューア)

```bash
software/venv/bin/python3 software/vision/vision_http_test.py
# ブラウザで http://<ラズパイのIP>:8080/ を開く(約0.3秒ごとに更新)
```

撮影画像に以下を重ねた JPEG を配信する:

- 左上: `Ball: YES/NO` と自信度 `conf=…`
- 推定したボール: 赤い円(中心 + 円周)と直径・中心座標
- 検出した赤エッジ: 画像左端〜右端に伸ばした線分と `y=a x+b`

オプション: `--port 8080` `--width 640` `--height 480`。

> flask は未導入のため HTTP は標準ライブラリ `http.server` を使用。flask で
> 動かしたい場合は `build_overlay_jpeg(bgr) -> bytes` をそのまま流用できる。
