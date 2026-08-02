# software/

ラズパイ上で動くコードと、外部ESP32上で動くコード(`mob/`)が同居している。

## ディレクトリ構成

| ディレクトリ | 役割 | systemdサービス |
|---|---|---|
| `default_app/` | ホーム画面アプリ(メニュー・バッテリー監視・Discord通知) | `default-ui.service` |
| `ui/` | OLED/ボタン/ブザーを提供するUIサーバー | `ui_server.service` |
| `camera/` | CSIカメラの動作確認・Discord投稿スクリプト(手動実行) | なし |
| `beacon/` | 起動時のDiscord IP通知 | (cron等で個別運用、`beacon/README.md`参照) |
| `vision/` | カメラ画像認識ライブラリ(黄ボール検出・赤壁エッジ検出) | なし |
| `mob/` | 外部ESP32上で動くモーター制御ファームウェア(Arduino/C++) | 対象外(ハードウェア無しでテスト不可) |

各ディレクトリの詳細は個別の `README.md` を参照。

`micromouse/`・`manual_controller/` は2026-08-02に削除(ESP32側の移動系
制御ロジックを根本から作り直すため、依存していたPythonスクリプトごと削除。
`CLAUDE.md`参照)。

## 開発用venvの方針

- **`software/venv`**(このディレクトリ直下、共有): `default_app/`・`ui/`・`camera/` はいずれも同じRPi固有のハードウェアライブラリ(lgpio, picamera2, luma等)や共通ライブラリに依存しているため、個別にvenvを分けても実質的な独立性は得られない。まとめて1つの `--system-site-packages` venvで管理する。

  ```bash
  python3 -m venv --system-site-packages software/venv
  software/venv/bin/pip install -r software/requirements.txt
  ```

  `camera/` を使う場合はさらに以下も必要(`picamera2` はpipでは信頼できないため、apt側(system-site-packages)から解決する。`default_app/`・`ui/` だけを触るなら不要):

  ```bash
  sudo apt install -y python3-picamera2
  ```

- **`beacon/venv`**(独立): ハードウェア依存の無い純粋なPythonスクリプトなので、`beacon/setup.sh` が作る完全に独立したpip-onlyのvenvのまま。他のディレクトリと混ぜない。

- 本番デプロイ(`default-ui.service` / `ui_server.service`)はvenvを介さず `/usr/bin/python3` を直接使う(各サービスのファイルは `/opt/...` へ個別配置される)。`software/venv` はあくまでこのリポジトリ上での開発・テスト用。

- `default_app/requirements.txt` / `ui/requirements.txt` は、それぞれを単体で `/opt/...` にデプロイする場合に何をインストールすべきかの参考リストとして個別に維持している(自動化されたインストール手順ではなく、手動で環境を作る際の参考。実際にこのリポジトリでテストを動かす際は `software/venv` を使う)。

## テスト実行

```bash
software/venv/bin/python3 -m unittest discover -s software/default_app/tests -q
software/venv/bin/python3 -m unittest discover -s software/ui/tests -q
software/venv/bin/python3 -m unittest discover -s software/vision/tests -q
```

`camera/` はカメラ実機が無いと動作確認できないため自動テストは無い。
