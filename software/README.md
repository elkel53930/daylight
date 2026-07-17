# software/

ラズパイ上で動くコードと、外部ESP32上で動くコード(`mob/`)が同居している。

## ディレクトリ構成

| ディレクトリ | 役割 | systemdサービス |
|---|---|---|
| `default_app/` | ホーム画面アプリ(メニュー・バッテリー監視・Discord通知) | `default-ui.service` |
| `ui/` | OLED/ボタン/ブザーを提供するUIサーバー | `ui_server.service` |
| `camera/` | CSIカメラの動作確認・Discord投稿スクリプト(手動実行) | なし |
| `beacon/` | 起動時のDiscord IP通知 | (cron等で個別運用、`beacon/README.md`参照) |
| `mob/` | 外部ESP32上で動くモーター制御ファームウェア(Arduino/C++) | 対象外(ハードウェア無しでテスト不可) |

各ディレクトリの詳細は個別の `README.md` を参照。

## 開発用venvの方針

- **`software/venv`**(このディレクトリ直下、共有): `default_app/`・`ui/`・`camera/` はいずれも同じRPi固有のハードウェアライブラリ(lgpio, picamera2, luma等)に依存しているため、個別にvenvを分けても実質的な独立性は得られない。まとめて1つの `--system-site-packages` venvで管理する。

  ```bash
  sudo apt install -y python3-picamera2
  python3 -m venv --system-site-packages software/venv
  software/venv/bin/pip install -r software/requirements.txt
  ```

  `picamera2` はpipでは信頼できないため、apt側(system-site-packages)から解決する。

- **`beacon/venv`**(独立): ハードウェア依存の無い純粋なPythonスクリプトなので、`beacon/setup.sh` が作る完全に独立したpip-onlyのvenvのまま。他のディレクトリと混ぜない。

- 本番デプロイ(`default-ui.service` / `ui_server.service`)はvenvを介さず `/usr/bin/python3` を直接使う(各サービスのファイルは `/opt/...` へ個別配置される)。`software/venv` はあくまでこのリポジトリ上での開発・テスト用。

- `default_app/requirements.txt` / `ui/requirements.txt` は、それぞれを単体で `/opt/...` にデプロイする場合に必要なパッケージのドキュメントとして個別に維持している(実際にテストを動かす際は `software/venv` を使う)。

## テスト実行

```bash
software/venv/bin/python3 -m unittest discover -s default_app/tests -q
software/venv/bin/python3 -m unittest discover -s ui/tests -q
```

`camera/` はカメラ実機が無いと動作確認できないため自動テストは無い。
