#!/usr/bin/env python3

import pygame


BUTTON_NAMES = {
    0: "×",
    1: "○",
    2: "△",
    3: "□",
    4: "L1",
    5: "R1",
    6: "L2",
    7: "R2",
}


DPAD_NAMES = {
    (0, 1): "上",
    (1, 0): "右",
    (0, -1): "下",
    (-1, 0): "左",
}


def main():
    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() == 0:
        print("ゲームコントローラが見つかりません")
        return

    joystick = pygame.joystick.Joystick(0)
    joystick.init()

    print(f"コントローラ: {joystick.get_name()}")
    print("入力待ち...")
    print("Ctrl+C で終了")

    try:
        while True:
            for event in pygame.event.get():

                # ボタンが押された
                if event.type == pygame.JOYBUTTONDOWN:
                    name = BUTTON_NAMES.get(
                        event.button,
                        f"Button {event.button}"
                    )

                    print(f"[押下] {name}")

                # ボタンが離された
                elif event.type == pygame.JOYBUTTONUP:
                    name = BUTTON_NAMES.get(
                        event.button,
                        f"Button {event.button}"
                    )

                    print(f"[解放] {name}")

                # 十字キー
                elif event.type == pygame.JOYHATMOTION:
                    if event.hat != 0:
                        continue

                    direction = DPAD_NAMES.get(event.value)

                    if direction is not None:
                        print(f"[押下] 十字キー {direction}")
                    else:
                        # (0, 0) は十字キーが離された状態
                        print("[解放] 十字キー")

    except KeyboardInterrupt:
        print("\n終了")

    finally:
        joystick.quit()
        pygame.quit()


if __name__ == "__main__":
    main()