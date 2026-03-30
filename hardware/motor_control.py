import socket
import time
from pynput import keyboard


class MotorController:
    def __init__(self, esp_ip='192.168.1.100', port=8888):
        self.esp_ip = esp_ip
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(1.0)

    def send(self, cmd, retries=2):
        last_error = "ERR"
        for _ in range(retries):
            try:
                self.sock.sendto(cmd.encode(), (self.esp_ip, self.port))
                data, _ = self.sock.recvfrom(1024)
                return data.decode()
            except socket.timeout:
                last_error = "ERR: TIMEOUT"
            except Exception as e:
                last_error = f"ERR: {e}"
            time.sleep(0.03)
        return last_error

    def set_speed(self, left_speed, right_speed):
        return self.send(f"SET_SPEED:{left_speed:.4f},{right_speed:.4f}", retries=1)

    def enable(self):
        return self.send("ENABLE", retries=3)

    def stop(self):
        return self.send("STOP", retries=3)

    def ping(self):
        return self.send("PING", retries=5)

    def close(self):
        self.sock.close()


pressed = {
    "up": False,
    "down": False,
    "left": False,
    "right": False,
    "esc": False,
}


def set_key(name, value):
    pressed[name] = value


def on_press(key):
    if key == keyboard.Key.esc:
        set_key("esc", True)
        return

    if key == keyboard.Key.up:
        set_key("up", True)
        return
    if key == keyboard.Key.down:
        set_key("down", True)
        return
    if key == keyboard.Key.left:
        set_key("left", True)
        return
    if key == keyboard.Key.right:
        set_key("right", True)
        return

    try:
        if key.char is None:
            return
        ch = key.char.lower()

        if ch == "w":
            set_key("up", True)
        elif ch == "s":
            set_key("down", True)
        elif ch == "a":
            set_key("left", True)
        elif ch == "d":
            set_key("right", True)
    except AttributeError:
        pass


def on_release(key):
    if key == keyboard.Key.up:
        set_key("up", False)
        return
    if key == keyboard.Key.down:
        set_key("down", False)
        return
    if key == keyboard.Key.left:
        set_key("left", False)
        return
    if key == keyboard.Key.right:
        set_key("right", False)
        return

    try:
        if key.char is None:
            return
        ch = key.char.lower()

        if ch == "w":
            set_key("up", False)
        elif ch == "s":
            set_key("down", False)
        elif ch == "a":
            set_key("left", False)
        elif ch == "d":
            set_key("right", False)
    except AttributeError:
        pass


def move_towards(current, target, step):
    if current < target - step:
        return current + step
    if current > target + step:
        return current - step
    return target


def keyboard_drive(controller):
    print("Управление:")
    print("  Up / Down / Left / Right")
    print("  или W / S / A / D")
    print("  Esc - выход")

    max_speed = 10.0
    accel = 10.0
    decel = 14.0
    dt = 0.02

    left_speed = 0.0
    right_speed = 0.0
    last_left_sent = None
    last_right_sent = None
    last_print = 0.0

    enable_resp = controller.enable()
    print("ENABLE ->", enable_resp)

    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()

    try:
        while True:
            if pressed["esc"]:
                break

            target_left = 0.0
            target_right = 0.0

            if pressed["up"] and not pressed["down"]:
                target_left += max_speed
                target_right += max_speed
            elif pressed["down"] and not pressed["up"]:
                target_left -= max_speed
                target_right -= max_speed

            if pressed["left"] and not pressed["right"]:
                target_left -= max_speed
                target_right += max_speed
            elif pressed["right"] and not pressed["left"]:
                target_left += max_speed
                target_right -= max_speed

            target_left = max(-max_speed, min(max_speed, target_left))
            target_right = max(-max_speed, min(max_speed, target_right))

            if target_left == 0.0:
                left_speed = move_towards(left_speed, 0.0, decel * dt)
            else:
                left_speed = move_towards(left_speed, target_left, accel * dt)

            if target_right == 0.0:
                right_speed = move_towards(right_speed, 0.0, decel * dt)
            else:
                right_speed = move_towards(right_speed, target_right, accel * dt)

            need_send = (
                    last_left_sent is None or
                    last_right_sent is None or
                    abs(left_speed - last_left_sent) >= 0.02 or
                    abs(right_speed - last_right_sent) >= 0.02
            )

            if need_send:
                response = controller.set_speed(left_speed, right_speed)
                last_left_sent = left_speed
                last_right_sent = right_speed
            else:
                response = "SKIP"

            now = time.monotonic()
            if now - last_print > 0.1:
                print(
                    f"\rL:{left_speed:6.2f}  R:{right_speed:6.2f}  {response:14}",
                    end="",
                    flush=True
                )
                last_print = now

            time.sleep(dt)

    finally:
        print()
        print("STOP ->", controller.stop())
        listener.stop()


if __name__ == "__main__":
    ip = input("ESP IP (default 192.168.1.100): ").strip() or "192.168.1.100"
    ctrl = MotorController(ip)

    ping_response = ctrl.ping()
    print("Connected:", ping_response)

    if ping_response == "PONG":
        keyboard_drive(ctrl)
    else:
        print("Нет связи с ESP32")

    ctrl.close()