#!/usr/bin/env python3
"""Hardware check for the SO-101 bus bridge - run this BEFORE launching ROS.

  pip install pyserial
  python3 servo_check.py /dev/ttyUSB0 --calib ~/.cache/huggingface/lerobot/calibration/robots/so101_follower/<id>.json
  python3 servo_check.py /dev/ttyUSB0 --calib calib.json --watch        # live read-out, move the arm by hand
  python3 servo_check.py /dev/ttyUSB0 --torque-off                      # release all servos

It only reads (and, with --torque-off, disables torque). It never commands motion.
"""
import argparse, json, math, sys, time
import serial

def chk(b): return (~sum(b)) & 0xFF
def packet(i, inst, params=b""):
    body = bytes([i, len(params) + 2, inst]) + bytes(params)
    return b"\xff\xff" + body + bytes([chk(body)])

class Bus:
    def __init__(self, port, baud, echo=False):
        self.s = serial.Serial(port, baud, timeout=0.05)
        self.echo = echo
        time.sleep(2.0)                 # ESP32 resets when the port opens
        self.s.reset_input_buffer()
    def _tx(self, pkt):
        self.s.reset_input_buffer(); self.s.write(pkt)
        if self.echo: self.s.read(len(pkt))
    def _rx_status(self, i):
        buf = bytearray(self.s.read(64))
        while len(buf) >= 4:
            if buf[0] == 0xFF and buf[1] == 0xFF and buf[2] != 0xFF:
                n = buf[3]
                if len(buf) >= n + 4 and chk(buf[2:n + 3]) == buf[n + 3]:
                    if buf[2] == i: return buf[4], bytes(buf[5:n + 3])
                    del buf[:n + 4]; continue
                if len(buf) < n + 4: buf += self.s.read(n + 4 - len(buf)); continue
            del buf[0]
        return None
    def ping(self, i):
        self._tx(packet(i, 0x01)); return self._rx_status(i) is not None
    def read(self, i, addr, n):
        self._tx(packet(i, 0x02, [addr, n])); r = self._rx_status(i)
        return None if r is None else r[1]
    def write(self, i, addr, data):
        self._tx(packet(i, 0x03, [addr] + list(data))); return self._rx_status(i) is not None

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port"); ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--calib"); ap.add_argument("--watch", action="store_true")
    ap.add_argument("--torque-off", action="store_true"); ap.add_argument("--echo", action="store_true")
    a = ap.parse_args()

    cal = {}
    if a.calib:
        for name, c in json.load(open(a.calib)).items():
            # arm joints: zero = middle of recorded range; gripper: zero = fully closed (range_min)
            cal[c["id"]] = (name, c["range_min"] if name == "gripper" else (c["range_min"] + c["range_max"]) / 2.0)

    bus = Bus(a.port, a.baud, a.echo)
    ids = [1, 2, 3, 4, 5, 6]
    print("Ping:")
    alive = []
    for i in ids:
        ok = any(bus.ping(i) for _ in range(3)); alive.append(ok)
        print(f"  id {i}: {'OK' if ok else 'NO ANSWER'}")
    if not all(alive):
        sys.exit("Not all servos answered: check power, wiring, firmware pins and baud rate.")

    if a.torque_off:
        for i in ids: bus.write(i, 40, [0])
        print("Torque disabled on all servos."); return

    def show():
        for i in ids:
            d = bus.read(i, 56, 8)
            if d is None: print(f"  id {i}: read failed"); continue
            pos = d[0] | (d[1] << 8); volt = d[6] / 10.0; temp = d[7]
            name, zero = cal.get(i, (f"id{i}", 2048.0))
            rad = (pos - zero) * 2 * math.pi / 4096
            print(f"  {name:<14} id{i}  ticks={pos:4d}  {math.degrees(rad):7.1f} deg  {rad:7.3f} rad   {volt:.1f} V  {temp} C")
    modes = [bus.read(i, 33, 1) for i in ids]
    print("Operating modes (expect all 0 = position):", [m[0] if m else None for m in modes])
    if a.watch:
        try:
            while True:
                print("\033[2J\033[H", end=""); show(); time.sleep(0.1)
        except KeyboardInterrupt: pass
    else:
        print("Positions:"); show()

if __name__ == "__main__":
    main()
