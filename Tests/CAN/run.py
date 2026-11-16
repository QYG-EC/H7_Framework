#!/usr/bin/env python3
"""Compile actual BSP/Daemon/VESC sources against host HAL/RTOS mocks; no firmware cache changes."""
from pathlib import Path
import subprocess

root = Path(__file__).resolve().parents[2]
out = root / "build/host-can"
out.mkdir(parents=True, exist_ok=True)
includes = ["Tests/CAN/stubs", "User_File/Middleware/BSP/CAN",
            "User_File/System/Daemon", "User_File/Device/Peripheral/Motor/VESCmotor",
            "User_File/Device/Peripheral/Motor/EL05motor"]
flags = ["-Wall", "-Wextra", "-Werror", "-g", "-fsanitize=undefined", "-fno-sanitize-recover=all"]
flags += ["-I" + str(root / path) for path in includes]
cpp_sources = ["User_File/System/Daemon/daemon.cpp",
               "User_File/Device/Peripheral/Motor/VESCmotor/vesc_motor.cpp", "User_File/Device/Peripheral/Motor/EL05motor/el05_motor.cpp",
               "Tests/CAN/can_test.cpp"]
objects = []
for source in cpp_sources:
    obj = out / (Path(source).stem + ".o")
    subprocess.run(["c++", "-std=c++17", *flags, "-c", str(root / source), "-o", str(obj)], check=True)
    objects.append(str(obj))
for mask in (0, 1, 2, 4, 7):
    obj = out / "bsp_can.o"
    defines = [f"-DH7_CAN{bus + 1}_EXTENDED={(mask >> bus) & 1}" for bus in range(3)]
    subprocess.run(["cc", "-std=c11", *flags, *defines, "-c",
                    str(root / "User_File/Middleware/BSP/CAN/bsp_can.c"), "-o", str(obj)], check=True)
    binary = out / f"can_test_{mask}"
    subprocess.run(["c++", *flags, *objects, str(obj), "-o", str(binary)], check=True)
    for scenario in ("routing", "motor", "capacity", "race", "el05"):
        subprocess.run([str(binary), str(mask), scenario], check=True)
