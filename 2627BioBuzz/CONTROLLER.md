# Controller Controls — FTC 32722 BioBuzz

![PlayStation Controller Reference](doc/media/controller.svg)

> Edit this file directly whenever you change button bindings in [`DriverControlled.java`](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/teleop/DriverControlled.java).
> Numbers **1–13** match the labels on the PlayStation controller diagram above.

---

## Button Mapping

| # | PlayStation Button | FTC SDK Field | Function | Notes |
| :-: | --- | --- | --- | --- |
| **1** | **L2 Trigger** | `gamepad.left_trigger` | **Global Emergency Stop** *(with R2)* | Pull **L2 + R2** > 85% for 150 ms (`gamepad1` or `gamepad2`, all OpModes) |
| **2** | **L1 Bumper** | `gamepad.left_bumper` | **Slow Speed (50%)** | Sets drive speed multiplier to `0.5` |
| **3** | **D-Pad** | `gamepad.dpad_*` | *Unassigned* | — |
| **4** | **Left Stick (`L3`)** | `left_stick_x / y` | **Mecanum:** Forward/Back (`Y`) + Strafe (`X`)<br>**Tank (legacy):** Left drive (`Y`) | 5% deadzone (`GAMEPAD_DEADZONE`) |
| **5** | **SHARE** | `gamepad.back` | **Clear / Reset All E-Stops** | Hold for `0.5 s` on `gamepad1` or `gamepad2` |
| **6** | **OPTIONS** | `gamepad.start` | *Reserved by FTC Driver Station* | `OPTIONS + ✕` / `OPTIONS + ○` assigns Gamepad 1 / 2 |
| **7** | **R2 Trigger** | `gamepad.right_trigger` | **Global Emergency Stop** *(with L2)* | Latches all motors to `0` until **SHARE** is held `0.5 s` |
| **8** | **R1 Bumper** | `gamepad.right_bumper` | **Fast Speed (100%)** | Sets drive speed multiplier to `1.0` |
| **9** | **△ Triangle** | `gamepad.y` | **Intake Reverse** | Runs a `200 ms` reverse pulse to clear jams |
| **10** | **○ Circle** | `gamepad.b` | **Reset Odometry Pose** | Sets `(X, Y, Heading)` to `(0, 0, 0°)` in Mecanum+Odo mode |
| **11** | **✕ Cross** | `gamepad.a` | **Toggle FlyWheel** | If FlyWheel Auto E-Stop is active, 1st press clears the latch safely |
| **12** | **□ Square** | `gamepad.x` | **Toggle Intake** | If Intake Auto E-Stop is active, 1st press clears the latch safely |
| **13** | **Right Stick (`R3`)** | `right_stick_x / y` | **Mecanum:** Turn Left/Right (`X`)<br>**Tank (legacy):** Right drive (`Y`) | 5% deadzone (`GAMEPAD_DEADZONE`) |

---

## Automatic Subsystem E-Stops

- **FlyWheel**: Auto E-Stops on `STALLED`, `ENCODER_FAULT`, or `OVERCURRENT`. Press **`✕` Cross** once to clear (stays off), then press **`✕`** again to spin up.
- **Intake**: Auto E-Stops if automatic jam recovery (`REVERSE` → `SUPER`) fails, or on `ENCODER_FAULT` / `OVERCURRENT`. Press **`□` Square** once to clear (stays off), then press **`□`** again to enable.
- **DriveTrain**: Auto E-Stops on sustained `OVERCURRENT` (`AUTO_ESTOP_DRIVETRAIN_ON_OVERCURRENT = true` in [`RobotConstants.java`](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/config/RobotConstants.java)). Hold **`SHARE`** for `0.5 s` to clear.

---

## Drive Mode (`MECANUM_ODO_INSTALLED`)

Configured in [`RobotConstants.java`](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/config/RobotConstants.java):
- `false` *(current default)*: Legacy 2-motor Tank drive (`Left Stick Y` = left, `Right Stick Y` = right).
- `true`: 4-motor Mecanum drive + 2-pod Odometry (`Left Stick` = move/strafe, `Right Stick X` = turn, `○ Circle` = reset pose).
