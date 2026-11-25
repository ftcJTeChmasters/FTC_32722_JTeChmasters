"""
Regenerates doc/media/controller.svg (PlayStation DualSense/DualShock layout)
and CONTROLLER.md from doc/controller-map.json.

Usage:
    python doc/generate_controller.py
"""

from __future__ import annotations

import html
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
CONFIG_PATH = ROOT / "doc" / "controller-map.json"
SVG_PATH = ROOT / "doc" / "media" / "controller.svg"
MD_PATH = ROOT / "CONTROLLER.md"

CATEGORY_STYLES = {
    "estop": {"stroke": "#EF4444", "fill": "#450A0A", "title": "#FCA5A5", "sub": "#FECACA"},
    "estop_reset": {"stroke": "#FACC15", "fill": "#1E293B", "title": "#FEF08A", "sub": "#CBD5E1"},
    "drive": {"stroke": "#A78BFA", "fill": "#1E293B", "title": "#DDD6FE", "sub": "#CBD5E1"},
    "intake": {"stroke": "#34D399", "fill": "#1E293B", "title": "#6EE7B7", "sub": "#CBD5E1"},
    "flywheel": {"stroke": "#FB923C", "fill": "#1E293B", "title": "#FDBA74", "sub": "#CBD5E1"},
    "odometry": {"stroke": "#38BDF8", "fill": "#1E293B", "title": "#7DD3FC", "sub": "#CBD5E1"},
    "unassigned": {"stroke": "#475569", "fill": "#1E293B", "title": "#94A3B8", "sub": "#64748B"},
}


def esc(text: str) -> str:
    return html.escape(text, quote=True)


def callout_box(
    callout_id: str,
    button_data: dict,
    x: int,
    y: int,
    w: int,
    h: int,
    points: str,
    dot_x: int,
    dot_y: int,
) -> str:
    cat = button_data.get("category", "unassigned")
    style = CATEGORY_STYLES.get(cat, CATEGORY_STYLES["unassigned"])
    dash = ' stroke-dasharray="4 3"' if cat == "unassigned" else ""
    title = esc(button_data["title"])
    subtitle = esc(button_data["subtitle"])
    return f"""  <!-- {callout_id}: EDIT LABEL IN doc/controller-map.json OR DIRECTLY BELOW -->
  <g id="callout-{callout_id}">
    <polyline points="{points}" fill="none" stroke="{style['stroke']}" stroke-width="2"{dash}/>
    <circle cx="{dot_x}" cy="{dot_y}" r="3.5" fill="{style['stroke']}"/>
    <rect x="{x}" y="{y}" width="{w}" height="{h}" rx="8" fill="{style['fill']}" stroke="{style['stroke']}" stroke-width="1.8"/>
    <text x="{x + 12}" y="{y + 20}" fill="{style['title']}" font-size="12.5" font-weight="700">{title}</text>
    <text x="{x + 12}" y="{y + 37}" fill="{style['sub']}" font-size="11">{subtitle}</text>
  </g>"""


def build_svg(config: dict) -> str:
    b = config["buttons"]
    robot_name = esc(config["robotName"])
    subtitle = esc(config["subtitle"])

    callouts = "\n\n".join([
        # Top callouts
        callout_box("SHARE", b["SHARE"], 265, 80, 260, 48, "470,214 470,128", 470, 214),
        callout_box("OPTIONS", b["OPTIONS"], 635, 80, 260, 48, "690,214 690,128", 690, 214),
        # Left column (top to bottom)
        callout_box("L2", b["L2"], 20, 128, 232, 50, "370,148 252,148", 370, 148),
        callout_box("L1", b["L1"], 20, 192, 232, 50, "352,178 290,178 252,217", 352, 178),
        callout_box("DPAD", b["DPAD"], 20, 256, 232, 50, "358,268 290,268 252,281", 358, 268),
        callout_box("LEFT_STICK", b["LEFT_STICK"], 20, 330, 232, 52, "446,358 252,358", 446, 358),
        # Right column (top to bottom)
        callout_box("R2", b["R2"], 908, 128, 232, 50, "790,148 908,148", 790, 148),
        callout_box("R1", b["R1"], 908, 192, 232, 50, "808,178 870,178 908,217", 808, 178),
        callout_box("TRIANGLE", b["TRIANGLE"], 908, 250, 232, 48, "783,228 865,228 908,274", 783, 228),
        callout_box("CIRCLE", b["CIRCLE"], 908, 306, 232, 48, "823,268 870,268 908,330", 823, 268),
        callout_box("CROSS", b["CROSS"], 908, 362, 232, 48, "783,308 865,308 908,386", 783, 308),
        callout_box("RIGHT_STICK", b["RIGHT_STICK"], 908, 418, 232, 52, "714,358 750,358 750,444 908,444", 714, 358),
        # Bottom-center callout for Square
        callout_box("SQUARE", b["SQUARE"], 455, 444, 250, 50, "725,286 725,469 705,469", 725, 286),
    ])

    return f"""<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 1160 660" width="1160" height="660" font-family="Segoe UI, Arial, sans-serif">
  <defs>
    <linearGradient id="bgGrad" x1="0" y1="0" x2="0" y2="1">
      <stop offset="0%" stop-color="#0F172A"/>
      <stop offset="100%" stop-color="#0B1120"/>
    </linearGradient>
    <linearGradient id="bodyGrad" x1="0" y1="0" x2="0" y2="1">
      <stop offset="0%" stop-color="#334155"/>
      <stop offset="45%" stop-color="#1E293B"/>
      <stop offset="100%" stop-color="#0F172A"/>
    </linearGradient>
    <linearGradient id="touchGrad" x1="0" y1="0" x2="0" y2="1">
      <stop offset="0%" stop-color="#1E293B"/>
      <stop offset="100%" stop-color="#0F172A"/>
    </linearGradient>
    <linearGradient id="stickGrad" x1="0" y1="0" x2="0" y2="1">
      <stop offset="0%" stop-color="#475569"/>
      <stop offset="100%" stop-color="#1E293B"/>
    </linearGradient>
    <filter id="shadow" x="-10%" y="-10%" width="120%" height="120%">
      <feDropShadow dx="0" dy="6" stdDeviation="8" flood-color="#000000" flood-opacity="0.45"/>
    </filter>
  </defs>

  <!-- Background -->
  <rect x="0" y="0" width="1160" height="660" rx="16" fill="url(#bgGrad)" stroke="#1E293B" stroke-width="2"/>

  <!-- Header -->
  <text x="580" y="38" text-anchor="middle" fill="#F8FAFC" font-size="22" font-weight="700">{robot_name} — Controller Layout</text>
  <text x="580" y="60" text-anchor="middle" fill="#94A3B8" font-size="13">{subtitle}</text>

  <!-- ==================== PLAYSTATION CONTROLLER SILHOUETTE ==================== -->
  <g id="playstation-controller" filter="url(#shadow)">
    <!-- L2 Trigger -->
    <path d="M 358 170 L 368 142 Q 373 132 388 132 L 424 132 Q 437 132 441 142 L 446 170 Z" fill="#7F1D1D" stroke="#EF4444" stroke-width="2"/>
    <text x="402" y="156" text-anchor="middle" fill="#FECACA" font-size="12.5" font-weight="700">L2</text>

    <!-- R2 Trigger -->
    <path d="M 714 170 L 719 142 Q 723 132 736 132 L 772 132 Q 787 132 792 142 L 802 170 Z" fill="#7F1D1D" stroke="#EF4444" stroke-width="2"/>
    <text x="758" y="156" text-anchor="middle" fill="#FECACA" font-size="12.5" font-weight="700">R2</text>

    <!-- L1 Bumper -->
    <rect x="352" y="166" width="96" height="24" rx="10" fill="#312E81" stroke="#818CF8" stroke-width="2"/>
    <text x="400" y="182" text-anchor="middle" fill="#E0E7FF" font-size="12" font-weight="700">L1</text>

    <!-- R1 Bumper -->
    <rect x="712" y="166" width="96" height="24" rx="10" fill="#312E81" stroke="#818CF8" stroke-width="2"/>
    <text x="760" y="182" text-anchor="middle" fill="#E0E7FF" font-size="12" font-weight="700">R1</text>

    <!-- DualSense / DualShock Main Body -->
    <path d="
      M 378 188
      C 334 188, 306 218, 298 265
      L 278 398
      C 270 448, 288 492, 326 495
      C 356 498, 382 465, 412 406
      C 424 388, 444 384, 474 384
      L 686 384
      C 716 384, 736 388, 748 406
      C 778 465, 804 498, 834 495
      C 872 492, 890 448, 882 398
      L 862 265
      C 854 218, 826 188, 782 188
      Z"
      fill="url(#bodyGrad)" stroke="#475569" stroke-width="2.5"/>

    <!-- Center Touchpad -->
    <rect x="498" y="196" width="164" height="78" rx="12" fill="url(#touchGrad)" stroke="#475569" stroke-width="2"/>
    <line x1="504" y1="202" x2="656" y2="202" stroke="#38BDF8" stroke-width="2" stroke-opacity="0.55"/>
    <text x="580" y="240" text-anchor="middle" fill="#64748B" font-size="10.5" font-weight="600">TOUCHPAD</text>

    <!-- SHARE Button (Left of Touchpad) -->
    <rect x="462" y="214" width="16" height="28" rx="7" fill="#422006" stroke="#FACC15" stroke-width="2"/>
    <text x="470" y="255" text-anchor="middle" fill="#FEF08A" font-size="8.5" font-weight="700">SHARE</text>

    <!-- OPTIONS Button (Right of Touchpad) -->
    <rect x="682" y="214" width="16" height="28" rx="7" fill="#1E293B" stroke="#64748B" stroke-width="2"/>
    <text x="690" y="255" text-anchor="middle" fill="#94A3B8" font-size="8.5" font-weight="700">OPT</text>

    <!-- PS / Team Badge -->
    <circle cx="580" cy="304" r="15" fill="#0F172A" stroke="#38BDF8" stroke-width="2"/>
    <text x="580" y="308" text-anchor="middle" fill="#38BDF8" font-size="8.5" font-weight="700">32722</text>

    <!-- D-Pad (Top-Left) -->
    <g transform="translate(395, 268)">
      <path d="M -13 -37 L 13 -37 L 13 -13 L 37 -13 L 37 13 L 13 13 L 13 37 L -13 37 L -13 13 L -37 13 L -37 -13 L -13 -13 Z"
            fill="#0F172A" stroke="#64748B" stroke-width="2"/>
      <polygon points="0,-30 -6,-21 6,-21" fill="#94A3B8"/>
      <polygon points="0,30 -6,21 6,21" fill="#94A3B8"/>
      <polygon points="-30,0 -21,-6 -21,6" fill="#94A3B8"/>
      <polygon points="30,0 21,-6 21,6" fill="#94A3B8"/>
    </g>

    <!-- PlayStation Face Buttons (Top-Right) -->
    <!-- Triangle (Top - Green) -->
    <circle cx="765" cy="228" r="18" fill="#052E16" stroke="#34D399" stroke-width="2.5"/>
    <polygon points="765,218 775,235 755,235" fill="none" stroke="#6EE7B7" stroke-width="2.4" stroke-linejoin="round"/>

    <!-- Square (Left - Pink) -->
    <circle cx="725" cy="268" r="18" fill="#500724" stroke="#F472B6" stroke-width="2.5"/>
    <rect x="717" y="260" width="16" height="16" rx="2" fill="none" stroke="#FBCFE8" stroke-width="2.4"/>

    <!-- Circle (Right - Red) -->
    <circle cx="805" cy="268" r="18" fill="#450A0A" stroke="#F87171" stroke-width="2.5"/>
    <circle cx="805" cy="268" r="8.5" fill="none" stroke="#FECACA" stroke-width="2.4"/>

    <!-- Cross (Bottom - Blue) -->
    <circle cx="765" cy="308" r="18" fill="#082F49" stroke="#60A5FA" stroke-width="2.5"/>
    <line x1="758" y1="301" x2="772" y2="315" stroke="#BFDBFE" stroke-width="2.6" stroke-linecap="round"/>
    <line x1="772" y1="301" x2="758" y2="315" stroke="#BFDBFE" stroke-width="2.6" stroke-linecap="round"/>

    <!-- Symmetrical Thumbsticks (Bottom-Center) -->
    <!-- Left Stick (L3) -->
    <circle cx="484" cy="358" r="38" fill="#0F172A" stroke="#475569" stroke-width="2"/>
    <circle cx="484" cy="358" r="25" fill="url(#stickGrad)" stroke="#A78BFA" stroke-width="2.5"/>
    <text x="484" y="362" text-anchor="middle" fill="#EDE9FE" font-size="11" font-weight="700">L3</text>

    <!-- Right Stick (R3) -->
    <circle cx="676" cy="358" r="38" fill="#0F172A" stroke="#475569" stroke-width="2"/>
    <circle cx="676" cy="358" r="25" fill="url(#stickGrad)" stroke="#A78BFA" stroke-width="2.5"/>
    <text x="676" y="362" text-anchor="middle" fill="#EDE9FE" font-size="11" font-weight="700">R3</text>
  </g>

  <!-- ==================== CALLOUTS ==================== -->
{callouts}

  <!-- ==================== FOOTER / LEGEND ==================== -->
  <rect x="20" y="524" width="1120" height="116" rx="10" fill="#1E293B" stroke="#334155" stroke-width="1.5"/>
  <text x="38" y="548" fill="#F8FAFC" font-size="13.5" font-weight="700">Safety &amp; Editing Guide</text>

  <circle cx="46" cy="572" r="6" fill="#EF4444"/>
  <text x="60" y="576" fill="#E2E8F0" font-size="12"><tspan font-weight="700" fill="#FCA5A5">Global E-Stop:</tspan> Squeeze L2 + R2 (&gt;85% for 150ms) on gamepad1 or gamepad2. Hold SHARE (0.5s) to release.</text>

  <circle cx="46" cy="596" r="6" fill="#FB923C"/>
  <text x="60" y="600" fill="#E2E8F0" font-size="12"><tspan font-weight="700" fill="#FDBA74">Auto E-Stops:</tspan> FlyWheel locks on stall/encoder fault/overcurrent (press ✕ to clear). Intake locks on failed jam recovery/fault (press □ to clear).</text>

  <circle cx="46" cy="620" r="6" fill="#38BDF8"/>
  <text x="60" y="624" fill="#E2E8F0" font-size="12"><tspan font-weight="700" fill="#7DD3FC">Easy Editing:</tspan> Edit labels in doc/controller-map.json and run "python doc/generate_controller.py" to update both controller.svg and CONTROLLER.md.</text>
</svg>
"""


def build_markdown(config: dict) -> str:
    b = config["buttons"]

    def row(key: str) -> str:
        item = b[key]
        action = item["tableAction"].replace("|", "&#124;")
        notes = item["tableNotes"].replace("|", "&#124;")
        return f"| **{item['psName']}** | `{item['ftcCode']}` | {action} | {notes} |"

    rows = "\n".join(
        row(k)
        for k in [
            "L2",
            "R2",
            "L1",
            "R1",
            "SHARE",
            "OPTIONS",
            "LEFT_STICK",
            "RIGHT_STICK",
            "DPAD",
            "TRIANGLE",
            "CIRCLE",
            "CROSS",
            "SQUARE",
        ]
    )

    return f"""# Controller Map — {config['robotName']}

![PlayStation Controller Layout](doc/media/controller.svg)

Controls implemented in [`DriverControlled.java`](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/teleop/DriverControlled.java) and [`RobotMonitor.java`](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/debug/RobotMonitor.java).

> **How to edit this diagram & table easily:**
> 1. Edit any button's title, subtitle, category color, or notes in [`doc/controller-map.json`](doc/controller-map.json).
> 2. Run `python doc/generate_controller.py` to regenerate both [`doc/media/controller.svg`](doc/media/controller.svg) and this `CONTROLLER.md`.

---

## PlayStation Button Reference

| PlayStation Control | FTC SDK Field | Action | Notes |
| --- | --- | --- | --- |
{rows}

---

## Emergency Stop & Auto E-Stop Details

- **Global E-Stop (`L2 + R2`)**: Pull both triggers past `85%` for `150 ms` on `gamepad1` or `gamepad2` (active in **all** OpModes). Cuts power to DriveTrain, FlyWheel, and Intake and latches until cleared.
- **Clear E-Stops (`SHARE`)**: Hold `SHARE` (`gamepad.back`) for `500 ms` on either gamepad to release global and subsystem E-Stop latches.
- **FlyWheel Auto E-Stop**: Locks FlyWheel on `STALLED`, `ENCODER_FAULT`, or `OVERCURRENT`. Press **`✕` Cross** once to clear the latch safely (stays off), then press **`✕`** again to restart.
- **Intake Auto E-Stop**: Locks Intake if jam recovery (`REVERSE` → `SUPER`) fails, or on `ENCODER_FAULT` / `OVERCURRENT`. Press **`□` Square** once to clear the latch safely (stays off), then press **`□`** again to re-enable.
- **DriveTrain Auto E-Stop**: Locks DriveTrain on sustained `OVERCURRENT` (`AUTO_ESTOP_DRIVETRAIN_ON_OVERCURRENT = true` in [`RobotConstants.java`](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/config/RobotConstants.java)). Hold **`SHARE`** for `0.5 s` to clear.

---

## Drive Mode Switch (`MECANUM_ODO_INSTALLED`)

Configured in [`RobotConstants.java`](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/config/RobotConstants.java):
- `MECANUM_ODO_INSTALLED = true`: Left Stick = Forward/Backward (`Y`) + Strafe (`X`), Right Stick = Turn (`X`), **`○` Circle** = Reset Odometry Pose `(0, 0, 0°)`.
- `MECANUM_ODO_INSTALLED = false`: Legacy Tank Drive (Left Stick `Y` = Left motors, Right Stick `Y` = Right motors).
"""


def main() -> None:
    config = json.loads(CONFIG_PATH.read_text(encoding="utf-8"))
    SVG_PATH.parent.mkdir(parents=True, exist_ok=True)
    SVG_PATH.write_text(build_svg(config), encoding="utf-8")
    MD_PATH.write_text(build_markdown(config), encoding="utf-8")
    print(f"Generated {SVG_PATH} and {MD_PATH}")


if __name__ == "__main__":
    main()
