robomas_plugins
====
## Description
It is for my Debug_CAN board.
It works on ros2 Lyrical Luth. (Also tested on Humble Hawksbill and Foxy Fitzroy, though Foxy is EOL and not recommended.)

This supports only environment as 1 byte of 8 bits. (uint8_t == unsigned char)

## Usage

TODO: add example.

## Install

copy udev rule
```
sudo cp ~/ros2_ws/src/robomas_plugins/udev/60-robomas.rules /etc/udev/rules.d/60-robomas.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

## Mode 4: stablepos (velocity-limited position control)

The STM32 firmware `robomas_center` implements `Mode::stablepos` as follows:

```text
target position [deg] -- position P (poskp) -- clamp (+/- stable_pos_limit_vel [rad/s])
                                  -- velocity PI (velkp, velki) --> motor current
```

Set `robomas_plugins/msg/RobomasFrame` on `/robomas_frame` with
`mode: 4` (`MODE_STABLEPOS`). The firmware resets that motor's position
origin when this configuration frame is received. Then send the target
angle in degrees using `robomas_plugins/msg/RobomasTarget` on
`/robomas_target1` through `/robomas_target8`. Target messages and mode
configuration use independent COBS-encoded serial frames.

Example for motor 1 (CAN ID `0x201`, `motor: 0`):

```bash
ros2 topic pub --once /robomas_frame robomas_plugins/msg/RobomasFrame \
  "{motor: 0, c620: true, mode: 4, temp: 75, velkp: 0.15, velki: 13.0, poskp: 0.5, stable_pos_limit_vel: 1.0}"
ros2 topic pub --once /robomas_target1 robomas_plugins/msg/RobomasTarget \
  "{target: 90.0}"
```

- `stable_pos_limit_vel`: max speed in **rad/s**, not rpm.
- `target`: relative position in **degrees**, with origin set when mode 4 starts.
- As with mode 2, enabling mode 4 also enables CAN feedback forwarding
  using the firmware's diagnostic command (increases USB traffic).
- If you set `RobomasFrame.header.stamp` when entering mode 4, later
  `RobomasTarget.header.stamp` must be at least as recent. Targets with
  older/empty stamps are discarded; leave both at zero for legacy clients.
- Controller gains and thermal limits are examples only; validate them safely
  before operating a motor under load.
