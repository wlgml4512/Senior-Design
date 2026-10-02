Created branches for tasks
- object detection
- devleoping app
- voice recognition

## Running Fusion

Fusion is intended to combine three inputs into one safe motion decision:
- computer vision provides target angle and distance so the robot can follow a person/marker
- ultrasonic sensing overrides that motion when the robot is about to hit something
- voice commands gate or modify motion with commands such as `follow`, `stop`, `resume`, `slow down`, and `speed up`

The software path for that integration is in place, but hardware wiring and sensor availability should still be validated before arming the drivetrain.

From the project root:

```bash
cd ECE49022-1
```

You can see the available fusion options with:

```bash
python3 fusion/main.py --help
```

Run fusion with both subsystems enabled and automatically start the computer vision process:

```bash
python3 fusion/main.py --cv on --ultrasonic on --start-cv
```

Run a bounded dry-run test on the robot without arming the motors:

```bash
python3 fusion/main.py --cv on --ultrasonic on --voice on --max-steps 50 --trace-file fusion_trace.jsonl
```

Arm the drivetrain only after the dry run looks correct:

```bash
python3 fusion/main.py --cv on --ultrasonic on --voice on --arm-motors
```

Run fusion with only computer vision enabled:

```bash
python3 fusion/main.py --cv on --ultrasonic off --start-cv
```

If you already started the computer vision system in another terminal, use:

```bash
python3 computer-vision/main.py
python3 fusion/main.py --cv on --ultrasonic off
```

Run fusion with only the ultrasonic subsystem enabled:

```bash
python3 fusion/main.py --cv off --ultrasonic on
```

Run fusion with the voice-command bridge enabled:

```bash
python3 fusion/main.py --cv on --ultrasonic on --voice on
```

If you want fusion to also start the voice controller process, use:

```bash
python3 fusion/main.py --cv on --ultrasonic on --voice on --start-voice
```

Run fusion in dry-run mode with both subsystems disabled:

```bash
python3 fusion/main.py --cv off --ultrasonic off
```

To inject a recognized voice command into the fusion voice bridge from another terminal:

```bash
python3 fusion/voice_command_cli.py "follow"
python3 fusion/voice_command_cli.py "stop"
python3 fusion/voice_command_cli.py "resume"
python3 fusion/voice_command_cli.py "slow down"
python3 fusion/voice_command_cli.py "speed up"
```

While fusion is running, it prints:

- the enabled subsystem configuration at startup
- live `cv` data being read by fusion
- live `ultrasonic` data being read by fusion
- recognized `voice_commands` and the live `voice_state`
- a full per-cycle snapshot if `--trace-file` is provided
- the `motion` command computed from those inputs
- a formatted motor command line showing `Linear`, `Angular`, and `Stop`

To stop the fusion run:

- Press `Ctrl+C` in the terminal where `python3 fusion/main.py ...` is running.
- Fusion now sends one final stop command with `Linear: 0.00 | Angular: 0.00 | Stop: True` before exiting.
- If you used `--start-cv`, the computer vision child process is also terminated automatically when fusion exits.
- If you started `python3 computer-vision/main.py` in a separate terminal, stop that one with `Ctrl+C` in its terminal.

Notes:

- If only one ultrasonic sensor exists, fusion uses the single `get_distance()` reading as the front obstacle distance.
- If left and right ultrasonic functions exist, fusion uses both and computes the front distance from them.
- The current fusion interface supports either configuration and degrades cleanly to `None` distances if the embedded ultrasonic module cannot be imported or read.
- For `--start-cv` to work, the computer vision dependencies and camera hardware must be available on the machine running the code.
- `--voice on` enables fusion's voice-command bridge and reads command files, but it does not automatically launch the standalone voice subsystem.
- For `--start-voice` to work, the dependencies used by `voice-recognition/voice_controller.py` must be available on the machine running the code, including the local `keys.py` credentials file expected by the voice stack.
- External voice subsystems can also publish JSON or plain-text commands into `voice-recognition/commands.jsonl` and `voice-recognition/embedded-commands.jsonl`, which fusion tails continuously.
- Fusion now handles all supported voice actions: `follow`, `reduce_speed`, `increase_speed`, `emergency_stop`, and `resume`.
- When voice control is enabled, fusion waits for a `follow` command before sending CV/ultrasonic-driven motion.
- `follow` enables CV target following, `speed up` / `slow down` scale the resulting forward motor command, and `stop` / `resume` now latch through fusion's motor arbitration before reaching the embedded motor backend.
- Fusion now uses an explicit motion priority order: `voice emergency stop` > `voice follow gate` > `ultrasonic obstacle override` > `default CV tracking behavior` > `voice speed adjustments`.
- Computer vision remains the guidance source once following is enabled, and ultrasonic data can still override motion for obstacle avoidance unless a voice emergency stop is active.
- Motor output is now disarmed by default. Use `--arm-motors` only when you are ready for the robot to move.
- `--max-steps` and `--trace-file` make it easier to run short, repeatable on-robot tests and review exactly what fusion decided on each loop.
- Fusion now converts planner output into differential left/right motor commands for the two drive motors, where each side motor powers that side's three wheels.
- Positive angular commands turn the robot left by slowing or reversing the left motor and speeding up the right motor; negative angular commands do the opposite.
- The embedded motor backend now falls back to the default wiring declared in `embedded/motors_encoders.py`: left PWM `13`, left DIR `24`, right PWM `12`, right DIR `26`.
- Encoder telemetry also falls back to the default wiring in `embedded/motors_encoders.py`: left encoder `16/7`, right encoder `21/20`.
- You can still override any of those defaults with `LEFT_MOTOR_PWM_PIN`, `LEFT_MOTOR_DIR_PIN`, `RIGHT_MOTOR_PWM_PIN`, `RIGHT_MOTOR_DIR_PIN`, `LEFT_ENCODER_A_PIN`, `LEFT_ENCODER_B_PIN`, `RIGHT_ENCODER_A_PIN`, and `RIGHT_ENCODER_B_PIN`.
- If one motor is mounted with the opposite polarity, set `LEFT_MOTOR_INVERT_DIRECTION=1` or `RIGHT_MOTOR_INVERT_DIRECTION=1`.

## Fusion Subsystem Communication Breakdown

The fusion subsystem is organized as a set of interface modules plus a central control loop. Each interface is responsible for converting subsystem-specific data into a normalized format, so the planner and motion arbiter can operate on one consistent internal representation each cycle.

### 1. Control Loop Orchestration

The main loop in `fusion/control_loop.py` is the communication hub for the subsystem. At each control-cycle iteration, it performs the following sequence:

1. Read the latest computer vision state through `interfaces/cv_interface.py`.
2. Read the latest ultrasonic measurements through `interfaces/ultrasonic_interface.py`.
3. Read any new voice events through `interfaces/voice_interface.py`.
4. Update the `VoiceCommandController` state based on those voice events.
5. Fuse the vision and obstacle data through `fusion.py`.
6. Compute a candidate motion command through `planner.py`.
7. Apply subsystem priority rules through `motion_priority.py`.
8. Convert the final motion command into left/right motor output through `interfaces/motor_interface.py`.

This means all communication between subsystems is funneled through the fusion control loop rather than allowing each subsystem to command the motors independently.

### 2. Computer Vision to Fusion

The computer vision subsystem communicates with fusion through the file `computer-vision/log.txt`. The CV interface tails this log and parses each line into structured tracking data. Fusion recognizes three main CV states:

- `tracking`: the target is visible and the system has a usable `target_id`, `angle`, and `distance`
- `waiting`: the tracker has temporarily lost fresh geometry, but fusion may continue using the last known target state
- `lost`: the target is no longer considered available

Fusion also marks CV data as `stale` if no fresh log updates are received within the timeout window. The CV interface keeps both the source log timestamp and a fusion-side `received_at` timestamp so traces can distinguish when the sample was produced versus when fusion consumed it.

### 3. Ultrasonic to Fusion

The ultrasonic subsystem communicates through the Python module `embedded.ultrasonic_sensor`, which is imported by `interfaces/ultrasonic_interface.py`. The interface attempts to read:

- `sensor_left_distance()`
- `sensor_right_distance()`
- or a shared `get_distance(trig, echo)` helper together with left/right pin constants
- or, if only one sensor path exists, `get_distance()`

The interface converts readings from centimeters to meters and produces a normalized obstacle dictionary with:

- `left_distance`
- `right_distance`
- `front_distance`

If separate left and right distances exist, fusion treats the minimum of those values as the effective front obstacle distance. The interface also records a measurement timestamp and a `received_at` timestamp for tracing.

### 4. Voice Control to Fusion

The voice subsystem communicates with fusion through two JSONL bridges:

- `voice-recognition/commands.jsonl`
- `voice-recognition/embedded-commands.jsonl`

The voice interface supports two communication paths:

- in-process queue events, for direct function-based injection
- file-based events, for external voice processes or scripts

Each voice entry is normalized into a common event structure with fields such as `timestamp`, `text`, `command`, `confidence`, `source`, and `command_id`. Fusion currently accepts the voice actions:

- `follow`
- `reduce_speed`
- `increase_speed`
- `emergency_stop`
- `resume`

The `VoiceCommandController` stores the current voice-governed state, including whether follow mode is active, whether an emergency stop is latched, and what speed multiplier should be applied. This allows voice control to influence navigation without bypassing the fusion safety logic.

### 5. Fusion Layer Internal Communication

The file `fusion/fusion.py` combines the normalized CV and ultrasonic inputs into two internal data structures:

- `target_state`
- `obstacle_state`

`target_state` contains the navigation-relevant tracking information such as target angle, target distance, target ID, whether the target is live, and whether the system is operating from a held last-known target. `obstacle_state` contains the obstacle distances used by the planner.

This layer is also responsible for short-term continuity when vision data briefly drops out. If CV enters `waiting`, `lost`, or `stale`, fusion can temporarily preserve the previous target geometry for a bounded timeout instead of immediately clearing the target. This prevents abrupt behavior when the camera has a short interruption.

### 6. Planner to Motion Arbiter Communication

The planner in `fusion/planner.py` receives `target_state` and `obstacle_state` and returns a motion command with:

- `linear`
- `angular`
- `stop`
- `reason`

This is the stage where fusion decides the robot's intended motion before it ever thinks about PWM or motor direction pins.

The planner combines three ideas:

- steer toward the target
- move forward only when the target is farther than the desired follow distance
- combine target steering with an obstacle-based steering bias, and fully override with a stop-and-turn behavior if something is directly in front

#### Step 6A. Side obstacle avoidance

The planner first turns left and right ultrasonic readings into an avoidance steering term.

- `side_clearance = 0.30`
- `avoidance_gain = 1.2`

For each side:

- if the obstacle is farther than `0.30 m`, that side contributes `0.0`
- if the obstacle is closer than `0.30 m`, the contribution grows linearly as it gets closer

The formulas are:

```text
left_avoidance  = 0.0                               if left_distance  is None or left_distance  >= 0.30
                  (0.30 - left_distance) / 0.30    otherwise

right_avoidance = 0.0                               if right_distance is None or right_distance >= 0.30
                  (0.30 - right_distance) / 0.30    otherwise

avoidance_angular = 1.2 * (right_avoidance - left_avoidance)
```

Interpretation:

- if the left side is tighter than the right side, `left_avoidance` is larger, so `avoidance_angular` becomes negative and pushes the robot right
- if the right side is tighter, `avoidance_angular` becomes positive and pushes the robot left
- if both sides are equally clear, the avoidance term is `0.0`

#### Step 6B. Front obstacle override

Before tracking the target, the planner checks the front obstacle distance:

- `stop_distance = 0.35`

If `front_distance < 0.35 m`, normal target following is skipped and the planner immediately returns:

- `linear = 0.0`
- `angular = +1.0` rad/s if the left side is more open
- `angular = -1.0` rad/s if the right side is more open
- `stop = True`
- `reason = "front_obstacle"`

So a close obstacle in front does not just slow the robot down. It completely takes control of the motion decision for that cycle.

#### Step 6C. Target steering from computer vision

If there is no front-obstacle override and a valid target exists, the planner uses the target angle to decide how much to turn.

Fusion uses the convention:

- positive target angle means the target is to the robot's left
- negative target angle means the target is to the robot's right
- positive angular motion means turn left
- negative angular motion means turn right

- `angle_deadband_deg = 4.0`
- `angle_gain = 0.04`
- `max_angular_speed = 1.0`

Formula:

```text
tracking_angular = 0.0                    if abs(target_angle) <= 4.0
                   0.04 * target_angle    otherwise

angular_cmd = clamp(tracking_angular + avoidance_angular, -1.0, 1.0)
```

Interpretation:

- if the target is almost centered, within `+/-4 degrees`, the planner does not steer at all
- if the target is off-center, steering grows proportionally with the angle error
- obstacle avoidance is added on top of target tracking
- the final angular command is limited to `+/-1.0 rad/s`

#### Step 6D. Forward speed from target distance

The planner separately decides how fast to move forward based on how far the robot is from the target.

- `follow_distance = 1.0`
- `follow_distance_tolerance = 0.10`
- `target_min_distance = 0.35`
- `max_linear_speed = 0.6`

Formula:

```text
distance_error = target_distance - 1.0

linear_cmd = 0.0                                      if abs(distance_error) <= 0.10
             clamp(0.8 * distance_error, 0.0, 0.6)    otherwise
```

Important behavior:

- if the target is between `0.90 m` and `1.10 m`, the robot does not drive forward
- if the target is farther than that band, the robot moves forward faster as the distance error grows
- the forward command is capped at `0.6 m/s`
- the clamp floor is `0.0`, so the planner does not command reverse motion to back away from a far/close target in normal tracking
- if `target_distance <= 0.35 m`, the planner forces `linear = 0.0`, keeps any needed steering, and returns `reason = "target_too_close"`

#### Step 6E. Special tracking states

The planner handles CV states differently so motion stays stable during brief tracking interruptions:

- `tracking`: use the full `linear_cmd` and `angular_cmd`
- `waiting`: hold position with `linear = 0.0` but keep steering with `angular_cmd`
- `holding`: use a reduced forward speed based on the last known target
- `lost`: stop completely

For the `holding` state:

- `hold_linear_scale = 0.35`
- `hold_max_linear_speed = 0.20`

Formula:

```text
held_linear_cmd = min(linear_cmd * 0.35, 0.20)
```

This lets the robot keep creeping carefully toward the last known target instead of charging forward on stale vision data.

The motion arbiter in `fusion/motion_priority.py` then decides which subsystem has authority over the final command. The current priority order is:

1. Voice emergency stop
2. Voice follow gate
3. Ultrasonic front-obstacle override
4. Default CV-driven motion
5. Voice speed adjustment

This design keeps command ownership explicit and prevents conflicting subsystems from issuing independent motor requests.

### 7. Motor Control Calculation and Actuation

Motor control is performed in `fusion/interfaces/motor_interface.py`. This is the part that converts the final fused motion intent into the actual left and right motor commands.

The easiest way to read it is as a pipeline:

1. Inputs produce a planner command: `linear`, `angular`, `stop`, and `reason`
2. Voice arbitration may block motion or scale only the forward speed
3. The motor interface normalizes those values
4. The motor interface mixes them into left and right side commands
5. Each side command becomes a motor direction bit plus a PWM duty cycle

#### Step 7A. Voice can gate or scale the motion before motor math

The motion arbiter can change the planner output before it reaches the motors:

- `emergency_stop` forces `linear = 0.0`, `angular = 0.0`, `stop = True`
- if voice follow mode has not been enabled yet, fusion also forces a full stop
- `increase_speed` and `reduce_speed` only scale `linear`
- `angular` is intentionally left alone so voice does not distort steering or safety turns

The voice speed multiplier starts at `1.0`, changes in steps of `0.2`, and is limited to:

- minimum `0.4`
- maximum `1.6`

So if the planner asks for `linear = 0.50` and the voice multiplier is `0.8`, the motor interface receives `linear = 0.40`.

#### Step 7B. Normalize motion into a motor-friendly range

The planner outputs physical-style motion terms:

- `linear` in meters per second
- `angular` in radians per second

The motor interface first normalizes these values using the configured maximum motion scales:

- `MAX_LINEAR_SPEED_MPS = 0.6`
- `MAX_ANGULAR_SPEED_RAD_S = 1.0`

This produces:

- `linear_normalized = linear / MAX_LINEAR_SPEED_MPS`
- `angular_normalized = angular / MAX_ANGULAR_SPEED_RAD_S`

Both values are clamped into the range `[-1.0, 1.0]`.

So after this step:

- `-1.0` means full reverse or full clockwise turn
- `0.0` means no command
- `+1.0` means full forward or full counterclockwise turn

#### Step 7C. Mix forward motion and turning into left/right motor commands

The robot uses differential drive, so turning is created by giving the left and right sides different commands.

The interface applies:

- `left = linear_normalized - TURN_MIX_GAIN * angular_normalized`
- `right = linear_normalized + TURN_MIX_GAIN * angular_normalized`

where `TURN_MIX_GAIN = 0.75`.

This means:

- positive linear values drive both sides forward
- negative linear values drive both sides backward
- positive angular values reduce the left side command and increase the right side command, causing a left turn
- negative angular values do the opposite, causing a right turn

The `TURN_MIX_GAIN` matters because it controls how strongly turning changes the motor split. A value of `0.75` means a full-scale turn command contributes `0.75` to one side and `-0.75` to the other before clamping.

After mixing, both side commands are clamped to `[-1.0, 1.0]`.

#### Step 7D. Deadband removes tiny commands

A small deadband is then applied using `MIN_COMMAND_DEADBAND = 0.05`.

That means:

- if `abs(left) < 0.05`, left becomes `0.0`
- if `abs(right) < 0.05`, right becomes `0.0`

This keeps the robot from twitching because of tiny steering corrections or floating-point noise.

#### Step 7E. Stop handling

If the final motion command indicates `stop=True` and the angular command is also effectively zero, the motor interface forces both sides to `0.0`.

This is an important detail:

- `stop=True` with `angular = 0.0` means a true stop
- `stop=True` with nonzero `angular` is allowed to become a turn-in-place command

That is why front-obstacle avoidance can still rotate the robot even though the planner marks the motion as a stop condition.

#### Step 7F. Final conversion into hardware actions

Each side command is then converted into low-level motor control:

- if the command is inside the deadband, that motor is stopped
- otherwise the sign chooses direction
- the magnitude chooses PWM duty cycle

Formula for each side:

```text
direction = 1 if command < 0 else 0
duty_cycle = abs(command)
```

If `LEFT_MOTOR_INVERT_DIRECTION=1` or `RIGHT_MOTOR_INVERT_DIRECTION=1` is set, that direction bit is flipped for the corresponding side.

So a final side command of:

- `+0.80` means forward direction with `80%` duty cycle
- `-0.35` means reverse direction with `35%` duty cycle
- `0.00` means stop

#### Step 7G. Worked examples

Example 1: Target straight ahead and far away

- target angle = `0 deg`
- target distance = `1.50 m`
- no close obstacles
- voice multiplier = `1.0`

Planner:

```text
distance_error = 1.50 - 1.00 = 0.50
linear = clamp(0.8 * 0.50, 0.0, 0.6) = 0.40
angular = 0.0
```

Motor normalization and mixing:

```text
linear_normalized  = 0.40 / 0.60 = 0.67
angular_normalized = 0.00 / 1.00 = 0.00

left  = 0.67 - 0.75 * 0.00 = 0.67
right = 0.67 + 0.75 * 0.00 = 0.67
```

Result:

- both motors run forward at about `67%`
- the robot drives straight forward

Example 2: Target is to the left and still far away

- target angle = `15 deg`
- target distance = `1.80 m`
- no close obstacles
- voice multiplier = `1.0`

Planner:

```text
tracking_angular = 0.04 * 15 = 0.60
distance_error = 1.80 - 1.00 = 0.80
linear = clamp(0.8 * 0.80, 0.0, 0.6) = 0.60
angular = clamp(0.60, -1.0, 1.0) = 0.60
```

Motor normalization and mixing:

```text
linear_normalized  = 0.60 / 0.60 = 1.00
angular_normalized = 0.60 / 1.00 = 0.60

left  = 1.00 - 0.75 * 0.60 = 0.55
right = 1.00 + 0.75 * 0.60 = 1.45 -> clamp to 1.00
```

Result:

- left motor runs at `55%`
- right motor runs at `100%`
- the robot moves forward while turning left toward the target

Example 3: Obstacle directly ahead, more space on the left

- front distance = `0.20 m`
- left distance = `0.60 m`
- right distance = `0.25 m`

Planner immediately overrides tracking:

```text
linear = 0.0
angular = +1.0
stop = True
reason = "front_obstacle"
```

Motor normalization and mixing:

```text
linear_normalized  = 0.0
angular_normalized = 1.0

left  = 0.0 - 0.75 * 1.0 = -0.75
right = 0.0 + 0.75 * 1.0 = 0.75
```

Result:

- left side reverses at `75%`
- right side drives forward at `75%`
- the robot turns in place to avoid the obstacle

Example 4: Voice slows the robot down without changing steering

- planner output before voice: `linear = 0.40`, `angular = 0.30`
- voice multiplier = `0.8`

After voice arbitration:

```text
linear = 0.40 * 0.8 = 0.32
angular = 0.30
```

So voice changes how fast the robot advances, but it does not weaken the steering correction.

### 8. Embedded Motor Backend Communication

Once the left and right side commands are computed, the motor interface sends them to the embedded drivetrain backend if and only if:

- the embedded motor module is available
- the runtime has explicitly armed motor output through `--arm-motors`

The embedded backend is constructed from `embedded.motors_encoders`. For each side of the drivetrain, fusion creates a `Motor` object and, when available, an `Encoder` object. Each `_MotorSide` performs the final low-level actuation:

- chooses a direction bit based on the sign of the command
- applies optional direction inversion using environment variables
- sets the PWM duty cycle based on the magnitude of the command
- stops the motor entirely when the command is inside the deadband

Because each side of the drivetrain corresponds to one side motor assembly, the final left and right commands represent the net drive command for that side’s wheel set.

If hardware is unavailable or motors are intentionally disarmed, fusion still computes the exact same left/right commands but reports them in `dry_run` or `disarmed` mode instead of touching hardware. This makes it possible to validate the full communication and decision pipeline without moving the robot.

### 9. Trace and Debug Communication

Fusion also communicates its internal state outward through terminal logs and optional JSONL trace snapshots. When `--trace-file` is provided, each control-cycle snapshot contains:

- raw CV, ultrasonic, and voice inputs as seen by fusion
- the fused target and obstacle state
- the planner output
- the motion-arbitrated output
- the final motor command
- timing information for input-sample-to-fusion and input-sample-to-motor latency

This trace path is important because it documents not only what decision fusion made, but also which interface or subsystem caused that decision.
