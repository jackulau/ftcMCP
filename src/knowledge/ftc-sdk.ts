export const FTC_SDK_KNOWLEDGE = {
  overview: `
## FTC (FIRST Tech Challenge) Overview

FIRST Tech Challenge (FTC) is a robotics competition where teams design, build, program, and operate
robots built with Android-based control systems. Robots are programmed using Java (primarily) or
Kotlin via Android Studio.

### SDK Details
- **Current SDK Version**: 11.1.0
- **Platform**: Android-based (REV Control Hub / REV Expansion Hub)
- **IDE**: Android Studio (recommended), OnBot Java, or Blocks
- **Language**: Java (primary), Kotlin (supported)

### Project Structure
Team-written code lives in:
\`\`\`
TeamCode/src/main/java/org/firstinspires/ftc/teamcode/
\`\`\`

Package declaration for all team files:
\`\`\`java
package org.firstinspires.ftc.teamcode;
\`\`\`

Or with sub-packages:
\`\`\`java
package org.firstinspires.ftc.teamcode.opmodes;
package org.firstinspires.ftc.teamcode.subsystems;
package org.firstinspires.ftc.teamcode.util;
\`\`\`

### Key SDK Modules
- **FtcRobotController**: The main Android app module (DO NOT modify unless advanced)
- **TeamCode**: Where ALL team code goes — OpModes, subsystems, utilities
- **RobotCore**: Low-level hardware abstraction (provided by SDK)
- **Hardware**: Hardware device interfaces (provided by SDK)

### Control System Hardware
- **REV Control Hub**: Main robot controller (has built-in Android computer)
- **REV Expansion Hub**: Additional I/O (connected via RS-485 to Control Hub)
- **Driver Station**: Android phone or REV Driver Hub running the DS app
- **Gamepads**: Logitech F310 or Xbox-compatible (plugged into Driver Station)
`,

  opmodePatterns: `
## OpMode Patterns and Lifecycle

There are two base classes for writing OpModes in FTC:

### 1. Iterative OpMode (extends OpMode)

The iterative OpMode uses a state-machine-friendly callback lifecycle:

\`\`\`
init() → init_loop() → start() → loop() → stop()
\`\`\`

**Lifecycle Methods:**
- \`init()\` — Called ONCE when INIT is pressed. Initialize hardware, set starting positions.
- \`init_loop()\` — Called REPEATEDLY after init() until START is pressed. Use for sensor calibration, waiting feedback.
- \`start()\` — Called ONCE when START is pressed (or auto-start timer fires). Reset timers, start motion.
- \`loop()\` — Called REPEATEDLY after start(). Main robot logic goes here. Runs until STOP.
- \`stop()\` — Called ONCE when STOP is pressed. Clean up resources, stop motors.

**When to use:** Best for finite-state-machine (FSM) based designs, TeleOp with subsystem states,
and when you need non-blocking architecture. Ideal for complex autonomous routines with many parallel actions.

**Complete Example:**
\`\`\`java
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Iterative TeleOp Example", group = "Examples")
public class IterativeTeleOpExample extends OpMode {

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private Servo claw;

    private boolean clawOpen = false;
    private boolean previousA = false;

    @Override
    public void init() {
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");
        claw       = hardwareMap.get(Servo.class, "claw");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        claw.setPosition(0.0); // closed

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        // Could do sensor calibration here
        telemetry.addData("Status", "Waiting for start...");
        telemetry.update();
    }

    @Override
    public void start() {
        resetRuntime();
        telemetry.addData("Status", "Running");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Mecanum drive
        double drive  = -gamepad1.left_stick_y;  // Negate Y: pushing stick forward = negative value
        double strafe =  gamepad1.left_stick_x;
        double rotate =  gamepad1.right_stick_x;

        double frontLeftPower  = drive + strafe + rotate;
        double frontRightPower = drive - strafe - rotate;
        double backLeftPower   = drive - strafe + rotate;
        double backRightPower  = drive + strafe - rotate;

        // Normalize
        double maxPower = Math.max(1.0, Math.max(
            Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
            Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))
        ));
        frontLeft.setPower(frontLeftPower / maxPower);
        frontRight.setPower(frontRightPower / maxPower);
        backLeft.setPower(backLeftPower / maxPower);
        backRight.setPower(backRightPower / maxPower);

        // Toggle claw on A button (rising edge detection)
        if (gamepad1.a && !previousA) {
            clawOpen = !clawOpen;
            claw.setPosition(clawOpen ? 0.5 : 0.0);
        }
        previousA = gamepad1.a;

        telemetry.addData("Drive", "FL=%.2f FR=%.2f BL=%.2f BR=%.2f",
            frontLeftPower, frontRightPower, backLeftPower, backRightPower);
        telemetry.addData("Claw", clawOpen ? "Open" : "Closed");
        telemetry.addData("Runtime", "%.1f seconds", getRuntime());
        telemetry.update();
    }

    @Override
    public void stop() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
}
\`\`\`

### 2. LinearOpMode (extends LinearOpMode)

The linear OpMode runs as a single sequential method:

\`\`\`
runOpMode() {
    // initialize
    waitForStart();
    // sequential logic
}
\`\`\`

**Key Methods:**
- \`runOpMode()\` — The single entry point. ALL code goes here.
- \`waitForStart()\` — Blocks until the START button is pressed.
- \`opModeIsActive()\` — Returns true while OpMode is running (not stopped). Use in while loops.
- \`opModeInInit()\` — Returns true while in INIT phase (before START). Use for init loops.
- \`idle()\` — Yields thread to allow other operations to run. Use in tight loops.
- \`sleep(long milliseconds)\` — Pauses execution for the specified time (only safe in LinearOpMode).

**When to use:** Best for sequential autonomous routines, simple TeleOp, beginners, and when
step-by-step execution flow is clearer. Ideal for linear autonomous paths.

**Complete Example:**
\`\`\`java
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Linear Auto Example", group = "Examples")
public class LinearAutoExample extends LinearOpMode {

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private Servo claw;

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        // INIT PHASE
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");
        claw       = hardwareMap.get(Servo.class, "claw");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        claw.setPosition(0.5); // grip preloaded sample

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Init loop — runs until START pressed
        while (opModeInInit()) {
            telemetry.addData("Status", "Waiting for start...");
            telemetry.update();
        }

        // PLAY PHASE — waitForStart() has been replaced by opModeInInit() above
        runtime.reset();

        // Drive forward
        setDrivePower(0.5, 0.5, 0.5, 0.5);
        sleep(1000);

        // Stop
        setDrivePower(0, 0, 0, 0);

        // Strafe right
        setDrivePower(0.5, -0.5, -0.5, 0.5);
        sleep(500);

        // Stop and release
        setDrivePower(0, 0, 0, 0);
        claw.setPosition(0.0); // open claw
        sleep(500);

        telemetry.addData("Status", "Auto Complete in %.1f sec", runtime.seconds());
        telemetry.update();
    }

    private void setDrivePower(double fl, double fr, double bl, double br) {
        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);
    }
}
\`\`\`

### OpMode Annotations

\`\`\`java
// Autonomous OpMode — appears in Autonomous dropdown on Driver Station
@Autonomous(name = "Blue Left Auto", group = "Competition")
public class BlueLeftAuto extends LinearOpMode { ... }

// TeleOp OpMode — appears in TeleOp dropdown on Driver Station
@TeleOp(name = "Main TeleOp", group = "Competition")
public class MainTeleOp extends OpMode { ... }

// Disable an OpMode — hides it from the Driver Station without deleting code
@TeleOp(name = "Old TeleOp", group = "Deprecated")
@Disabled
public class OldTeleOp extends OpMode { ... }
\`\`\`

**Annotation Parameters:**
- \`name\` — Display name on Driver Station (required, should be descriptive)
- \`group\` — Groups OpModes together in the dropdown (optional but recommended)
- \`@Disabled\` — Hides the OpMode from the Driver Station list

### Choosing Between OpMode Types
| Criteria | Iterative (OpMode) | Linear (LinearOpMode) |
|---|---|---|
| Control flow | Callback-based (FSM) | Sequential (top-to-bottom) |
| sleep() safe? | NO — blocks the loop | YES — natural flow |
| TeleOp | Preferred | Works fine |
| Complex auto | FSM pattern needed | Simpler sequential code |
| Parallel actions | Natural with states | Requires threads or FSM overlay |
| Pedro Pathing | Works great with FSM | Works with built-in follower loop |
`,

  hardwareMap: `
## hardwareMap Usage

The \`hardwareMap\` object is available in all OpModes. It maps configured device names (from the
Robot Configuration on the Driver Station) to Java hardware objects.

### Basic Pattern
\`\`\`java
DeviceType variable = hardwareMap.get(DeviceType.class, "configName");
\`\`\`

The \`"configName"\` MUST match the name in the robot's active configuration on the Driver Station exactly.

### All Hardware Types

#### Motors
\`\`\`java
// Basic DC Motor
DcMotor motor = hardwareMap.get(DcMotor.class, "motor");

// Extended DC Motor (adds velocity control, current monitoring, PID access)
DcMotorEx motorEx = hardwareMap.get(DcMotorEx.class, "motor");

// You can also cast:
DcMotorEx motorEx = (DcMotorEx) hardwareMap.get(DcMotor.class, "motor");
\`\`\`

**Motor Configuration:**
\`\`\`java
motor.setDirection(DcMotor.Direction.FORWARD);   // or REVERSE
motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // or FLOAT
motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);   // raw power
motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);     // velocity-regulated
motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // reset encoder count
motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);        // PID to target

// RUN_TO_POSITION pattern (order matters!):
motor.setTargetPosition(1000);                          // SET TARGET FIRST
motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);        // THEN set mode
motor.setPower(0.5);                                    // THEN set power
while (motor.isBusy()) { /* wait */ }                   // Wait for completion
motor.setPower(0);
\`\`\`

#### Servos
\`\`\`java
// Standard Servo (position 0.0 to 1.0)
Servo servo = hardwareMap.get(Servo.class, "servo");
servo.setPosition(0.5); // move to center

// Extended Servo (adds PWM range control)
ServoImplEx servoEx = hardwareMap.get(ServoImplEx.class, "servo");
servoEx.setPwmRange(new PwmControl.PwmRange(500, 2500)); // custom PWM range
servoEx.setPwmDisable(); // disable PWM signal (servo goes limp)
servoEx.setPwmEnable();  // re-enable PWM signal

// Casting pattern:
ServoImplEx servoEx = (ServoImplEx) hardwareMap.get(Servo.class, "servo");

// Continuous Rotation Servo (power -1.0 to 1.0, no position control)
CRServo crServo = hardwareMap.get(CRServo.class, "crServo");
crServo.setPower(0.5); // spin at half speed
crServo.setPower(0);   // stop
\`\`\`

#### IMU (Inertial Measurement Unit)
\`\`\`java
IMU imu = hardwareMap.get(IMU.class, "imu");

// Configure orientation based on how the Control Hub is mounted
imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
    RevHubOrientationOnRobot.LogoFacingDirection.UP,
    RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
)));

// Reset yaw
imu.resetYaw();

// Read angles (in degrees)
YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
double heading = angles.getYaw(AngleUnit.DEGREES);
double pitch   = angles.getPitch(AngleUnit.DEGREES);
double roll    = angles.getRoll(AngleUnit.DEGREES);

// Read angular velocity
AngularVelocity angVel = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
double yawRate = angVel.zRotationRate;
\`\`\`

#### Distance Sensor
\`\`\`java
DistanceSensor distanceSensor = hardwareMap.get(DistanceSensor.class, "distance");
double distCM = distanceSensor.getDistance(DistanceUnit.CM);
double distIN = distanceSensor.getDistance(DistanceUnit.INCH);

// REV Color/Distance sensor also implements DistanceSensor:
// Configure as "REV Color Sensor V3" in config, access as DistanceSensor
\`\`\`

#### Color Sensor
\`\`\`java
NormalizedColorSensor colorSensor = hardwareMap.get(NormalizedColorSensor.class, "color");
colorSensor.setGain(2.0f); // optional gain adjustment

NormalizedRGBA colors = colorSensor.getNormalizedColors();
float red   = colors.red;
float green = colors.green;
float blue  = colors.blue;
float alpha = colors.alpha;

// REV Color Sensor V3 can also be used as a DistanceSensor (dual interface):
DistanceSensor distFromColor = (DistanceSensor) hardwareMap.get(NormalizedColorSensor.class, "color");
// OR
DistanceSensor distFromColor = hardwareMap.get(DistanceSensor.class, "color");
\`\`\`

#### Touch Sensor
\`\`\`java
TouchSensor touchSensor = hardwareMap.get(TouchSensor.class, "touch");
boolean isPressed = touchSensor.isPressed();
\`\`\`

#### Digital Channel
\`\`\`java
DigitalChannel digitalInput = hardwareMap.get(DigitalChannel.class, "digitalInput");
digitalInput.setMode(DigitalChannel.Mode.INPUT);
boolean state = digitalInput.getState(); // true = HIGH, false = LOW

// Magnetic limit switch example:
DigitalChannel limitSwitch = hardwareMap.get(DigitalChannel.class, "limitSwitch");
limitSwitch.setMode(DigitalChannel.Mode.INPUT);
boolean isTriggered = !limitSwitch.getState(); // Often active-low
\`\`\`

#### Analog Input
\`\`\`java
AnalogInput analogSensor = hardwareMap.get(AnalogInput.class, "analog");
double voltage = analogSensor.getVoltage();       // 0 to 3.3V
double maxVoltage = analogSensor.getMaxVoltage();  // typically 3.3V

// Potentiometer example:
double position = analogSensor.getVoltage() / analogSensor.getMaxVoltage(); // 0.0 to 1.0
\`\`\`

#### Webcam
\`\`\`java
WebcamName webcam = hardwareMap.get(WebcamName.class, "Webcam 1");
// Used with VisionPortal for AprilTag detection, TFOD, or custom processors
\`\`\`

#### GoBilda Pinpoint Odometry Computer
\`\`\`java
GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

// Configure encoder resolution (ticks per mm)
pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
// OR for custom pods:
// pinpoint.setEncoderResolution(13.26291192); // ticks per mm

// Set encoder directions
pinpoint.setEncoderDirections(
    GoBildaPinpointDriver.EncoderDirection.FORWARD,
    GoBildaPinpointDriver.EncoderDirection.FORWARD
);

// Set offsets from robot center (in mm)
pinpoint.setOffsets(-84.0, -168.0); // x offset, y offset

pinpoint.resetPosAndIMU(); // call in init

// In loop:
pinpoint.update(); // MUST call every loop
Pose2D pose = pinpoint.getPosition();
double x = pose.getX(DistanceUnit.MM);
double y = pose.getY(DistanceUnit.MM);
double heading = pose.getHeading(AngleUnit.RADIANS);
\`\`\`

#### SparkFun Optical Tracking Odometry Sensor (OTOS)
\`\`\`java
SparkFunOTOS otos = hardwareMap.get(SparkFunOTOS.class, "otos");

otos.setLinearUnit(DistanceUnit.INCH);
otos.setAngularUnit(AngleUnit.DEGREES);

// Offset from robot center
SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 0, 0);
otos.setOffset(offset);

otos.setLinearScalar(1.0);
otos.setAngularScalar(1.0);

otos.calibrateImu();
otos.resetTracking();

// In loop:
SparkFunOTOS.Pose2D pos = otos.getPosition();
double x = pos.x;
double y = pos.y;
double h = pos.h;
\`\`\`

#### Limelight 3A
\`\`\`java
Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
limelight.setPollRateHz(100);
limelight.start();
limelight.pipelineSwitch(0); // switch to pipeline 0

// In loop:
LLResult result = limelight.getLatestResult();
if (result != null && result.isValid()) {
    double tx = result.getTx(); // horizontal offset
    double ty = result.getTy(); // vertical offset
    double ta = result.getTa(); // target area

    // AprilTag results
    List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
    for (LLResultTypes.FiducialResult fr : fiducials) {
        int id = fr.getFiducialId();
        // fr.getRobotPoseFieldSpace() etc.
    }
}
\`\`\`

### Bulk Read Optimization (LynxModule)
\`\`\`java
// Get all Lynx modules (Control Hub + Expansion Hubs)
List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

// Set bulk caching mode to MANUAL for best performance
for (LynxModule hub : allHubs) {
    hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
}

// In your loop, clear cache ONCE at the start:
for (LynxModule hub : allHubs) {
    hub.clearBulkCache();
}
// All subsequent hardware reads in this loop iteration use cached values
// This dramatically reduces I2C/USB traffic and speeds up loop times
\`\`\`

**Bulk Caching Modes:**
- \`OFF\` — Default. Every read is a separate USB transaction (slow).
- \`AUTO\` — Caches reads, auto-clears when a new read of same type is requested. Good for simple OpModes.
- \`MANUAL\` — You must call \`clearBulkCache()\` each loop. Best performance but you manage the cache.

### Common Casting Patterns
\`\`\`java
// DcMotor → DcMotorEx (adds velocity PID, current reading)
DcMotorEx motorEx = (DcMotorEx) hardwareMap.get(DcMotor.class, "motor");

// Servo → ServoImplEx (adds PWM range control, PWM disable)
ServoImplEx servoEx = (ServoImplEx) hardwareMap.get(Servo.class, "servo");

// ColorSensor → DistanceSensor (REV Color Sensor V3 dual interface)
DistanceSensor dist = (DistanceSensor) hardwareMap.get(NormalizedColorSensor.class, "color");

// These all work because the underlying hardware implementations implement multiple interfaces
\`\`\`
`,

  gamepadApi: `
## Gamepad API

FTC provides two gamepads: \`gamepad1\` (primary driver) and \`gamepad2\` (secondary/operator).
Both are available as fields in any OpMode.

### Joysticks (Analog Sticks)
All stick values range from **-1.0 to 1.0**.

\`\`\`java
gamepad1.left_stick_x   // Left stick horizontal: -1.0 (left) to 1.0 (right)
gamepad1.left_stick_y   // Left stick vertical:   -1.0 (UP) to 1.0 (DOWN) ← INVERTED!
gamepad1.right_stick_x  // Right stick horizontal: -1.0 (left) to 1.0 (right)
gamepad1.right_stick_y  // Right stick vertical:   -1.0 (UP) to 1.0 (DOWN) ← INVERTED!
\`\`\`

**CRITICAL: Y-axis is inverted!** Pushing the stick FORWARD (up) gives a NEGATIVE value.
This means for driving forward, you almost always negate the Y value:
\`\`\`java
double drive = -gamepad1.left_stick_y; // Now forward = positive
\`\`\`

### Buttons (Digital — boolean)
\`\`\`java
gamepad1.a               // A button (Xbox) / Cross (PS)
gamepad1.b               // B button (Xbox) / Circle (PS)
gamepad1.x               // X button (Xbox) / Square (PS)
gamepad1.y               // Y button (Xbox) / Triangle (PS)

gamepad1.dpad_up         // D-Pad up
gamepad1.dpad_down       // D-Pad down
gamepad1.dpad_left       // D-Pad left
gamepad1.dpad_right      // D-Pad right

gamepad1.left_bumper     // Left bumper (LB)
gamepad1.right_bumper    // Right bumper (RB)

gamepad1.left_stick_button   // Left stick click (L3)
gamepad1.right_stick_button  // Right stick click (R3)

gamepad1.back            // Back/Select button
gamepad1.guide           // Guide/Home button (may not work on all controllers)
gamepad1.start           // Start button
\`\`\`

### Triggers (Analog — float)
Trigger values range from **0.0 (released) to 1.0 (fully pressed)**.

\`\`\`java
gamepad1.left_trigger    // Left trigger:  0.0 to 1.0
gamepad1.right_trigger   // Right trigger: 0.0 to 1.0
\`\`\`

### Gamepad1 vs Gamepad2
- \`gamepad1\` — Connected to the FIRST gamepad slot on the Driver Station. Typically the **driver** (drivetrain control).
- \`gamepad2\` — Connected to the SECOND slot. Typically the **operator** (mechanisms: arm, claw, intake, etc.).
- Both gamepads have identical APIs.
- During competition, gamepads are assigned at the Driver Station.

### Common Drive Patterns

**Mecanum Drive (most common FTC drivetrain):**
\`\`\`java
double drive  = -gamepad1.left_stick_y;   // Forward/backward (negated!)
double strafe =  gamepad1.left_stick_x;   // Left/right strafe
double rotate =  gamepad1.right_stick_x;  // Rotation

double fl = drive + strafe + rotate;
double fr = drive - strafe - rotate;
double bl = drive - strafe + rotate;
double br = drive + strafe - rotate;

// Normalize so no motor exceeds 1.0
double max = Math.max(1.0, Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
                                     Math.max(Math.abs(bl), Math.abs(br))));
frontLeft.setPower(fl / max);
frontRight.setPower(fr / max);
backLeft.setPower(bl / max);
backRight.setPower(br / max);
\`\`\`

**Tank Drive:**
\`\`\`java
double leftPower  = -gamepad1.left_stick_y;
double rightPower = -gamepad1.right_stick_y;
leftMotor.setPower(leftPower);
rightMotor.setPower(rightPower);
\`\`\`

**Arcade Drive:**
\`\`\`java
double drive  = -gamepad1.left_stick_y;
double rotate =  gamepad1.right_stick_x;
leftMotor.setPower(drive + rotate);
rightMotor.setPower(drive - rotate);
\`\`\`

### Button Edge Detection (Toggle Pattern)
Gamepads report CURRENT state each loop. For toggles, detect the rising edge:

\`\`\`java
// Fields
private boolean previousA = false;
private boolean toggleState = false;

// In loop():
boolean currentA = gamepad1.a;
if (currentA && !previousA) {
    // Rising edge — button was just pressed
    toggleState = !toggleState;
}
previousA = currentA;
\`\`\`

### Gamepad Deadzone
Sticks may not return exactly 0 when released. Apply a deadzone:

\`\`\`java
private double applyDeadzone(double value, double deadzone) {
    return Math.abs(value) < deadzone ? 0.0 : value;
}

// Usage:
double drive = applyDeadzone(-gamepad1.left_stick_y, 0.05);
\`\`\`

### Trigger as Button
\`\`\`java
boolean leftTriggerPressed = gamepad1.left_trigger > 0.5;
\`\`\`
`,

  bestPractices: `
## Best Practices for FTC Coding

### Package Structure
Organize your TeamCode directory with sub-packages:

\`\`\`
org.firstinspires.ftc.teamcode/
├── opmodes/              # All OpModes (TeleOp, Autonomous)
│   ├── auto/             # Autonomous OpModes
│   └── teleop/           # TeleOp OpModes
├── subsystems/           # Hardware subsystems (Drivetrain, Arm, Intake, etc.)
├── util/                 # Utility classes (PID controllers, math helpers, etc.)
└── pathing/              # Path following code, trajectories, waypoints
\`\`\`

### Naming Conventions
- **UPPER_SNAKE_CASE** — For \`@Config\` (FTC Dashboard) static fields:
  \`\`\`java
  @Config
  public class ArmConstants {
      public static double ARM_KP = 0.01;
      public static double ARM_KI = 0.0;
      public static double ARM_KD = 0.001;
      public static int ARM_TARGET_HIGH = 2500;
  }
  \`\`\`
- **camelCase** — For methods and local/instance variables:
  \`\`\`java
  public void setArmPosition(int targetTicks) { ... }
  private double currentPower = 0.0;
  \`\`\`
- **PascalCase** — For class names:
  \`\`\`java
  public class MecanumDrivetrain { ... }
  public class BlueLeftAuto extends LinearOpMode { ... }
  \`\`\`
- **Config names** — Use camelCase in robot configuration:
  \`\`\`
  frontLeft, frontRight, backLeft, backRight, armMotor, clawServo
  \`\`\`

### Common Mistakes and How to Avoid Them

#### 1. Forgetting follower.update() (Pedro Pathing)
\`\`\`java
// WRONG — robot won't move, path won't execute
follower.followPath(path);
// ... nothing happens

// CORRECT — must call update() every loop iteration
@Override
public void loop() {
    follower.update();  // CRITICAL: call every loop!
    // rest of your code
}
\`\`\`

#### 2. Caching @Config Values at Init Time
\`\`\`java
// WRONG — value is captured at init, Dashboard changes won't take effect
private double kP = ArmConstants.ARM_KP;

public void loop() {
    double output = kP * error; // Always uses initial value
}

// CORRECT — read the static field each loop for live tuning
public void loop() {
    double output = ArmConstants.ARM_KP * error; // Picks up Dashboard changes
}
\`\`\`

#### 3. Using Thread.sleep() in Iterative OpMode
\`\`\`java
// WRONG — blocks the entire loop, stops ALL updates including telemetry
@Override
public void loop() {
    motor.setPower(1.0);
    Thread.sleep(1000);  // NEVER DO THIS in OpMode.loop()!
    motor.setPower(0.0);
}

// CORRECT — use a timer / state machine
private ElapsedTime timer = new ElapsedTime();
private enum State { DRIVING, STOPPED }
private State state = State.DRIVING;

@Override
public void start() {
    timer.reset();
    state = State.DRIVING;
    motor.setPower(1.0);
}

@Override
public void loop() {
    switch (state) {
        case DRIVING:
            if (timer.seconds() > 1.0) {
                motor.setPower(0.0);
                state = State.STOPPED;
            }
            break;
        case STOPPED:
            break;
    }
}
\`\`\`

#### 4. RUN_TO_POSITION Without setTargetPosition First
\`\`\`java
// WRONG — may run to position 0 or cause errors
motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
motor.setTargetPosition(1000);
motor.setPower(0.5);

// CORRECT — set target BEFORE setting mode
motor.setTargetPosition(1000);                      // 1. Set target
motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);     // 2. Set mode
motor.setPower(0.5);                                 // 3. Set power
\`\`\`

#### 5. Not Negating Gamepad Y Stick
\`\`\`java
// WRONG — forward on stick = robot goes backward
double drive = gamepad1.left_stick_y;

// CORRECT — negate Y so forward stick = positive value = forward motion
double drive = -gamepad1.left_stick_y;
\`\`\`

#### 6. Not Checking opModeIsActive() in LinearOpMode Loops
\`\`\`java
// WRONG — loop continues even after STOP is pressed
while (motor.isBusy()) {
    // keeps running forever if you hit STOP
}

// CORRECT — always check opModeIsActive()
while (opModeIsActive() && motor.isBusy()) {
    idle();
}
\`\`\`

#### 7. Forgetting Direction Reversal on One Side
\`\`\`java
// WRONG — one side of drivetrain runs backward
frontLeft.setPower(1.0);
frontRight.setPower(1.0); // Robot turns instead of going straight

// CORRECT — reverse one side (typically left for most configurations)
frontLeft.setDirection(DcMotor.Direction.REVERSE);
backLeft.setDirection(DcMotor.Direction.REVERSE);
\`\`\`

### Competition-Specific Rules

#### RS09 — No FTC Dashboard in Competition
- FTC Dashboard (the web-based tuning interface) is NOT allowed during competition matches.
- You MUST NOT have Dashboard actively serving during competition.
- However, you CAN keep the Dashboard library in your code — just don't access the web UI during matches.
- Dashboard is only for practice/tuning sessions.

#### Disable Wi-Fi Direct on Expansion Hub
- If using an Expansion Hub connected to a Control Hub, Wi-Fi Direct on the Expansion Hub MUST be disabled.
- The Expansion Hub connects via RS-485, not Wi-Fi.
- Having Wi-Fi Direct enabled on the Expansion Hub can cause interference and connectivity issues.
- Disable it via the REV Hardware Client or the Robot Controller settings.

### General Best Practices
- **Use Bulk Reads**: Always enable LynxModule bulk caching (MANUAL mode) for faster loop times.
- **Keep loop() fast**: Target <20ms per loop. No blocking calls, no heavy computation.
- **Use subsystem classes**: Encapsulate hardware in subsystem classes, not raw in OpModes.
- **ElapsedTime for timing**: Use \`ElapsedTime\` instead of \`System.currentTimeMillis()\`.
- **Telemetry for debugging**: Use \`telemetry.addData()\` liberally during development.
- **Zero power on stop**: Always set motors to 0 power in \`stop()\` or end of \`runOpMode()\`.
- **Brake mode for arms/lifts**: Use \`BRAKE\` zero power behavior for mechanisms that fight gravity.
- **FLOAT mode for drivetrain**: Sometimes preferred for driver feel during TeleOp.
- **Version control**: Use Git. Commit working code before making changes.
- **Comment your code**: Future you (and your teammates) will thank you.
`
};
