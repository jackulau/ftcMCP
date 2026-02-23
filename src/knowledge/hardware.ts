export const HARDWARE_KNOWLEDGE = {
  motorsApi: `
## DcMotor & DcMotorEx API

### Instantiation
\`\`\`java
// Always use DcMotorEx for full functionality
DcMotorEx motor = hardwareMap.get(DcMotorEx.class, "motorName");
\`\`\`

### DcMotor Interface
\`\`\`java
// Power control: range is -1.0 to 1.0
motor.setPower(0.75);           // Set motor power
double p = motor.getPower();    // Get current power setting

// Direction
motor.setDirection(DcMotorSimple.Direction.FORWARD);
motor.setDirection(DcMotorSimple.Direction.REVERSE);

// Run modes (see motorRunModes for details)
motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

// Zero power behavior
motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);  // Active braking
motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);  // Coast

// Encoder
int ticks = motor.getCurrentPosition();    // Current encoder ticks
motor.setTargetPosition(1000);             // Set target for RUN_TO_POSITION
boolean busy = motor.isBusy();             // True if RUN_TO_POSITION in progress

// Controller info
DcMotorController ctrl = motor.getController();
int port = motor.getPortNumber();
\`\`\`

### DcMotorEx Interface (extends DcMotor)
\`\`\`java
// Velocity control (ticks per second)
motor.setVelocity(1000);                          // Set velocity in ticks/sec
motor.setVelocity(360, AngleUnit.DEGREES);         // Set velocity in degrees/sec
double vel = motor.getVelocity();                  // Get velocity in ticks/sec
double velDeg = motor.getVelocity(AngleUnit.DEGREES);

// Current monitoring
double current = motor.getCurrent(CurrentUnit.AMPS);
double currentMa = motor.getCurrent(CurrentUnit.MILLIAMPS);
boolean overCurrent = motor.isOverCurrent();

// PIDF coefficient control
// CRITICAL: PIDF output range is internal 16-bit short (-32767 to 32767), NOT -1 to 1
// The SDK maps this internally to motor power
motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
    new PIDFCoefficients(10, 3, 0, 12));
PIDFCoefficients coeffs = motor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
// coeffs.p, coeffs.i, coeffs.d, coeffs.f

// Motor enable/disable (cuts power without changing power setting)
motor.setMotorDisable();
motor.setMotorEnable();
boolean enabled = motor.isMotorEnabled();
\`\`\`

### Complete Motor Initialization Example
\`\`\`java
DcMotorEx motor = hardwareMap.get(DcMotorEx.class, "liftMotor");
motor.setDirection(DcMotorSimple.Direction.REVERSE);
motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
motor.setPower(0);
\`\`\`
`,

  motorRunModes: `
## Motor RunModes Detailed

### RUN_WITHOUT_ENCODER
- Applies direct voltage fraction to the motor
- setPower(0.5) = 50% of battery voltage
- The encoder is STILL READABLE via getCurrentPosition() even in this mode
- Most common mode for drivetrain teleop
- No closed-loop control — power is open-loop voltage
\`\`\`java
motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
motor.setPower(0.8); // 80% voltage
int pos = motor.getCurrentPosition(); // Still works!
\`\`\`

### RUN_USING_ENCODER
- PIDF velocity control mode
- setPower(1.0) means "max velocity" NOT "max voltage"
- setPower(0.5) means 50% of max velocity
- The hub runs a PIDF loop to maintain consistent velocity
- Better for consistent speed across battery voltage changes
- Slightly slower max speed than RUN_WITHOUT_ENCODER (reserves headroom for control)
\`\`\`java
motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
motor.setPower(1.0); // Target max velocity, not max voltage
\`\`\`

### RUN_TO_POSITION
- PID position control mode
- CRITICAL: You MUST call setTargetPosition() BEFORE setMode(RUN_TO_POSITION)
- setPower() sets the MAXIMUM SPEED (always use positive values, direction determined by target)
- Use isBusy() to check if the motor has reached its target
- Motor holds position after reaching target
\`\`\`java
// Correct order — target BEFORE mode
motor.setTargetPosition(2000);
motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
motor.setPower(0.5); // Max speed (always positive!)

while (opModeIsActive() && motor.isBusy()) {
    telemetry.addData("Position", motor.getCurrentPosition());
    telemetry.update();
}
motor.setPower(0);
\`\`\`

### STOP_AND_RESET_ENCODER
- Resets encoder count to 0
- This is NOT a persistent running mode — set another mode after resetting
- Motor power is set to 0 while in this mode
\`\`\`java
motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
\`\`\`
`,

  motorSpecs: `
## Motor Specifications — Exact Counts Per Revolution (CPR)

### Formula
\`\`\`
ticks_per_rev = bare_encoder_CPR × actual_gear_ratio
\`\`\`

### goBILDA Yellow Jacket Motors (28 bare encoder CPR)
| Gear Ratio | CPR (ticks/rev) | Free Speed (RPM) |
|-----------|-----------------|-------------------|
| 1:1       | 28              | 6000              |
| 5.2:1     | 145.6           | 1150              |
| 13.7:1    | 383.6           | 435               |
| 19.2:1    | 537.6           | 312               |
| 26.9:1    | 753.2           | 223               |
| 50.9:1    | 1425.2          | 117               |
| 71.2:1    | 1993.6          | 84                |
| 99.5:1    | 2786            | 60                |
| 139:1     | 3892            | 43                |
| 188:1     | 5264            | 30                |

### REV HD Hex Motor (28 bare encoder CPR)
| Gear Ratio | CPR (ticks/rev) |
|-----------|-----------------|
| 20:1      | 560             |
| 40:1      | 1120            |

### REV Core Hex Motor
- Bare encoder: 4 CPR
- Internal gear ratio: 72:1
- Total CPR: 4 × 72 = 288

### REV UltraPlanetary
- CRITICAL: Actual gear ratios differ from nominal labels!
- Bare encoder: 28 CPR (uses HD Hex motor)
| Nominal | Actual Ratio | CPR           |
|---------|-------------|---------------|
| "3:1"   | 3.61:1      | 28 × 3.61 = 101.08   |
| "4:1"   | 5.23:1      | 28 × 5.23 = 146.44   |
| "5:1"   | 5.23:1      | 28 × 5.23 = 146.44   |
- Compound ratios: multiply actual ratios together
- Example: "3:1" × "4:1" = 3.61 × 5.23 = 18.88:1 → 28 × 18.88 = 528.64 CPR

### COUNTS_PER_INCH Calculation
\`\`\`java
// For a wheel/spool driven directly by the motor
static final double COUNTS_PER_MOTOR_REV = 537.6; // goBILDA 19.2:1
static final double WHEEL_DIAMETER_INCHES = 4.0;   // e.g., mecanum wheel
static final double COUNTS_PER_INCH =
    COUNTS_PER_MOTOR_REV / (WHEEL_DIAMETER_INCHES * Math.PI);
// 537.6 / (4.0 * 3.14159) = ~42.8 ticks per inch

// For geared mechanisms, multiply by external gear ratio
static final double EXTERNAL_GEAR_RATIO = 2.0; // driven gear / driving gear
static final double COUNTS_PER_INCH_GEARED =
    (COUNTS_PER_MOTOR_REV * EXTERNAL_GEAR_RATIO) / (WHEEL_DIAMETER_INCHES * Math.PI);
\`\`\`
`,

  servosApi: `
## Servo, ServoImplEx, and CRServo API

### Standard Servo
\`\`\`java
Servo servo = hardwareMap.get(Servo.class, "servoName");

// Position control: 0.0 to 1.0
servo.setPosition(0.5);           // Move to center
double pos = servo.getPosition();  // Returns TARGET position, NOT actual physical position

// Scale range — remaps 0-1 to a sub-range of the full PWM sweep
servo.scaleRange(0.2, 0.8);       // Now setPosition(0) = 0.2, setPosition(1) = 0.8

// Direction
servo.setDirection(Servo.Direction.FORWARD);
servo.setDirection(Servo.Direction.REVERSE);
\`\`\`

### ServoImplEx (advanced PWM control)
\`\`\`java
ServoImplEx servo = hardwareMap.get(ServoImplEx.class, "servoName");

// Custom PWM range (microseconds)
// Default is 600-2400us, max safe range is 500-2500us
servo.setPwmRange(new PwmControl.PwmRange(500, 2500));

// Disable/enable PWM signal (servo goes limp when disabled)
servo.setPwmDisable();   // Stop sending PWM — servo relaxes
servo.setPwmEnable();    // Resume sending PWM
boolean active = servo.isPwmEnabled();
\`\`\`

### Continuous Rotation Servo (CRServo)
\`\`\`java
CRServo crServo = hardwareMap.get(CRServo.class, "crServoName");

// Power: -1.0 to 1.0
crServo.setPower(0.5);   // Forward at half speed
crServo.setPower(-0.5);  // Reverse at half speed
crServo.setPower(0);     // Stop

crServo.setDirection(DcMotorSimple.Direction.REVERSE);
\`\`\`

### PWM Details
- PWM frequency: 50 Hz (20ms period)
- Default pulse range: 600–2400 microseconds
- Maximum safe pulse range: 500–2500 microseconds
- Servo power is 5V, paired on ports: (0-1), (2-3), (4-5)
- Each pair shares a 2A current limit
- If two high-torque servos on the same pair draw >2A total, the hub may brown out
- Solution: spread high-current servos across different pairs

### Servo Initialization Example
\`\`\`java
ServoImplEx clawServo = hardwareMap.get(ServoImplEx.class, "claw");
clawServo.setPwmRange(new PwmControl.PwmRange(500, 2500));
clawServo.setDirection(Servo.Direction.FORWARD);
clawServo.setPosition(0.0); // Start at closed position
\`\`\`
`,

  sensorsImu: `
## IMU (Inertial Measurement Unit) API

### Universal IMU Interface (SDK 8.1+)
Works with both BNO055 (Control Hub v1 / Expansion Hub) and BHI260AP (Control Hub v2).

### Instantiation & Initialization
\`\`\`java
IMU imu = hardwareMap.get(IMU.class, "imu");

// CRITICAL: Orientation must use orthogonal directions (aligned to hub axes)
IMU.Parameters params = new IMU.Parameters(
    new RevHubOrientationOnRobot(
        RevHubOrientationOnRobot.LogoFacingDirection.UP,
        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
    )
);
imu.initialize(params);

// Alternative: specify exact orientation with quaternion
// Orientation orientation = new Orientation(AxesReference.INTRINSIC, AxesOrder.ZYX,
//     AngleUnit.DEGREES, 0, 0, 0, 0);
// IMU.Parameters params = new IMU.Parameters(
//     new RevHubOrientationOnRobot(orientation));
\`\`\`

### Reading Orientation
\`\`\`java
// Reset yaw to 0 (does NOT reset pitch/roll)
imu.resetYaw();

// Get yaw (heading) — CCW positive (mathematical convention)
YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
double yaw = angles.getYaw(AngleUnit.DEGREES);     // -180 to +180, CCW positive
double pitch = angles.getPitch(AngleUnit.DEGREES);
double roll = angles.getRoll(AngleUnit.DEGREES);

// Angular velocity
AngularVelocity angVel = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
double yawRate = angVel.zRotationRate;
double pitchRate = angVel.xRotationRate;
double rollRate = angVel.yRotationRate;
\`\`\`

### Performance Notes
- I2C communication: ~7ms per read
- BNO055 vs BHI260AP differences are abstracted by the IMU interface
- BHI260AP (newer Control Hub) has better drift characteristics
- IMU reads are NOT included in bulk reads — each read is a separate I2C transaction
- For high-frequency heading, consider GoBilda Pinpoint or OTOS instead
`,

  sensorsDistance: `
## Distance Sensors

### DistanceSensor Interface / Rev2mDistanceSensor
Based on VL53L0X Time-of-Flight sensor.

\`\`\`java
DistanceSensor distSensor = hardwareMap.get(DistanceSensor.class, "distance");

double distCm = distSensor.getDistance(DistanceUnit.CM);
double distIn = distSensor.getDistance(DistanceUnit.INCH);
double distMm = distSensor.getDistance(DistanceUnit.MM);
double distM = distSensor.getDistance(DistanceUnit.METER);
\`\`\`

### Specifications
- Sensor IC: VL53L0X Time-of-Flight laser ranging
- Effective range: 2 cm to ~200 cm (accuracy degrades beyond ~120 cm)
- Interface: I2C
- Returns DistanceUnit.infinity if no target detected / out of range
- Each read is an I2C transaction (~7ms) — NOT included in bulk reads
- Multiple distance sensors on same I2C bus: use different addresses or different I2C ports

### Usage Example
\`\`\`java
Rev2mDistanceSensor distSensor = hardwareMap.get(Rev2mDistanceSensor.class, "frontDist");

if (distSensor.getDistance(DistanceUnit.CM) < 10) {
    // Object detected within 10cm
    motor.setPower(0);
}
\`\`\`
`,

  sensorsColor: `
## Color / Distance Combo Sensor (REV Color Sensor V3)

### NormalizedColorSensor
\`\`\`java
NormalizedColorSensor colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");

// Set gain (amplification) — higher gain for darker surfaces
colorSensor.setGain(2.0f);

// Get normalized RGBA values (0.0 to 1.0 after normalization)
NormalizedRGBA colors = colorSensor.getNormalizedColors();
float red = colors.red;
float green = colors.green;
float blue = colors.blue;
float alpha = colors.alpha; // Overall intensity

// Convert to Android Color for HSV analysis
int color = colors.toColor();
float[] hsvValues = new float[3];
Color.colorToHSV(color, hsvValues);
float hue = hsvValues[0];        // 0-360
float saturation = hsvValues[1]; // 0-1
float value = hsvValues[2];      // 0-1
\`\`\`

### Proximity via DistanceSensor Cast
\`\`\`java
// REV Color Sensor V3 also implements DistanceSensor
DistanceSensor proxSensor = (DistanceSensor) colorSensor;
double distCm = proxSensor.getDistance(DistanceUnit.CM);
// Effective range: ~1-10cm for proximity detection
\`\`\`

### LED Control via SwitchableLight Cast
\`\`\`java
// Control the built-in LED
if (colorSensor instanceof SwitchableLight) {
    ((SwitchableLight) colorSensor).enableLight(true);  // Turn LED on
    ((SwitchableLight) colorSensor).enableLight(false); // Turn LED off
}
\`\`\`

### Color Detection Example
\`\`\`java
NormalizedColorSensor colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");
colorSensor.setGain(5.0f);

// In loop:
NormalizedRGBA colors = colorSensor.getNormalizedColors();
int color = colors.toColor();
float[] hsv = new float[3];
Color.colorToHSV(color, hsv);

String detected;
if (hsv[0] < 30 || hsv[0] > 330) {
    detected = "RED";
} else if (hsv[0] > 200 && hsv[0] < 260) {
    detected = "BLUE";
} else if (hsv[0] > 60 && hsv[0] < 100) {
    detected = "YELLOW";
} else {
    detected = "UNKNOWN";
}
\`\`\`
`,

  sensorsDigital: `
## Digital Sensors (Touch Sensors, Limit Switches)

### TouchSensor (high-level)
\`\`\`java
TouchSensor touchSensor = hardwareMap.get(TouchSensor.class, "touch");

boolean pressed = touchSensor.isPressed(); // true when pressed
double value = touchSensor.getValue();     // 0.0 or 1.0
\`\`\`

### DigitalChannel (low-level)
\`\`\`java
DigitalChannel digitalPin = hardwareMap.get(DigitalChannel.class, "limitSwitch");
digitalPin.setMode(DigitalChannel.Mode.INPUT);

boolean state = digitalPin.getState();
// CRITICAL: getState() returns TRUE when the pin is HIGH (NOT pressed)
//           getState() returns FALSE when the pin is LOW (pressed / grounded)
// This is inverted from what you might expect!
boolean isPressed = !digitalPin.getState();
\`\`\`

### Wiring
- CRITICAL: Wire the switch between the GROUND pin and the SIGNAL pin
- Do NOT wire to the voltage pin
- REV Hub digital ports have internal 2.49kΩ pull-up resistors to 3.3V
- When the switch is open: pin is pulled HIGH (true) — not pressed
- When the switch is closed: pin is pulled to GROUND (false) — pressed

### Complete Example
\`\`\`java
// Limit switch for a lift
DigitalChannel lowerLimit = hardwareMap.get(DigitalChannel.class, "liftLimit");
lowerLimit.setMode(DigitalChannel.Mode.INPUT);

// In loop:
if (!lowerLimit.getState()) {
    // Switch is pressed — at lower limit
    liftMotor.setPower(0);
    liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
} else {
    liftMotor.setPower(gamepad1.left_stick_y);
}
\`\`\`
`,

  sensorsEncoder: `
## External Encoders

### REV Through Bore Encoder
- Resolution: 8192 counts per revolution (CPR)
- Quadrature output — reads position AND direction
- Absolute output also available (but not typically used via motor port)

### Reading External Encoders
External encoders are read through motor ports using DcMotorEx.
The encoder shares the port with a motor — you can read the encoder even if a different motor is physically connected to that port (deadwheel configuration).

\`\`\`java
// Read encoder on motor port 0 (motor may or may not be connected)
DcMotorEx encoderPort = hardwareMap.get(DcMotorEx.class, "leftEncoder");
encoderPort.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
encoderPort.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

// In loop:
int position = encoderPort.getCurrentPosition();
double velocity = encoderPort.getVelocity(); // ticks per second
\`\`\`

### Hardware vs Software Decoded Ports
- Ports 0 and 3: HARDWARE decoded (quadrature decoding in silicon)
  - Accurate at all speeds, no missed counts
  - Use these for high-CPR encoders (REV Through Bore 8192 CPR)
- Ports 1 and 2: SOFTWARE decoded (processed by hub firmware)
  - Can drift/miss counts at high tick rates (>4000 CPR at high speed)
  - Fine for lower CPR encoders or slower mechanisms

### Velocity Overflow
- DcMotorEx.getVelocity() returns a 16-bit signed value internally
- Overflow occurs at 32767 ticks/sec (wraps to negative)
- REV Through Bore at high speed CAN overflow: 8192 CPR × 4 rev/sec = 32768 ticks/sec
- Corrected velocity formula:
\`\`\`java
public double getCorrectedVelocity(DcMotorEx motor) {
    double rawVel = motor.getVelocity();
    double CPR = 8192;
    // Check for overflow
    while (Math.abs(rawVel) > CPR / 2.0) {
        rawVel -= Math.signum(rawVel) * CPR;
    }
    return rawVel;
}
\`\`\`

### Best Practices
- Use ports 0 and 3 for high-CPR encoders (8192 CPR through bore)
- Use ports 1 and 2 for lower-CPR motor encoders or slow mechanisms
- If using deadwheels, the motor on that port still works — only the encoder signal is shared
- Always reset encoders in init: STOP_AND_RESET_ENCODER then RUN_WITHOUT_ENCODER
`,

  pinpoint: `
## GoBilda Pinpoint Odometry Computer — Complete API

### Overview
- Dedicated I2C odometry processor at address 0x31
- I2C bus speed: 400 kHz
- Internal fusion rate: ~1500 Hz (IMU + two encoder pods)
- Offloads all odometry math from the Control Hub
- Connects two odometry pod encoders + built-in IMU

### Instantiation
\`\`\`java
GoBildaPinpointDriver odo = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
\`\`\`

### Configuration
\`\`\`java
// Set odometry pod offsets from robot center (in mm)
// X offset: positive = pod is to the LEFT of center (forward-facing pod)
// Y offset: positive = pod is FORWARD of center (strafe-facing pod)
odo.setOffsets(-84.0, 168.0, DistanceUnit.MM);

// Set encoder resolution
// Option 1: Use predefined pod constants
odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);

// Option 2: Custom ticks per mm
odo.setEncoderResolution(19.89); // ticks per mm for your encoder+wheel combo

// Set encoder directions (if pods read backwards)
odo.setEncoderDirections(
    GoBildaPinpointDriver.EncoderDirection.FORWARD,   // X (forward) pod
    GoBildaPinpointDriver.EncoderDirection.REVERSED    // Y (strafe) pod
);

// Reset position and IMU — robot MUST be stationary, takes ~0.25s
odo.resetPosAndIMU();
// Wait for calibration to complete
Thread.sleep(300);
\`\`\`

### Loop Usage — MUST Call update()
\`\`\`java
// CRITICAL: Must call update() every loop iteration to fetch latest data
odo.update();

// Get full pose
Pose2D pose = odo.getPosition();
double x = pose.getX(DistanceUnit.INCH);
double y = pose.getY(DistanceUnit.INCH);
double heading = pose.getHeading(AngleUnit.DEGREES);

// Convenience getters
double posX = odo.getPosX(DistanceUnit.MM);
double posY = odo.getPosY(DistanceUnit.MM);
double h = odo.getHeading(AngleUnit.DEGREES);         // Normalized [-180, 180]
double hRaw = odo.getHeading(UnnormalizedAngleUnit.DEGREES); // Continuous (no wrap)

// Velocity
double velX = odo.getVelX(DistanceUnit.MM);  // mm/sec
double velY = odo.getVelY(DistanceUnit.MM);
double headingVel = odo.getHeadingVelocity(); // rad/sec
\`\`\`

### Position Correction (e.g., from AprilTag)
\`\`\`java
// Override current position (useful for AprilTag relocation)
odo.setPosition(new Pose2D(DistanceUnit.INCH, newX, newY, AngleUnit.DEGREES, newHeading));
\`\`\`

### Heading-Only Fast Read
\`\`\`java
// For situations where you only need heading and want faster reads
odo.update(GoBildaPinpointDriver.ReadData.ONLY_UPDATE_HEADING);
double heading = odo.getHeading(AngleUnit.DEGREES);
\`\`\`

### Device Status Monitoring
\`\`\`java
GoBildaPinpointDriver.DeviceStatus status = odo.getDeviceStatus();

// Possible statuses:
// NOT_READY         — device is initializing
// READY             — normal operation
// CALIBRATING       — IMU calibration in progress
// FAULT_X_POD_NOT_DETECTED   — X encoder not connected/detected
// FAULT_Y_POD_NOT_DETECTED   — Y encoder not connected/detected
// FAULT_IMU_RUNAWAY           — IMU readings are diverging
// FAULT_BAD_READ              — I2C communication error

if (status != GoBildaPinpointDriver.DeviceStatus.READY) {
    telemetry.addData("WARNING", "Pinpoint status: " + status);
}
\`\`\`

### Diagnostics
\`\`\`java
double freq = odo.getFrequency();       // Internal loop frequency (~1500 Hz)
double loopTime = odo.getLoopTime();    // Internal loop time in ms
int deviceId = odo.getDeviceID();
int version = odo.getDeviceVersion();
\`\`\`

### V2 Features
- CRC8 error checking on I2C data for reliability
- 3D orientation support
- Configurable read data selections

### Complete Initialization + Loop Example
\`\`\`java
@TeleOp(name = "Pinpoint Example")
public class PinpointExample extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        GoBildaPinpointDriver odo = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        // Configure
        odo.setOffsets(-84.0, 168.0, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(
            GoBildaPinpointDriver.EncoderDirection.FORWARD,
            GoBildaPinpointDriver.EncoderDirection.REVERSED
        );
        odo.resetPosAndIMU();
        Thread.sleep(300);

        telemetry.addData("Status", "Pinpoint initialized");
        telemetry.addData("Pinpoint Status", odo.getDeviceStatus());
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            odo.update(); // MUST be called every loop

            Pose2D pose = odo.getPosition();
            telemetry.addData("X (in)", pose.getX(DistanceUnit.INCH));
            telemetry.addData("Y (in)", pose.getY(DistanceUnit.INCH));
            telemetry.addData("Heading (deg)", pose.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Frequency", odo.getFrequency());
            telemetry.addData("Status", odo.getDeviceStatus());
            telemetry.update();
        }
    }
}
\`\`\`
`,

  otos: `
## SparkFun OTOS (Optical Tracking Odometry Sensor) — Complete API

### Overview
- I2C at address 0x17
- Optical floor tracking sensor + built-in IMU
- NO external encoders needed — tracks motion by imaging floor surface
- Surface dependent: works best on hard, reflective floors
- Working distance: 10–27mm from floor surface

### Instantiation
\`\`\`java
SparkFunOTOS otos = hardwareMap.get(SparkFunOTOS.class, "otos");
\`\`\`

### Configuration
\`\`\`java
// Set measurement units
otos.setLinearUnit(DistanceUnit.INCH);
otos.setAngularUnit(AngleUnit.DEGREES);

// Set sensor offset from robot center
// x = forward offset, y = left offset, h = heading offset (rotation of sensor)
SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 0, 0);
otos.setOffset(offset);

// Linear scalar: corrects linear distance measurement
// Valid range: 0.872 to 1.127
// If robot actually travels 24" but OTOS reads 23", scalar = 24/23 = 1.043
otos.setLinearScalar(1.0);

// Angular scalar: corrects angular measurement
// If robot actually rotates 3600° but OTOS reads 3540°, scalar = 3600/3540 = 1.017
otos.setAngularScalar(1.0);

// Calibrate IMU — robot MUST be still, takes ~612ms
otos.calibrateImu();
Thread.sleep(700);

// Reset tracking to origin
otos.resetTracking();

// Optionally set starting position
otos.setPosition(new SparkFunOTOS.Pose2D(0, 0, 0));
\`\`\`

### Loop Usage — NO update() Call Needed
\`\`\`java
// Unlike Pinpoint, OTOS does NOT require an update() call
SparkFunOTOS.Pose2D pose = otos.getPosition();
double x = pose.x;  // Direct field access
double y = pose.y;
double h = pose.h;   // Heading

// Velocity
SparkFunOTOS.Pose2D vel = otos.getVelocity();
double velX = vel.x;
double velY = vel.y;
double velH = vel.h;

// Acceleration
SparkFunOTOS.Pose2D accel = otos.getAcceleration();
double accelX = accel.x;
double accelY = accel.y;
\`\`\`

### Position Override
\`\`\`java
// Reset position (e.g., after AprilTag correction)
otos.setPosition(new SparkFunOTOS.Pose2D(24.0, 12.0, 90.0));
\`\`\`

### CRITICAL Notes
- NOTHING persists across power cycles — must reconfigure every init
- Surface dependent — poor tracking on carpet, soft, or non-reflective surfaces
- Working distance: 10–27mm — sensor must be close to floor
- I2C reads are NOT included in bulk reads

### Calibration Procedure (for best accuracy)
1. Angular scalar first:
   - Program robot to spin in place
   - Spin exactly 10 full rotations (3600°)
   - Read OTOS heading — divide 3600 by OTOS reading for angular scalar
2. Linear scalar second:
   - After angular scalar is set
   - Drive robot a known distance (e.g., 48 inches)
   - Read OTOS distance — divide actual by OTOS reading for linear scalar

### Complete Example
\`\`\`java
@TeleOp(name = "OTOS Example")
public class OTOSExample extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SparkFunOTOS otos = hardwareMap.get(SparkFunOTOS.class, "otos");

        otos.setLinearUnit(DistanceUnit.INCH);
        otos.setAngularUnit(AngleUnit.DEGREES);
        otos.setOffset(new SparkFunOTOS.Pose2D(0, 0, 0));
        otos.setLinearScalar(1.0);
        otos.setAngularScalar(1.0);
        otos.calibrateImu();
        Thread.sleep(700);
        otos.resetTracking();
        otos.setPosition(new SparkFunOTOS.Pose2D(0, 0, 0));

        telemetry.addData("Status", "OTOS initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            SparkFunOTOS.Pose2D pose = otos.getPosition();

            telemetry.addData("X (in)", pose.x);
            telemetry.addData("Y (in)", pose.y);
            telemetry.addData("Heading (deg)", pose.h);
            telemetry.update();
        }
    }
}
\`\`\`
`,

  revHub: `
## REV Hub Internals (LynxModule)

### Hub Configuration
- Maximum: 1 Control Hub + 1 Expansion Hub (connected via RS-485 or USB)
- Control Hub: has onboard Android computer + hub
- Expansion Hub: hub only, must connect to Control Hub

### Port Layout per Hub
| Port Type  | Count | Details                                      |
|-----------|-------|----------------------------------------------|
| Motor     | 4     | Ports 0-3, with encoder pins                 |
| Servo     | 6     | Ports 0-5, paired 5V power: (0-1),(2-3),(4-5)|
| Digital   | 8     | Ports 0-7, 3.3V logic, pull-up resistors     |
| Analog    | 4     | Ports 0-3, 0-5V tolerant (3.3V reference)    |
| I2C       | 4     | Buses 0-3, 3.3V                              |

### Encoder Port Details
- Ports 0 and 3: HARDWARE quadrature decoding (done in silicon)
  - Accurate at all speeds, zero missed counts
- Ports 1 and 2: SOFTWARE quadrature decoding (firmware processed)
  - Can miss counts at high tick rates (>4000 effective CPR at high speed)

### Processor & Timing
- ARM Cortex M4 @ 80 MHz
- Command latency:
  - Control Hub: ~2ms per command (internal USB)
  - Expansion Hub: ~3ms per command (external USB/RS-485)
- Master lock per USB bus — all commands are serialized
  - Multithreading does NOT help — commands are queued behind a single USB lock
  - The only way to reduce latency is to reduce the number of commands

### Bulk Caching Mode
\`\`\`java
List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
for (LynxModule hub : hubs) {
    hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
}

// In loop — clear cache at start of each iteration
for (LynxModule hub : hubs) {
    hub.clearBulkCache();
}
\`\`\`

### I2C Bus Notes
- 4 separate I2C buses per hub (bus 0-3)
- Each bus runs independently
- Distribute I2C sensors across buses to parallelize reads
- Default I2C bus 0 is used by the built-in IMU
`,

  bulkReads: `
## Bulk Reads — Detailed Guide

### What Are Bulk Reads?
A single I2C/USB command that fetches ALL digital input, analog input, encoder position, and encoder velocity data from the hub at once, instead of individual commands for each sensor.

### BulkCachingMode.OFF (Default)
- Every hardware read triggers a separate USB command (~2-3ms each)
- Reading 4 encoders = 4 separate commands = 8-12ms
- Simple but slow
\`\`\`java
hub.setBulkCachingMode(LynxModule.BulkCachingMode.OFF);
\`\`\`

### BulkCachingMode.AUTO
- First read of any bulk-readable value triggers a bulk read
- Subsequent reads of different values in the same loop use the cached data
- Cache auto-refreshes when a previously-read value is read again
- Good default — minimal code changes needed
\`\`\`java
hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
\`\`\`

### BulkCachingMode.MANUAL (Best Performance)
- Cache is only refreshed when you explicitly call clearBulkCache()
- You control exactly when the bulk read happens (once per loop)
- Best performance — all reads in one loop use the same consistent snapshot
\`\`\`java
hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
// In loop:
hub.clearBulkCache(); // Triggers fresh bulk read
\`\`\`

### What IS Included in Bulk Reads
- Encoder positions (getCurrentPosition())
- Encoder velocities (getVelocity())
- Digital channel states (getState())
- Analog input values (getVoltage())
- Motor overcurrent flags (isOverCurrent())

### What is NOT Included in Bulk Reads
- I2C sensor reads (IMU, color sensor, distance sensor, Pinpoint, OTOS)
- Motor current draw (getCurrent())
- Servo positions (getPosition() — returns cached target anyway)
- Any I2C device communication

### Setup Code
\`\`\`java
// In init
List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
for (LynxModule hub : allHubs) {
    hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
}

// At the START of every loop iteration
for (LynxModule hub : allHubs) {
    hub.clearBulkCache();
}

// Now all encoder/digital/analog reads use the cached bulk data
int pos0 = motor0.getCurrentPosition(); // From cache — free
int pos1 = motor1.getCurrentPosition(); // From cache — free
int pos2 = motor2.getCurrentPosition(); // From cache — free
int pos3 = motor3.getCurrentPosition(); // From cache — free
// Total: 1 bulk read instead of 4 individual reads
\`\`\`

### Performance Impact
| Scenario                       | OFF      | MANUAL   |
|-------------------------------|----------|----------|
| 4 encoder reads               | 8-12ms   | 2-3ms    |
| 4 encoders + 2 digital + 1 analog | 14-21ms | 2-3ms |
| Loop time (typical drivetrain) | 30-50ms  | 10-15ms  |
`,

  cachingHardware: `
## Dairy Foundation CachingHardware

### Overview
Drop-in wrappers that prevent redundant writes to motors and servos. Instead of writing the same power value every loop iteration, CachingHardware only sends a USB command when the value actually changes.

### Available Wrappers
- \`CachingDcMotorEx\` — wraps DcMotorEx
- \`CachingDcMotor\` — wraps DcMotor
- \`CachingServo\` — wraps Servo
- \`CachingCRServo\` — wraps CRServo

### Instantiation
\`\`\`java
// Drop-in replacement — wrap existing hardware object
CachingDcMotorEx motor = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "motor"));
CachingServo servo = new CachingServo(hardwareMap.get(Servo.class, "servo"));
CachingCRServo crServo = new CachingCRServo(hardwareMap.get(CRServo.class, "crServo"));

// Custom tolerance (optional)
CachingDcMotorEx motor = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "motor"), 0.01);
CachingServo servo = new CachingServo(hardwareMap.get(Servo.class, "servo"), 0.002);
\`\`\`

### Default Tolerances
- Motors: 0.005 (changes smaller than 0.5% are suppressed)
- Servos: 0.001 (changes smaller than 0.1% are suppressed)

### Write Algorithm
A write is sent to the hub only if ANY of these conditions are true:
1. Delta >= tolerance (value changed significantly)
2. Value is crossing zero (sign change — direction reversal)
3. Value is crossing ±1 (max power boundary)
4. First write ever (internal state initializes to NaN)

### Additional API
\`\`\`java
// Check if last setPower() actually sent a write
boolean didWrite = motor.setPowerResult(); // Returns true if a USB write occurred

// Force a write regardless of caching (bypasses tolerance check)
motor.setPowerRaw(0.5);
\`\`\`

### Thread Safety
- All methods are synchronized — safe to use from multiple threads
- (Though multithreading doesn't help with USB latency due to hub master lock)

### Installation (build.gradle)
\`\`\`groovy
// In repositories block
repositories {
    maven { url = "https://repo.dairy.foundation/releases" }
}

// In dependencies block
dependencies {
    implementation "dev.frozenmilk.dairy:CachingHardware:1.0.0"
}
\`\`\`

### Complete Example
\`\`\`java
import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;
import dev.frozenmilk.dairy.cachinghardware.CachingServo;

@TeleOp(name = "Caching Example")
public class CachingExample extends LinearOpMode {
    @Override
    public void runOpMode() {
        CachingDcMotorEx fl = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "frontLeft"));
        CachingDcMotorEx fr = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "frontRight"));
        CachingDcMotorEx bl = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "backLeft"));
        CachingDcMotorEx br = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "backRight"));
        CachingServo claw = new CachingServo(hardwareMap.get(Servo.class, "claw"));

        waitForStart();

        while (opModeIsActive()) {
            double drive = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            fl.setPower(drive + strafe + turn); // Only writes if changed
            fr.setPower(drive - strafe - turn);
            bl.setPower(drive - strafe + turn);
            br.setPower(drive + strafe - turn);

            if (gamepad1.a) claw.setPosition(1.0); // Only writes once
            if (gamepad1.b) claw.setPosition(0.0);
        }
    }
}
\`\`\`
`,

  optimizationSummary: `
## FTC Hardware Optimization — Ranked Strategies

### 1. Bulk Reads MANUAL (Highest Impact)
- Switch to LynxModule.BulkCachingMode.MANUAL
- Clear cache once at the start of each loop
- Saves 2-3ms per sensor read (can save 10-20ms per loop)

### 2. Bulk Reads AUTO (Easy Win)
- If MANUAL is too complex, use AUTO mode
- Still significantly better than OFF
- Minimal code changes required

### 3. CachingHardware (Motor/Servo Writes)
- Use Dairy Foundation CachingHardware wrappers
- Eliminates redundant USB writes (~2-3ms saved per duplicate write)
- Drop-in replacement, no logic changes needed

### 4. Minimize I2C Sensor Reads
- I2C reads are NOT bulk-readable (~7ms each)
- IMU, color sensor, distance sensor, OTOS, Pinpoint are all I2C
- Read I2C sensors only when needed, not every loop

### 5. Control Hub over Phone
- Control Hub has internal USB (faster communication)
- ~2ms per command vs ~5ms+ for phone-based control

### 6. Reduced-Frequency I2C Reads
- Read expensive sensors every Nth loop instead of every loop
\`\`\`java
int loopCount = 0;
// In loop:
loopCount++;
if (loopCount % 5 == 0) {
    // Read IMU only every 5th loop
    heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
}
\`\`\`

### 7. Cache RunMode and ZeroPowerBehavior
- setMode() and setZeroPowerBehavior() send USB commands
- Only call them when the value actually changes
\`\`\`java
// BAD: sets mode every loop
motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // USB write every loop!

// GOOD: track and only set when changing
if (currentMode != DcMotor.RunMode.RUN_WITHOUT_ENCODER) {
    motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    currentMode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
}
\`\`\`

### 8. Use Encoder Ports 0 and 3 for High-CPR Encoders
- Hardware decoded — zero missed counts
- Critical for 8192 CPR through bore encoders

### 9. Avoid Multithreading for Hardware Access
- REV Hub has a master USB lock — all commands serialize
- Multiple threads just add contention overhead
- Use single-threaded loop with optimized read/write patterns

### 10. Distribute Sensors Across Hubs
- If using both Control Hub and Expansion Hub
- Split I2C sensors across both hubs to parallelize
- Each hub's I2C buses are independent
`,

  customWrappers: `
## Custom Hardware Wrapper Patterns

### Kotlin Delegation Pattern
\`\`\`kotlin
class SmartMotor(
    private val motor: DcMotorEx
) : DcMotorEx by motor {
    // Override only what you need
    private var lastPower = Double.NaN
    private var lastMode: DcMotor.RunMode? = null

    override fun setPower(power: Double) {
        if (abs(power - lastPower) > 0.005) {
            motor.setPower(power)
            lastPower = power
        }
    }

    fun setModeCached(mode: DcMotor.RunMode) {
        if (mode != lastMode) {
            motor.mode = mode
            lastMode = mode
        }
    }
}
\`\`\`

### Java Composition Pattern
\`\`\`java
public class SmartMotor {
    private final DcMotorEx motor;
    private double lastPower = Double.NaN;
    private DcMotor.RunMode lastMode = null;
    private DcMotor.ZeroPowerBehavior lastZpb = null;

    public SmartMotor(DcMotorEx motor) {
        this.motor = motor;
    }

    public DcMotorEx getRaw() { return motor; }

    // Cached power write
    public void setPower(double power) {
        if (Math.abs(power - lastPower) > 0.005 || Double.isNaN(lastPower)) {
            motor.setPower(power);
            lastPower = power;
        }
    }

    // Lazy RunMode write
    public void setMode(DcMotor.RunMode mode) {
        if (mode != lastMode) {
            motor.setMode(mode);
            lastMode = mode;
        }
    }

    // Lazy ZeroPowerBehavior write
    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zpb) {
        if (zpb != lastZpb) {
            motor.setZeroPowerBehavior(zpb);
            lastZpb = zpb;
        }
    }

    // Delegate reads directly
    public int getCurrentPosition() { return motor.getCurrentPosition(); }
    public double getVelocity() { return motor.getVelocity(); }
}
\`\`\`

### Timed Sensor Reads (Skip Expensive Reads Some Loops)
\`\`\`java
public class TimedSensor<T> {
    private final Supplier<T> reader;
    private final long intervalMs;
    private final ElapsedTime timer = new ElapsedTime();
    private T cachedValue;

    public TimedSensor(Supplier<T> reader, long intervalMs) {
        this.reader = reader;
        this.intervalMs = intervalMs;
    }

    public T read() {
        if (cachedValue == null || timer.milliseconds() >= intervalMs) {
            cachedValue = reader.get();
            timer.reset();
        }
        return cachedValue;
    }
}

// Usage:
TimedSensor<Double> imuHeading = new TimedSensor<>(
    () -> imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES),
    50 // Read at most every 50ms
);
// In loop:
double heading = imuHeading.read(); // Returns cached value if <50ms since last read
\`\`\`

### ElapsedTime Profiling Pattern
\`\`\`java
ElapsedTime loopTimer = new ElapsedTime();
ElapsedTime segmentTimer = new ElapsedTime();

// In loop:
loopTimer.reset();

segmentTimer.reset();
for (LynxModule hub : hubs) hub.clearBulkCache();
double bulkTime = segmentTimer.milliseconds();

segmentTimer.reset();
// ... motor reads ...
double readTime = segmentTimer.milliseconds();

segmentTimer.reset();
// ... motor writes ...
double writeTime = segmentTimer.milliseconds();

segmentTimer.reset();
// ... I2C reads ...
double i2cTime = segmentTimer.milliseconds();

double totalLoop = loopTimer.milliseconds();
telemetry.addData("Loop", "%.1fms (bulk:%.1f read:%.1f write:%.1f i2c:%.1f)",
    totalLoop, bulkTime, readTime, writeTime, i2cTime);
\`\`\`

### Complete Optimized OpMode Template
\`\`\`java
@TeleOp(name = "Optimized TeleOp")
public class OptimizedTeleOp extends LinearOpMode {

    // Hardware
    private CachingDcMotorEx fl, fr, bl, br;
    private CachingServo claw;
    private GoBildaPinpointDriver odo;
    private List<LynxModule> allHubs;

    // State
    private int loopCount = 0;
    private ElapsedTime loopTimer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        // ===== INIT =====

        // Bulk reads — MANUAL mode for best performance
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        // Motors — CachingHardware wrappers
        fl = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "frontLeft"));
        fr = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "frontRight"));
        bl = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "backLeft"));
        br = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "backRight"));

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Servo — CachingHardware wrapper
        claw = new CachingServo(hardwareMap.get(Servo.class, "claw"));

        // Odometry
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        odo.setOffsets(-84.0, 168.0, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(
            GoBildaPinpointDriver.EncoderDirection.FORWARD,
            GoBildaPinpointDriver.EncoderDirection.REVERSED
        );
        odo.resetPosAndIMU();
        Thread.sleep(300);

        waitForStart();

        // ===== LOOP =====
        while (opModeIsActive()) {
            loopTimer.reset();
            loopCount++;

            // 1. Clear bulk cache (one bulk read per hub)
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }

            // 2. Read odometry (I2C — update every loop for best tracking)
            odo.update();

            // 3. Process inputs
            double drive = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            // 4. Mecanum math
            double flPower = drive + strafe + turn;
            double frPower = drive - strafe - turn;
            double blPower = drive - strafe + turn;
            double brPower = drive + strafe - turn;

            // Normalize
            double max = Math.max(Math.abs(flPower),
                         Math.max(Math.abs(frPower),
                         Math.max(Math.abs(blPower), Math.abs(brPower))));
            if (max > 1.0) {
                flPower /= max;
                frPower /= max;
                blPower /= max;
                brPower /= max;
            }

            // 5. Write motors (CachingHardware suppresses redundant writes)
            fl.setPower(flPower);
            fr.setPower(frPower);
            bl.setPower(blPower);
            br.setPower(brPower);

            // 6. Servo control
            if (gamepad1.a) claw.setPosition(1.0);
            if (gamepad1.b) claw.setPosition(0.0);

            // 7. Telemetry (every 10th loop to reduce overhead)
            if (loopCount % 10 == 0) {
                Pose2D pose = odo.getPosition();
                telemetry.addData("Pose", "X:%.1f Y:%.1f H:%.1f",
                    pose.getX(DistanceUnit.INCH),
                    pose.getY(DistanceUnit.INCH),
                    pose.getHeading(AngleUnit.DEGREES));
                telemetry.addData("Loop (ms)", loopTimer.milliseconds());
                telemetry.update();
            }
        }
    }
}
\`\`\`
`,

  vision: `
## Vision — VisionPortal, AprilTag, and Limelight

### VisionPortal Setup
\`\`\`java
// Create an AprilTag processor
AprilTagProcessor aprilTag = new AprilTagProcessor.Builder()
    .setDrawAxes(true)
    .setDrawCubeProjection(false)
    .setDrawTagOutline(true)
    .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
    .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
    .build();

// Build the VisionPortal
VisionPortal portal = new VisionPortal.Builder()
    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
    .addProcessor(aprilTag)
    .setCameraResolution(new Size(640, 480))
    .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
    .build();
\`\`\`

### AprilTag Detection
\`\`\`java
// In loop:
List<AprilTagDetection> detections = aprilTag.getDetections();

for (AprilTagDetection detection : detections) {
    int id = detection.id; // Tag ID number

    if (detection.metadata != null) {
        // Tag is in the library — full pose available
        // ftcPose is camera-relative:
        double x = detection.ftcPose.x;         // Right/left offset (inches)
        double y = detection.ftcPose.y;         // Forward distance (inches)
        double z = detection.ftcPose.z;         // Vertical offset (inches)
        double pitch = detection.ftcPose.pitch; // Rotation about X axis (degrees)
        double roll = detection.ftcPose.roll;   // Rotation about Y axis (degrees)
        double yaw = detection.ftcPose.yaw;     // Rotation about Z axis (degrees)
        double range = detection.ftcPose.range;     // Direct distance to tag (inches)
        double bearing = detection.ftcPose.bearing; // Horizontal angle to tag (degrees)
        double elevation = detection.ftcPose.elevation; // Vertical angle (degrees)

        telemetry.addData("Tag " + id, "Range:%.1f Bearing:%.1f", range, bearing);
    }
}
\`\`\`

### Camera Controls
\`\`\`java
// Exposure control (reduces motion blur)
ExposureControl exposure = portal.getCameraControl(ExposureControl.class);
exposure.setMode(ExposureControl.Mode.Manual);
exposure.setExposure(6, TimeUnit.MILLISECONDS); // Lower = less blur, darker

// Gain control (brightness compensation for low exposure)
GainControl gain = portal.getCameraControl(GainControl.class);
gain.setGain(250); // Higher = brighter but more noise
\`\`\`

### Limelight 3A
\`\`\`java
Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");

// Switch pipeline (configured in Limelight web interface)
limelight.pipelineSwitch(0);  // Pipeline index 0-9

// Start the camera
limelight.start();

// Set robot orientation for MegaTag2 (field-relative localization)
limelight.updateRobotOrientation(currentHeadingDegrees);

// Get results
LLResult result = limelight.getLatestResult();
if (result != null && result.isValid()) {
    // AprilTag / fiducial results
    List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
    for (LLResultTypes.FiducialResult fr : fiducials) {
        int tagId = fr.getFiducialId();
        double tx = fr.getTargetXDegrees();  // Horizontal offset
        double ty = fr.getTargetYDegrees();  // Vertical offset
        double ta = fr.getTargetArea();       // Target area (0-100%)
    }

    // MegaTag2 robot pose (field-relative)
    Pose3D botPose = result.getBotpose_MT2();
    if (botPose != null) {
        double x = botPose.getPosition().x;  // Field X
        double y = botPose.getPosition().y;  // Field Y
        double heading = botPose.getOrientation().getYaw(); // Field heading
    }

    // Pipeline latency
    double latency = result.getPipelineLatency();
}
\`\`\`

### VisionPortal Management
\`\`\`java
// Pause/resume processing to save CPU
portal.setProcessorEnabled(aprilTag, false); // Disable
portal.setProcessorEnabled(aprilTag, true);  // Enable

// Close when done
portal.close();
\`\`\`

### Complete AprilTag Example
\`\`\`java
@TeleOp(name = "AprilTag Drive")
public class AprilTagDrive extends LinearOpMode {
    @Override
    public void runOpMode() {
        AprilTagProcessor aprilTag = new AprilTagProcessor.Builder()
            .setDrawAxes(true)
            .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
            .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
            .build();

        VisionPortal portal = new VisionPortal.Builder()
            .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
            .addProcessor(aprilTag)
            .setCameraResolution(new Size(640, 480))
            .build();

        // Set low exposure for less motion blur
        while (!isStopRequested() && portal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            sleep(20);
        }
        ExposureControl exposure = portal.getCameraControl(ExposureControl.class);
        exposure.setMode(ExposureControl.Mode.Manual);
        exposure.setExposure(6, TimeUnit.MILLISECONDS);
        GainControl gain = portal.getCameraControl(GainControl.class);
        gain.setGain(250);

        waitForStart();

        while (opModeIsActive()) {
            List<AprilTagDetection> detections = aprilTag.getDetections();
            for (AprilTagDetection det : detections) {
                if (det.metadata != null) {
                    telemetry.addData("Tag " + det.id,
                        "Range:%.1f  Bearing:%.1f  Yaw:%.1f",
                        det.ftcPose.range, det.ftcPose.bearing, det.ftcPose.yaw);
                }
            }
            telemetry.update();
        }

        portal.close();
    }
}
\`\`\`
`,

  commandPipeline: `
## The LynxCommand Pipeline — How Hardware Communication Actually Works

Understanding the USB command pipeline is critical for writing fast FTC code. Every
millisecond matters when your loop needs to run at 100+ Hz.

### How Every Hardware Call Becomes a USB Command

When you call \`motor.setPower(0.5)\`, the SDK does NOT directly talk to the motor.
Instead, it creates a **LynxCommand** — a serialized packet sent over USB (Expansion Hub)
or UART (Control Hub) to the REV Hub's ARM Cortex M4 microprocessor.

\`\`\`
Your Java Code         SDK Internal           REV Hub Firmware
─────────────        ─────────────────       ─────────────────
motor.setPower(0.5) → LynxSetMotorPowerCmd → UART/USB → ARM M4 → H-Bridge PWM update
motor.getPosition() → LynxGetBulkInputData → UART/USB → ARM M4 → Encoder register read
servo.setPosition() → LynxSetServoPosition → UART/USB → ARM M4 → Servo PWM update
\`\`\`

### Command Timing — The Fundamental Constraint

Every LynxCommand is a **blocking round-trip**:
1. SDK sends the command packet over UART/USB
2. Hub firmware receives, processes, and sends a response
3. SDK reads the response and returns to your code

| Connection Type | Latency per Command | Notes |
|---|---|---|
| Control Hub (UART) | ~2 ms | Direct internal UART to Lynx board |
| Expansion Hub (USB) | ~3 ms | USB → FTDI → UART → Lynx board |
| Expansion Hub (RS-485) | ~3-4 ms | Goes through parent hub first, then RS-485 |

### The Master Lock — Why Multithreading Doesn't Help

The SDK holds a **mutex lock per USB device** (per physical hub connection). This means:

\`\`\`
Thread A: motor1.setPower(0.5)  ─── acquires lock ─── sends cmd ─── waits ─── releases lock
Thread B: motor2.setPower(0.8)  ─── BLOCKED waiting ──────────── acquires lock ─── sends ─── ...
\`\`\`

**All commands to the same hub are serialized**, regardless of how many threads you use.
Multithreading hardware calls is AT BEST useless and typically HARMFUL (adds thread
scheduling overhead on top of the serial USB bottleneck).

**Exception:** If you have TWO hubs (Control Hub + Expansion Hub) on separate USB
connections, commands to different hubs CAN execute in parallel because they have
separate locks. But this is an unusual optimization and adds significant complexity.

### Command Count = Loop Time

Your loop time is approximately:

\`\`\`
loop_time ≈ (num_commands × latency_per_command) + compute_time

Example — Unoptimized mecanum TeleOp on Control Hub:
  4 × setPower()           = 4 × 2ms = 8ms   (motor writes)
  4 × getCurrentPosition() = 4 × 2ms = 8ms   (encoder reads — WITHOUT bulk read)
  1 × setPosition()        = 1 × 2ms = 2ms   (servo write)
  1 × IMU read             = 1 × 7ms = 7ms   (I2C — not bulk-readable)
  ─────────────────────────────────────────
  Total USB time:                     ≈ 25ms  (40 Hz)

Example — Optimized with bulk reads + caching:
  1 × clearBulkCache()     = 1 × 2ms = 2ms   (bulk read — all encoders at once)
  2 × setPower() (changed) = 2 × 2ms = 4ms   (only 2 of 4 motors changed power)
  0 × setPosition()        = 0ms              (servo value unchanged — cached)
  1 × IMU read             = 1 × 7ms = 7ms   (I2C — still slow)
  ─────────────────────────────────────────
  Total USB time:                     ≈ 13ms  (77 Hz)
\`\`\`

### What Bulk Read Actually Does Internally

A bulk read is a single LynxCommand (\`LynxGetBulkInputDataCommand\`) that returns:
- All 4 encoder positions
- All 4 encoder velocities
- All 8 digital input states
- All 4 analog input values
- All 4 motor overcurrent flags

**One USB round-trip (~2ms) replaces up to 20 individual reads.**

### What Bulk Read Does NOT Include
- I2C sensor data (IMU, color sensor, distance sensor) — always separate commands
- Motor current draw (\`getCurrent()\`) — separate command
- Servo position readback — no readback exists; servos are write-only
- Any device on a different hub — each hub needs its own bulk read

### I2C Commands Are Multi-Step

Reading an I2C sensor (IMU, color, distance, Pinpoint, OTOS) requires multiple
LynxCommands internally:
1. \`LynxI2cWriteSingleByteCommand\` — set register pointer
2. \`LynxI2cReadStatusQueryCommand\` — poll for completion (may repeat)
3. \`LynxI2cReadMultipleBytesCommand\` — read the result

This is why a single I2C read takes ~7ms on USB (3+ LynxCommands) but only
~3ms on the Control Hub's faster UART + higher polling rate (SDK 5.5+).

**I2C buses are independent.** Each hub has 4 I2C buses. Sensors on different
buses can be read without additional contention (though they still share the
USB master lock). Distribute sensors across buses to minimize blocking.
`,

  writeOptimization: `
## Hardware Write Optimization — Beyond Caching

### The Write Problem

There is **no bulk write** in the FTC SDK. Unlike bulk reads (which batch all sensor
reads into one USB command), each motor and servo write is a separate LynxCommand:

\`\`\`java
// This is 4 separate USB commands = 4 × 2ms = 8ms
fl.setPower(0.5);  // LynxSetMotorConstantPowerCommand(port=0, power=0.5)
fr.setPower(0.5);  // LynxSetMotorConstantPowerCommand(port=1, power=0.5)
bl.setPower(0.5);  // LynxSetMotorConstantPowerCommand(port=2, power=0.5)
br.setPower(0.5);  // LynxSetMotorConstantPowerCommand(port=3, power=0.5)
\`\`\`

Since you can't batch writes, the optimization strategy is to **minimize the
number of writes that actually get sent**.

### Strategy 1: CachingHardware (Write Suppression)

The Dairy Foundation's CachingHardware library wraps motors and servos to skip
writes when the value hasn't changed significantly.

\`\`\`java
// Without caching: 4 setPower() calls = 4 USB commands every loop
// With caching: only sends commands when power actually changes

CachingDcMotorEx fl = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "fl"));
// fl.setPower(0.5) → sends USB command
// fl.setPower(0.5) → SKIPPED (same value)
// fl.setPower(0.51) → SKIPPED (within 0.005 tolerance)
// fl.setPower(0.7) → sends USB command (changed enough)
\`\`\`

**Impact:** In steady-state driving (cruising at constant speed), this can eliminate
3-4 motor writes per loop, saving 6-8ms.

### Strategy 2: Hub-Aware Write Grouping

If you use both a Control Hub and an Expansion Hub, group your writes by hub.
Commands to different hubs are on different USB buses and can technically be
issued without waiting for the other hub's response.

\`\`\`
// GOOD: Group by hub — minimizes context switching
controlHubMotor1.setPower(p1);   // Control Hub
controlHubMotor2.setPower(p2);   // Control Hub
expansionHubMotor1.setPower(p3); // Expansion Hub
expansionHubMotor2.setPower(p4); // Expansion Hub

// ALSO FINE: Order doesn't matter much with the master lock
// The SDK serializes everything regardless
// But grouping helps with mental model and debugging
\`\`\`

**Practical tip:** Put all 4 drive motors on the SAME hub (Control Hub preferred
for lower latency). This ensures drive commands share one bulk read and all
drivetrain-related USB traffic goes through the faster UART path.

### Strategy 3: Odd/Even Frame Splitting

For non-critical hardware that doesn't need updating every single loop, alternate
which devices get written on odd vs even frames.

\`\`\`java
private int loopCount = 0;

@Override
public void loop() {
    clearBulkCache();
    loopCount++;

    // EVERY frame: drivetrain (critical — always update)
    fl.setPower(flPower);
    fr.setPower(frPower);
    bl.setPower(blPower);
    br.setPower(brPower);

    // ODD frames only: lift motor
    if (loopCount % 2 == 1) {
        lift.setPower(liftPower);
    }

    // EVEN frames only: intake + servo
    if (loopCount % 2 == 0) {
        intake.setPower(intakePower);
        claw.setPosition(clawPos);
    }

    // Every 5th frame: LED indicator, telemetry
    if (loopCount % 5 == 0) {
        telemetry.update();
    }
}
\`\`\`

**Impact:** Splitting 2 non-critical writes to alternate frames saves ~4ms every
frame. The mechanism still updates at 50Hz+ which is more than adequate for
lifts, intakes, and servos.

### Strategy 4: I2C Read Throttling

I2C reads (IMU, color, distance sensors) are the most expensive operations.
Throttle them to only read when the data is actually needed.

\`\`\`java
// BAD: Read IMU every loop (7ms wasted most loops)
double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

// GOOD: Read IMU only when heading is needed for field-centric calc
private double cachedHeading = 0;
private long lastImuReadMs = 0;
private static final long IMU_READ_INTERVAL_MS = 50; // 20 Hz is plenty

private double getHeading() {
    long now = System.currentTimeMillis();
    if (now - lastImuReadMs >= IMU_READ_INTERVAL_MS) {
        cachedHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        lastImuReadMs = now;
    }
    return cachedHeading;
}
\`\`\`

**Impact:** Reading the IMU at 20Hz instead of 100Hz saves ~5.6ms per loop
(7ms × 0.8 loops where the read is skipped).

**Better alternative:** Use a GoBilda Pinpoint or SparkFun OTOS for odometry.
These devices give you heading + position via I2C but with much less
per-read overhead because they do sensor fusion onboard.

### Strategy 5: Eliminate RunMode and ZeroPowerBehavior Redundancy

The SDK sends a LynxCommand when you call \`setMode()\` or \`setZeroPowerBehavior()\`
even if the value hasn't changed. Cache these yourself:

\`\`\`java
private DcMotor.RunMode currentMode = null;

public void setModeCached(DcMotor.RunMode mode) {
    if (mode != currentMode) {
        motor.setMode(mode);
        currentMode = mode;
    }
    // If mode is already set, this is FREE (no USB command)
}
\`\`\`

### Strategy 6: Telemetry Throttling

\`telemetry.update()\` sends data over WiFi to the Driver Station. While not a USB
command, it consumes CPU time and can add 1-5ms per call.

\`\`\`java
// Throttle to every 5th or 10th loop
if (loopCount % 10 == 0) {
    telemetry.addData("Loop (ms)", loopTimer.milliseconds());
    telemetry.update();
}
\`\`\`

### Summary: Write Optimization Checklist

| Strategy | Saves | Difficulty |
|---|---|---|
| CachingHardware wrappers | 4-8ms/loop | Easy (drop-in) |
| I2C read throttling | 5-7ms/loop | Easy |
| Odd/even frame splitting | 2-6ms/loop | Medium |
| RunMode/ZPB caching | 0-4ms/loop | Easy |
| Telemetry throttling | 1-5ms/loop | Trivial |
| Hub-aware device placement | 2-4ms/loop | Planning |

**Combined impact:** An unoptimized loop at 30-50ms can typically be reduced
to 5-12ms (80-200 Hz) using these strategies together.
`,

  loopTimeBudget: `
## Loop Time Budget — Where Every Millisecond Goes

### Anatomy of a Single Loop Iteration

\`\`\`
┌─────────────────────────────────────────────────────────────────┐
│                     ONE LOOP ITERATION                          │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  1. clearBulkCache()         │ ~2ms  │ One bulk read from hub   │
│  2. Read gamepad inputs      │ ~0ms  │ Local memory (cached)    │
│  3. Compute drive powers     │ ~0ms  │ CPU math (negligible)    │
│  4. setPower() × 4 motors    │ 4-8ms │ 4 USB writes (or fewer  │
│     (with CachingHardware)   │ 0-4ms │  if values unchanged)   │
│  5. Read encoders (cached)   │ ~0ms  │ Already in bulk cache    │
│  6. Read IMU (I2C)           │ ~7ms  │ EXPENSIVE — throttle!    │
│  7. Servo writes             │ 0-2ms │ Usually cached (0ms)     │
│  8. PID computation          │ ~0ms  │ CPU math (negligible)    │
│  9. Telemetry update         │ 1-5ms │ WiFi to Driver Station   │
│                                                                 │
│  TOTAL (unoptimized):        │ ~25ms │ = 40 Hz loop             │
│  TOTAL (optimized):          │ ~8ms  │ = 125 Hz loop            │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
\`\`\`

### Detailed Timing Table

| Operation | Control Hub | Expansion Hub | Bulk Readable? |
|---|---|---|---|
| Bulk read (clearBulkCache) | 2 ms | 3 ms | N/A (IS bulk read) |
| Motor setPower() | 2 ms | 3 ms | N/A (write) |
| Motor setMode() | 2 ms | 3 ms | N/A (write) |
| Servo setPosition() | 2 ms | 3 ms | N/A (write) |
| Encoder getPosition() | 0 ms* | 0 ms* | YES — in bulk cache |
| Encoder getVelocity() | 0 ms* | 0 ms* | YES — in bulk cache |
| Digital getState() | 0 ms* | 0 ms* | YES — in bulk cache |
| Analog getVoltage() | 0 ms* | 0 ms* | YES — in bulk cache |
| isOverCurrent() | 0 ms* | 0 ms* | YES — in bulk cache |
| IMU getYaw() | 3 ms | 7 ms | NO — I2C |
| Color sensor read | 3 ms | 7 ms | NO — I2C |
| Distance sensor read | 3 ms | 7 ms | NO — I2C |
| Pinpoint getPosition() | 3 ms | 7 ms | NO — I2C |
| OTOS getPosition() | 3 ms | 7 ms | NO — I2C |
| Motor getCurrent() | 2 ms | 3 ms | NO — separate command |
| telemetry.update() | 1-5 ms | 1-5 ms | N/A (WiFi) |

*After clearBulkCache() in MANUAL mode — reads from local cache (0ms).

### Optimization Priority (Impact Ranking)

1. **Bulk reads (MANUAL mode)** — saves 8-16ms by replacing N individual sensor
   reads with 1 bulk command. This is the single highest-impact optimization.

2. **I2C read throttling** — saves 5-7ms per avoided read. IMU/color/distance
   sensors at 20Hz instead of 100Hz is usually sufficient.

3. **CachingHardware wrappers** — saves 2-8ms by skipping redundant motor/servo
   writes. Most impactful during steady-state (constant speed) operation.

4. **Odd/even frame splitting** — saves 2-6ms by deferring non-critical mechanism
   updates to alternating frames. Mechanisms still update at 50+ Hz.

5. **Telemetry throttling** — saves 1-5ms. Update telemetry every 5-10 loops.

6. **RunMode/ZPB caching** — saves 0-4ms. Avoid redundant setMode() calls.

### Target Performance

| Metric | Unoptimized | Acceptable | Good | Excellent |
|---|---|---|---|---|
| Loop time | 50-100 ms | 15-25 ms | 8-15 ms | < 8 ms |
| Loop rate | 10-20 Hz | 40-67 Hz | 67-125 Hz | 125+ Hz |

**Why loop speed matters:**
- **Path following accuracy**: Pedro Pathing and Road Runner update their motion
  controllers every loop. Faster loops = smoother, more accurate paths.
- **PID responsiveness**: Faster PID loops = less overshoot, better disturbance rejection.
- **Gamepad responsiveness**: Faster loops = less input lag, better driver feel.

### Complete Optimized Loop Template

\`\`\`java
@Config
@TeleOp(name = "Optimized TeleOp")
public class OptimizedTeleOp extends OpMode {

    // Dashboard-tunable
    public static double SLOW_SPEED = 0.35;
    public static int TELEMETRY_INTERVAL = 10;
    public static long IMU_INTERVAL_MS = 50;

    private List<LynxModule> allHubs;
    private CachingDcMotorEx fl, fr, bl, br;
    private CachingServo claw;
    private IMU imu;

    private double cachedHeading = 0;
    private long lastImuRead = 0;
    private int loopCount = 0;
    private ElapsedTime loopTimer = new ElapsedTime();

    @Override
    public void init() {
        // 1. Enable MANUAL bulk reads
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        // 2. Use CachingHardware wrappers
        fl = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "fl"));
        fr = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "fr"));
        bl = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "bl"));
        br = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "br"));
        claw = new CachingServo(hardwareMap.get(Servo.class, "claw"));

        // 3. IMU setup
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.UP,
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
        imu.resetYaw();

        telemetry = new MultipleTelemetry(telemetry,
            FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        loopTimer.reset();
        loopCount++;

        // === PHASE 1: Bulk read (one USB command for all sensors) ===
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

        // === PHASE 2: Reads (from bulk cache = free) ===
        // Encoder positions, digital inputs, analog — all cached
        int flPos = fl.getCurrentPosition(); // 0ms — from cache
        int frPos = fr.getCurrentPosition(); // 0ms — from cache

        // IMU — throttled (only read every 50ms)
        long now = System.currentTimeMillis();
        if (now - lastImuRead >= IMU_INTERVAL_MS) {
            cachedHeading = imu.getRobotYawPitchRollAngles()
                .getYaw(AngleUnit.RADIANS);
            lastImuRead = now;
        }

        // === PHASE 3: Compute (CPU only — ~0ms) ===
        double y  = -gamepad1.left_stick_y;
        double x  =  gamepad1.left_stick_x * 1.1;
        double rx =  gamepad1.right_stick_x;

        // Field-centric rotation
        double rotX = x * Math.cos(-cachedHeading) - y * Math.sin(-cachedHeading);
        double rotY = x * Math.sin(-cachedHeading) + y * Math.cos(-cachedHeading);

        double denom = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double flP = (rotY + rotX + rx) / denom;
        double frP = (rotY - rotX - rx) / denom;
        double blP = (rotY - rotX + rx) / denom;
        double brP = (rotY + rotX - rx) / denom;

        // === PHASE 4: Writes (CachingHardware skips unchanged values) ===
        fl.setPower(flP); // Only sends USB command if value changed
        fr.setPower(frP);
        bl.setPower(blP);
        br.setPower(brP);

        // Servo — CachingServo skips if position unchanged
        if (gamepad1.a) claw.setPosition(0.6);
        if (gamepad1.b) claw.setPosition(0.0);

        // === PHASE 5: Telemetry (throttled) ===
        if (loopCount % TELEMETRY_INTERVAL == 0) {
            telemetry.addData("Loop (ms)", "%.1f", loopTimer.milliseconds());
            telemetry.addData("Heading", "%.1f", Math.toDegrees(cachedHeading));
            telemetry.addData("FL/FR", "%.2f / %.2f", flP, frP);
            telemetry.update();
        }
    }
}
\`\`\`

### Phase-Ordered Loop Pattern

For maximum performance, order your loop operations in phases:

1. **Bulk cache clear** — one USB command, gets fresh sensor data
2. **Reads** — all from cache (0ms each), plus throttled I2C reads
3. **Compute** — pure CPU math, negligible time
4. **Writes** — motor/servo commands, minimized by CachingHardware
5. **Telemetry** — throttled to every Nth loop

This ordering ensures reads get fresh data, writes happen after all
computations are complete, and telemetry doesn't block critical path.
`,
};
