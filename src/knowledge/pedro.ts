export const PEDRO_KNOWLEDGE = {
  apiReference: `# Pedro Pathing 2.0 - Complete API Reference

## Imports

\`\`\`java
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.Point;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.util.Timer;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.ftc.localization.constants.OTOSConstants;
\`\`\`

## Pose

The \`Pose\` class represents a robot position and heading on the field.

### Constructor
\`\`\`java
// Pose(x, y, headingInRadians)
Pose startPose = new Pose(9, 111, Math.toRadians(270));
Pose scorePose = new Pose(14, 129, Math.toRadians(315));
\`\`\`

- \`x\` — X coordinate in inches (range 0–144)
- \`y\` — Y coordinate in inches (range 0–144)
- \`heading\` — Heading in **radians**. Use \`Math.toRadians(degrees)\` for conversion.

### Methods
- \`getX()\` — Returns the x coordinate.
- \`getY()\` — Returns the y coordinate.
- \`getHeading()\` — Returns the heading in radians.

## Point

The \`Point\` class represents a 2D coordinate used for path control points.

### Constructors
\`\`\`java
// From a Pose (extracts x, y)
Point p1 = new Point(new Pose(9, 111, 0));

// From raw coordinates
Point p2 = new Point(36.0, 72.0, Point.CARTESIAN);
\`\`\`

## BezierLine

A straight-line path segment between two points.

### Constructor
\`\`\`java
BezierLine line = new BezierLine(
    new Point(startPose),   // start point
    new Point(endPose)      // end point
);
\`\`\`

## BezierCurve

A curved path segment using Bezier control points. Minimum 3 points (start, one control, end). More control points create more complex curves.

### Constructor
\`\`\`java
// Quadratic Bezier (3 points: start, control, end)
BezierCurve curve = new BezierCurve(
    new Point(startPose),
    new Point(36.0, 100.0, Point.CARTESIAN),  // control point
    new Point(endPose)
);

// Cubic Bezier (4 points: start, control1, control2, end)
BezierCurve curve = new BezierCurve(
    new Point(startPose),
    new Point(30.0, 100.0, Point.CARTESIAN),  // control point 1
    new Point(50.0, 120.0, Point.CARTESIAN),  // control point 2
    new Point(endPose)
);
\`\`\`

## PathBuilder

The \`PathBuilder\` is obtained from \`follower.pathBuilder()\` and is used to construct \`PathChain\` objects.

### Methods

- **\`addPath(BezierLine)\`** — Adds a straight-line path segment.
- **\`addPath(BezierCurve)\`** — Adds a curved path segment.
- **\`setLinearHeadingInterpolation(double startRad, double endRad)\`** — Linearly interpolates heading from \`startRad\` to \`endRad\` over the path segment. Commonly used for turning while driving.
- **\`setConstantHeadingInterpolation(double headingRad)\`** — Maintains a constant heading throughout the path segment.
- **\`setTangentHeadingInterpolation()\`** — Sets heading to follow the tangent direction of the path (robot faces forward along the curve).
- **\`setPathEndTimeoutConstraint(double seconds)\`** — Maximum time the follower will spend trying to reach the end of this path before moving on. Default is usually fine; lower for faster transitions.
- **\`setPathEndTValueConstraint(double tValue)\`** — The parametric t-value (0.0–1.0) at which the path is considered "close enough" to the end. Higher values (e.g., 0.99) require more precision. Lower values (e.g., 0.9) allow early termination.
- **\`setPathEndVelocityConstraint(double velocity)\`** — Maximum velocity (in/sec) at the end of the path for it to be considered complete. Lower values require the robot to slow down more.
- **\`setZeroPowerAccelerationMultiplier(double mult)\`** — Controls how aggressively the robot decelerates at the end of this path. Higher = more aggressive braking (range typically 1.0–4.0).
- **\`addParametricCallback(double t, Runnable callback)\`** — Fires the callback when the parametric progress along the path reaches \`t\` (0.0–1.0). Recommended over temporal callbacks.
- **\`addTemporalCallback(double seconds, Runnable callback)\`** — Fires the callback after \`seconds\` have elapsed since the start of this path segment. NOT recommended because timing varies with battery voltage and friction.
- **\`build()\`** — Builds and returns the completed \`PathChain\`.

### Example
\`\`\`java
PathChain scorePreload = follower.pathBuilder()
    .addPath(new BezierLine(new Point(startPose), new Point(scorePose)))
    .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
    .setPathEndTimeoutConstraint(3.0)
    .build();

PathChain pickupAndScore = follower.pathBuilder()
    .addPath(new BezierCurve(
        new Point(scorePose),
        new Point(30, 110, Point.CARTESIAN),
        new Point(pickupPose)
    ))
    .setLinearHeadingInterpolation(scorePose.getHeading(), pickupPose.getHeading())
    .addParametricCallback(0.5, () -> {
        // Lower arm halfway through path
    })
    .addPath(new BezierLine(new Point(pickupPose), new Point(scorePose)))
    .setLinearHeadingInterpolation(pickupPose.getHeading(), scorePose.getHeading())
    .build();
\`\`\`

## Follower

The \`Follower\` is the main class that controls path following and robot movement.

### Construction (v2.0 Builder Pattern)
\`\`\`java
Follower follower = new FollowerBuilder(followerConstants, hardwareMap)
    .mecanumDrivetrain(mecanumConstants)
    .pinpointLocalizer(pinpointConstants)
    .pathConstraints(pathConstraints)
    .build();
\`\`\`

### Path Following Methods

- **\`followPath(PathChain pathChain)\`** — Begins following the given PathChain. The robot will NOT hold its position at the end.
- **\`followPath(PathChain pathChain, boolean holdEnd)\`** — Begins following the given PathChain. If \`holdEnd\` is \`true\`, the robot actively holds position at the final Pose after finishing.
- **\`update()\`** — **MUST be called every loop iteration.** Updates the follower's state, motor powers, and localization. Without this, the robot will not move.
- **\`isBusy()\`** — Returns \`true\` if the follower is currently following a path. Returns \`false\` when the path is complete (or not started). Used as the primary state transition condition in autonomous.
- **\`breakFollowing()\`** — Immediately stops the current path following. Robot will coast to a stop.
- **\`holdPoint(Pose pose)\`** — Actively holds the robot at the given Pose using PID. Useful for maintaining position while performing other actions.

### Pose / Localization Methods

- **\`getPose()\`** — Returns the current estimated \`Pose\` of the robot on the field.
- **\`setPose(Pose pose)\`** — Overrides the current pose estimate. Used to set the starting position.
- **\`setStartingPose(Pose pose)\`** — Sets the starting pose of the robot. Call this in \`init\` before path following begins.
- **\`setMaxPower(double power)\`** — Sets the maximum motor power (0.0–1.0). Useful for slow/precise movements.

### Path Progress Methods

- **\`getCurrentTValue()\`** — Returns the current parametric t-value (0.0–1.0) along the current path segment. Useful for parametric-based state transitions.
- **\`getCurrentPathNumber()\`** — Returns the index of the current path segment within the PathChain (0-indexed).
- **\`getVelocity()\`** — Returns the current velocity as a vector.
- **\`getVelocityMagnitude()\`** — Returns the scalar speed of the robot in inches per second.

### Pause / Resume

- **\`pausePathFollowing()\`** — Pauses path following. The robot will hold its current position. Useful for mid-path actions (e.g., picking up a sample).
- **\`resumePathFollowing()\`** — Resumes path following from where it was paused.

### TeleOp Methods

- **\`startTeleopDrive()\`** — Initializes the follower for TeleOp control. Call once in \`start()\` or \`init()\`.
- **\`setTeleOpDrive(double forward, double strafe, double turn, boolean robotCentric)\`** — Sets TeleOp drive powers. If \`robotCentric\` is \`true\`, inputs are relative to the robot. If \`false\`, inputs are field-centric.

### Example TeleOp Drive
\`\`\`java
follower.setTeleOpDrive(
    -gamepad1.left_stick_y,   // forward (negated because stick y is inverted)
    -gamepad1.left_stick_x,   // strafe
    -gamepad1.right_stick_x,  // turn
    true                       // robot centric
);
\`\`\`

## Timer

The \`Timer\` utility class is used for time-based state transitions in autonomous.

### Constructor
\`\`\`java
Timer pathTimer = new Timer();
\`\`\`

### Methods
- **\`resetTimer()\`** — Resets the timer to zero.
- **\`getElapsedTimeSeconds()\`** — Returns the elapsed time in seconds since the last reset.
- **\`getElapsedTime()\`** — Returns the elapsed time in milliseconds.
\`\`\`java
if (pathTimer.getElapsedTimeSeconds() > 2.0) {
    // 2 seconds have elapsed
}
\`\`\`
`,

  constantsPattern: `# Pedro Pathing 2.0 - Constants Builder Pattern

Pedro 2.0 uses a builder pattern for all configuration constants. All constants classes are configured in a single \`Constants.java\` file.

## FollowerConstants

Controls PID tuning, path end constraints, and general follower behavior.

\`\`\`java
FollowerConstants followerConstants = new FollowerConstants.Builder()
    .mass(12.5)                                          // Robot mass in kg
    .translationalPIDFCoefficients(0.1, 0, 0.01, 0)     // Translational PIDF (p, i, d, f)
    .headingPIDFCoefficients(2.0, 0, 0.1, 0)            // Heading PIDF (p, i, d, f)
    .drivePIDFCoefficients(0.01, 0, 0.0001, 0.6)        // Drive (forward) PIDF (p, i, d, f)
    .zeroPowerAccelerationMultiplier(4.0)                // Braking aggressiveness (1.0–8.0)
    .translationalPIDFFeedForward(0.015)                 // Translational feed forward
    .headingPIDFFeedForward(0.01)                        // Heading feed forward
    .centripetalScaling(0.0005)                          // Centripetal force correction
    .pathEndTimeoutConstraint(3.0)                       // Max seconds to reach path end
    .pathEndTValueConstraint(0.995)                      // t-value to consider path done
    .pathEndVelocityConstraint(0.1)                      // Max velocity (in/s) at path end
    .build();
\`\`\`

### FollowerConstants Parameters Explained

| Parameter | Description | Typical Range |
|-----------|-------------|---------------|
| \`mass\` | Robot mass in kilograms | 8.0–15.0 |
| \`translationalPIDFCoefficients\` | Controls lateral/translational correction | P: 0.05–0.3, D: 0.005–0.05 |
| \`headingPIDFCoefficients\` | Controls heading (rotation) correction | P: 1.0–4.0, D: 0.05–0.2 |
| \`drivePIDFCoefficients\` | Controls forward drive correction | P: 0.005–0.05, F: 0.5–0.8 |
| \`zeroPowerAccelerationMultiplier\` | Braking aggressiveness | 2.0–6.0 |
| \`translationalPIDFFeedForward\` | Steady-state translational correction | 0.005–0.03 |
| \`headingPIDFFeedForward\` | Steady-state heading correction | 0.005–0.02 |
| \`centripetalScaling\` | Centripetal force for curves | 0.0001–0.001 |
| \`pathEndTimeoutConstraint\` | Seconds before giving up on path end | 1.0–5.0 |
| \`pathEndTValueConstraint\` | Parametric completeness threshold | 0.95–0.999 |
| \`pathEndVelocityConstraint\` | Velocity threshold for path end | 0.05–0.5 |

## MecanumConstants

Configures the mecanum drivetrain motor names, directions, and movement scalars.

\`\`\`java
MecanumConstants mecanumConstants = new MecanumConstants.Builder()
    .leftFrontMotorName("leftFront")
    .leftRearMotorName("leftRear")
    .rightFrontMotorName("rightFront")
    .rightRearMotorName("rightRear")
    .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
    .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
    .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
    .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
    .xMovement(70.0)      // Forward speed in inches/sec (measured via ForwardVelocityTuner)
    .yMovement(50.0)      // Strafe speed in inches/sec (measured via StrafeVelocityTuner)
    .turningMovement(3.5)  // Turning speed in radians/sec (measured via TurnVelocityTuner)
    .build();
\`\`\`

### Movement Values
- \`xMovement\` — Measured using the **ForwardVelocityTuner** OpMode. Push the robot forward at max speed and record the value.
- \`yMovement\` — Measured using the **StrafeVelocityTuner** OpMode. Push the robot sideways at max speed and record the value.
- \`turningMovement\` — Measured using the **TurnVelocityTuner** OpMode. Spin the robot and record the value.

## PinpointConstants

Configures the goBILDA Pinpoint odometry computer.

\`\`\`java
PinpointConstants pinpointConstants = new PinpointConstants.Builder()
    .hardwareMapName("pinpoint")
    .forwardPodY(-5.5)     // Y offset of the forward pod from robot center (inches)
    .strafePodX(-3.0)      // X offset of the strafe pod from robot center (inches)
    .distanceUnit(DistanceUnit.INCH)
    .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
    .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
    .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
    .build();
\`\`\`

### PinpointConstants Parameters
- \`hardwareMapName\` — The name in the robot configuration for the Pinpoint device.
- \`forwardPodY\` — The Y offset (in inches) of the forward-facing dead wheel from the robot's center of rotation. Positive = in front of center. Negative = behind center.
- \`strafePodX\` — The X offset (in inches) of the strafe dead wheel from the robot's center of rotation. Positive = right of center. Negative = left of center.
- \`encoderResolution\` — The encoder pods being used. Options:
  - \`GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD\`
  - \`GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD\`
- \`forwardEncoderDirection\` / \`strafeEncoderDirection\` — Reverse if the encoder counts in the wrong direction.

## OTOSConstants

Configures the SparkFun OTOS (Optical Tracking Odometry Sensor).

\`\`\`java
OTOSConstants otosConstants = new OTOSConstants.Builder()
    .hardwareMapName("otos")
    .linearUnit(DistanceUnit.INCH)
    .angleUnit(AngleUnit.RADIANS)
    .offset(new SparkFunOTOS.Pose2D(0, 0, Math.toRadians(90)))
    .linearScalar(1.0)    // Calibration scalar for distance (tune if distances are off)
    .angularScalar(1.0)   // Calibration scalar for rotation (tune if angles are off)
    .build();
\`\`\`

### OTOSConstants Parameters
- \`offset\` — The physical offset of the OTOS sensor from the center of the robot as a \`Pose2D(x, y, heading)\`.
- \`linearScalar\` — Multiplied against all distance measurements. Increase if robot undershoots, decrease if overshoots.
- \`angularScalar\` — Multiplied against all angle measurements. Same tuning logic as linearScalar.

## PathConstraints

Constrains maximum velocities and accelerations during path following.

\`\`\`java
PathConstraints pathConstraints = new PathConstraints.Builder()
    .maxVelocity(60.0)              // Max translational velocity in inches/sec
    .maxAcceleration(60.0)          // Max translational acceleration in inches/sec^2
    .maxAngularVelocity(Math.PI)    // Max angular velocity in radians/sec
    .maxAngularAcceleration(Math.PI) // Max angular acceleration in radians/sec^2
    .build();
\`\`\`

## FollowerBuilder

Assembles the Follower from all constants. This is the entry point.

\`\`\`java
Follower follower = new FollowerBuilder(followerConstants, hardwareMap)
    .mecanumDrivetrain(mecanumConstants)
    .pinpointLocalizer(pinpointConstants)    // OR .otosLocalizer(otosConstants)
    .pathConstraints(pathConstraints)
    .build();
\`\`\`

## Complete Constants.java Example

\`\`\`java
package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.GoBildaPinpointDriver;

public class Constants {

    public static FollowerConstants followerConstants = new FollowerConstants.Builder()
        .mass(13.2)
        .translationalPIDFCoefficients(0.15, 0, 0.02, 0)
        .headingPIDFCoefficients(2.0, 0, 0.1, 0)
        .drivePIDFCoefficients(0.02, 0, 0.001, 0.6)
        .zeroPowerAccelerationMultiplier(4.0)
        .translationalPIDFFeedForward(0.015)
        .headingPIDFFeedForward(0.01)
        .centripetalScaling(0.0005)
        .pathEndTimeoutConstraint(3.0)
        .pathEndTValueConstraint(0.995)
        .pathEndVelocityConstraint(0.1)
        .build();

    public static MecanumConstants mecanumConstants = new MecanumConstants.Builder()
        .leftFrontMotorName("leftFront")
        .leftRearMotorName("leftRear")
        .rightFrontMotorName("rightFront")
        .rightRearMotorName("rightRear")
        .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
        .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
        .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
        .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
        .xMovement(65.0)
        .yMovement(48.0)
        .turningMovement(3.2)
        .build();

    public static PinpointConstants pinpointConstants = new PinpointConstants.Builder()
        .hardwareMapName("pinpoint")
        .forwardPodY(-5.5)
        .strafePodX(-3.0)
        .distanceUnit(DistanceUnit.INCH)
        .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
        .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
        .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
        .build();

    public static PathConstraints pathConstraints = new PathConstraints.Builder()
        .maxVelocity(60.0)
        .maxAcceleration(60.0)
        .maxAngularVelocity(Math.PI)
        .maxAngularAcceleration(Math.PI)
        .build();
}
\`\`\`
`,

  coordinateSystem: `# Pedro Pathing 2.0 - Coordinate System

## Field Coordinate System

Pedro Pathing uses a coordinate system where:

- **Origin (0, 0)** is at the **bottom-left corner** of the field (when viewed from the standard FTC field diagram orientation).
- **X-axis** runs left to right (0 to 144 inches).
- **Y-axis** runs bottom to top (0 to 144 inches).
- The full field is **144 x 144 inches** (12 feet x 12 feet).

\`\`\`
  144 ┌─────────────────────────────┐
      │                             │
      │         FTC Field           │
  Y   │                             │
      │                             │
      │                             │
    0 └─────────────────────────────┘
      0             X             144
\`\`\`

## Headings

- Headings are specified in **radians**.
- 0 radians = facing right (+X direction).
- Math.PI / 2 (90 degrees) = facing up (+Y direction).
- Math.PI (180 degrees) = facing left (-X direction).
- 3 * Math.PI / 2 or -Math.PI / 2 (270 degrees) = facing down (-Y direction).

### Conversion

Use \`Math.toRadians(degrees)\` to convert degrees to radians:
\`\`\`java
double heading90 = Math.toRadians(90);    // PI / 2
double heading180 = Math.toRadians(180);  // PI
double heading270 = Math.toRadians(270);  // 3 * PI / 2
double heading315 = Math.toRadians(315);  // 7 * PI / 4
\`\`\`

## Converting from Road Runner Coordinates

Road Runner uses a coordinate system where (0, 0) is at the **center** of the field, with:
- X range: -72 to +72
- Y range: -72 to +72

To convert from Road Runner coordinates to Pedro Pathing coordinates:

\`\`\`java
// Road Runner -> Pedro Pathing
double pedroX = roadRunnerX + 72;
double pedroY = roadRunnerY + 72;

// Pedro Pathing -> Road Runner
double rrX = pedroX - 72;
double rrY = pedroY - 72;
\`\`\`

### Examples
| Road Runner | Pedro Pathing |
|-------------|---------------|
| (0, 0) | (72, 72) |
| (-72, -72) | (0, 0) |
| (72, 72) | (144, 144) |
| (-36, 48) | (36, 120) |
| (24, -60) | (96, 12) |

## Common Field Positions (Into The Deep 2024-2025)

These are approximate positions in Pedro coordinates:

### Red Alliance (starting on right side)
- **Start Pose (observation side)**: \`new Pose(9, 111, Math.toRadians(270))\`
- **Start Pose (specimen side)**: \`new Pose(9, 63, Math.toRadians(270))\`
- **High Basket**: \`new Pose(14, 129, Math.toRadians(315))\`
- **Observation Zone**: \`new Pose(9, 15, Math.toRadians(0))\`
- **Submersible (center)**: \`new Pose(72, 96, Math.toRadians(0))\`
- **Samples on spike marks**: Around x=48, y=120–130 area

### Blue Alliance
Blue alliance positions are mirrored. Swap coordinates appropriately for your field setup.

## Best Practices

1. **Always use \`Math.toRadians()\`** for heading values for readability.
2. **Measure positions empirically** — place the robot on the field and read coordinates from telemetry.
3. **Use FTC Dashboard field overlay** to visualize and verify your coordinates.
4. **Name your poses descriptively** — \`scorePose\`, \`pickupPose\`, \`parkPose\`, etc.
5. **Define poses as static fields** or @Config annotated fields for easy tuning.
`,

  autoStructure: `# Pedro Pathing 2.0 - Autonomous Structure

## Architecture Overview

Pedro autonomous programs use a **finite state machine (FSM)** pattern with:
1. **Poses** — Define key positions on the field.
2. **PathChains** — Prebuilt paths between poses (built in \`buildPaths()\`).
3. **State Machine** — A \`switch/case\` block that transitions between states based on \`follower.isBusy()\`, timers, and other conditions.
4. **Path Timer** — A \`Timer\` used for time-based waits within states.

## Complete 3-Sample Scoring Autonomous Example

\`\`\`java
package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Point;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Constants;

@Config
@Autonomous(name = "Sample Auto - 3+1", group = "Auto")
public class SampleAuto extends LinearOpMode {

    // Follower
    private Follower follower;

    // Timer for state transitions
    private Timer pathTimer;

    // State machine
    private int pathState = 0;

    // ---- Tunable Poses (adjustable via FTC Dashboard) ---- //

    public static double START_X = 9;
    public static double START_Y = 111;
    public static double START_HEADING = 270;

    public static double SCORE_X = 14;
    public static double SCORE_Y = 129;
    public static double SCORE_HEADING = 315;

    public static double PICKUP_1_X = 37;
    public static double PICKUP_1_Y = 121;
    public static double PICKUP_1_HEADING = 0;

    public static double PICKUP_2_X = 37;
    public static double PICKUP_2_Y = 131;
    public static double PICKUP_2_HEADING = 0;

    public static double PICKUP_3_X = 44;
    public static double PICKUP_3_Y = 131;
    public static double PICKUP_3_HEADING = 30;

    public static double PARK_X = 60;
    public static double PARK_Y = 96;
    public static double PARK_HEADING = 90;

    // ---- Pose Objects ---- //

    private Pose startPose;
    private Pose scorePose;
    private Pose pickup1Pose;
    private Pose pickup2Pose;
    private Pose pickup3Pose;
    private Pose parkPose;

    // ---- PathChains ---- //

    private PathChain scorePreload;
    private PathChain grabPickup1;
    private PathChain scorePickup1;
    private PathChain grabPickup2;
    private PathChain scorePickup2;
    private PathChain grabPickup3;
    private PathChain scorePickup3;
    private PathChain park;

    /** Build all Poses from the tunable static fields */
    private void buildPoses() {
        startPose   = new Pose(START_X, START_Y, Math.toRadians(START_HEADING));
        scorePose   = new Pose(SCORE_X, SCORE_Y, Math.toRadians(SCORE_HEADING));
        pickup1Pose = new Pose(PICKUP_1_X, PICKUP_1_Y, Math.toRadians(PICKUP_1_HEADING));
        pickup2Pose = new Pose(PICKUP_2_X, PICKUP_2_Y, Math.toRadians(PICKUP_2_HEADING));
        pickup3Pose = new Pose(PICKUP_3_X, PICKUP_3_Y, Math.toRadians(PICKUP_3_HEADING));
        parkPose    = new Pose(PARK_X, PARK_Y, Math.toRadians(PARK_HEADING));
    }

    /** Build all PathChains. Call after buildPoses() and follower initialization. */
    private void buildPaths() {
        // Score the preloaded sample
        scorePreload = follower.pathBuilder()
            .addPath(new BezierLine(new Point(startPose), new Point(scorePose)))
            .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
            .build();

        // Drive to first ground sample
        grabPickup1 = follower.pathBuilder()
            .addPath(new BezierLine(new Point(scorePose), new Point(pickup1Pose)))
            .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
            .build();

        // Score first ground sample
        scorePickup1 = follower.pathBuilder()
            .addPath(new BezierLine(new Point(pickup1Pose), new Point(scorePose)))
            .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
            .build();

        // Drive to second ground sample
        grabPickup2 = follower.pathBuilder()
            .addPath(new BezierLine(new Point(scorePose), new Point(pickup2Pose)))
            .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
            .build();

        // Score second ground sample
        scorePickup2 = follower.pathBuilder()
            .addPath(new BezierLine(new Point(pickup2Pose), new Point(scorePose)))
            .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
            .build();

        // Drive to third ground sample (with curve to avoid obstacles)
        grabPickup3 = follower.pathBuilder()
            .addPath(new BezierCurve(
                new Point(scorePose),
                new Point(30, 115, Point.CARTESIAN),
                new Point(pickup3Pose)
            ))
            .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
            .build();

        // Score third ground sample
        scorePickup3 = follower.pathBuilder()
            .addPath(new BezierCurve(
                new Point(pickup3Pose),
                new Point(30, 115, Point.CARTESIAN),
                new Point(scorePose)
            ))
            .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
            .build();

        // Park
        park = follower.pathBuilder()
            .addPath(new BezierCurve(
                new Point(scorePose),
                new Point(60, 120, Point.CARTESIAN),
                new Point(parkPose)
            ))
            .setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading())
            .build();
    }

    /** Set the path state and reset the path timer */
    private void setPathState(int state) {
        pathState = state;
        pathTimer.resetTimer();
    }

    /** Main autonomous state machine */
    private void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Score preloaded sample
                follower.followPath(scorePreload, true);
                setPathState(1);
                break;

            case 1: // Wait for preload path, then score
                if (!follower.isBusy()) {
                    // Score the preloaded sample (e.g., raise arm, open claw)
                    // robot.scoreSample();
                    setPathState(2);
                }
                break;

            case 2: // Wait for scoring action, then go to pickup 1
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    follower.followPath(grabPickup1, true);
                    setPathState(3);
                }
                break;

            case 3: // Wait for pickup 1 path, then grab
                if (!follower.isBusy()) {
                    // Grab sample 1 (e.g., lower arm, close claw)
                    // robot.grabSample();
                    setPathState(4);
                }
                break;

            case 4: // Wait for grab, then drive to score
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    follower.followPath(scorePickup1, true);
                    setPathState(5);
                }
                break;

            case 5: // Wait for score path, then score
                if (!follower.isBusy()) {
                    // robot.scoreSample();
                    setPathState(6);
                }
                break;

            case 6: // Wait for scoring, then go to pickup 2
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    follower.followPath(grabPickup2, true);
                    setPathState(7);
                }
                break;

            case 7: // Wait for pickup 2 path, then grab
                if (!follower.isBusy()) {
                    // robot.grabSample();
                    setPathState(8);
                }
                break;

            case 8: // Wait for grab, then drive to score
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    follower.followPath(scorePickup2, true);
                    setPathState(9);
                }
                break;

            case 9: // Wait for score path, then score
                if (!follower.isBusy()) {
                    // robot.scoreSample();
                    setPathState(10);
                }
                break;

            case 10: // Wait for scoring, then go to pickup 3
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    follower.followPath(grabPickup3, true);
                    setPathState(11);
                }
                break;

            case 11: // Wait for pickup 3 path, then grab
                if (!follower.isBusy()) {
                    // robot.grabSample();
                    setPathState(12);
                }
                break;

            case 12: // Wait for grab, then drive to score
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    follower.followPath(scorePickup3, true);
                    setPathState(13);
                }
                break;

            case 13: // Wait for score path, then score
                if (!follower.isBusy()) {
                    // robot.scoreSample();
                    setPathState(14);
                }
                break;

            case 14: // Wait for scoring, then park
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    follower.followPath(park, true);
                    setPathState(15);
                }
                break;

            case 15: // Wait for park
                if (!follower.isBusy()) {
                    setPathState(-1); // Done
                }
                break;

            default:
                // Autonomous complete — do nothing
                break;
        }
    }

    @Override
    public void runOpMode() {
        // Initialize telemetry with FTC Dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Initialize path timer
        pathTimer = new Timer();

        // Build poses from tunable fields
        buildPoses();

        // Build follower from constants
        follower = new FollowerBuilder(Constants.followerConstants, hardwareMap)
            .mecanumDrivetrain(Constants.mecanumConstants)
            .pinpointLocalizer(Constants.pinpointConstants)
            .pathConstraints(Constants.pathConstraints)
            .build();

        // Set starting pose
        follower.setStartingPose(startPose);

        // Build all paths
        buildPaths();

        // Initialize robot hardware (subsystems, etc.)
        // robot = new Robot(hardwareMap);

        // Wait for start
        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("Status", "Initialized - 3+1 Sample Auto");
            telemetry.addData("Start Pose", "(%s, %s, %s)", START_X, START_Y, START_HEADING);
            telemetry.update();
        }

        // Start
        pathTimer.resetTimer();

        // Main autonomous loop
        while (opModeIsActive() && !isStopRequested()) {
            // Update follower — CRITICAL: must be called every loop
            follower.update();

            // Run state machine
            autonomousPathUpdate();

            // Telemetry
            telemetry.addData("Path State", pathState);
            telemetry.addData("Follower Busy", follower.isBusy());
            telemetry.addData("Timer", "%.2f sec", pathTimer.getElapsedTimeSeconds());
            telemetry.addData("Pose", "(%s, %s, %s)",
                String.format("%.1f", follower.getPose().getX()),
                String.format("%.1f", follower.getPose().getY()),
                String.format("%.1f", Math.toDegrees(follower.getPose().getHeading()))
            );
            telemetry.update();

            // FTC Dashboard field overlay
            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay()
                .setStroke("#3F51B5")
                .strokeCircle(follower.getPose().getX(), follower.getPose().getY(), 9)
                .setStroke("#FF0000")
                .strokeLine(
                    follower.getPose().getX(),
                    follower.getPose().getY(),
                    follower.getPose().getX() + 12 * Math.cos(follower.getPose().getHeading()),
                    follower.getPose().getY() + 12 * Math.sin(follower.getPose().getHeading())
                );
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
    }
}
\`\`\`

## Key Patterns

### State Machine Pattern
\`\`\`java
private int pathState = 0;

private void setPathState(int state) {
    pathState = state;
    pathTimer.resetTimer();
}
\`\`\`
- Each state corresponds to one action (drive, score, pickup, wait, etc.).
- \`setPathState()\` always resets the timer so each state has its own elapsed time.
- Use \`follower.isBusy()\` to detect when a path is complete.
- Use \`pathTimer.getElapsedTimeSeconds()\` for time-based waits (e.g., waiting for a mechanism).

### Path Building
- Build ALL paths in \`buildPaths()\` during \`init\`, NOT during the loop.
- Use \`follower.pathBuilder()\` to create PathChains.
- Multi-segment paths: chain multiple \`addPath()\` calls in one builder.

### Follower Update
\`\`\`java
// MUST be called every loop iteration
follower.update();
\`\`\`
This is **critical**. Without \`follower.update()\`, the robot will not move and localization will not update.

### @Config Tunable Poses
Using the \`@Config\` annotation with \`public static\` fields allows real-time tuning via FTC Dashboard without redeploying code:
\`\`\`java
@Config
public class SampleAuto extends LinearOpMode {
    public static double SCORE_X = 14;
    public static double SCORE_Y = 129;
    public static double SCORE_HEADING = 315;
    // These can be changed live in the FTC Dashboard
}
\`\`\`

### MultipleTelemetry
Sends telemetry to both the Driver Station and FTC Dashboard:
\`\`\`java
telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
\`\`\`
`,

  teleopStructure: `# Pedro Pathing 2.0 - TeleOp Structure

## Overview

Pedro Pathing can be used in TeleOp for:
1. **Manual driving** — Field-centric or robot-centric control via gamepads.
2. **Automated path following** — Press a button to have the robot autonomously drive to a position while you control mechanisms.
3. **Slow mode** — Toggle reduced speed for precise movements.

## Complete TeleOp Example

\`\`\`java
package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Point;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants;

import java.util.function.Supplier;

@TeleOp(name = "Main TeleOp", group = "TeleOp")
public class MainTeleOp extends OpMode {

    private Follower follower;

    // Drive power multiplier
    private double driveSpeed = 1.0;
    private boolean slowModeActive = false;
    private boolean previousLeftBumper = false;

    // Automated path following state
    private boolean isAutoPathing = false;

    // ---- Predefined Poses ---- //
    private static final Pose SCORE_POSE = new Pose(14, 129, Math.toRadians(315));
    private static final Pose PICKUP_POSE = new Pose(37, 121, Math.toRadians(0));
    private static final Pose HANG_POSE = new Pose(72, 96, Math.toRadians(90));

    // ---- Lazy PathChain Suppliers ---- //
    // These regenerate the path from the robot's CURRENT position every time they're called.
    // This avoids stale paths when the robot has drifted or been bumped.

    private Supplier<PathChain> pathToScore;
    private Supplier<PathChain> pathToPickup;
    private Supplier<PathChain> pathToHang;

    @Override
    public void init() {
        // Initialize telemetry
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Build follower
        follower = new FollowerBuilder(Constants.followerConstants, hardwareMap)
            .mecanumDrivetrain(Constants.mecanumConstants)
            .pinpointLocalizer(Constants.pinpointConstants)
            .pathConstraints(Constants.pathConstraints)
            .build();

        // Set the starting pose (should match where auto ended, or a known position)
        follower.setStartingPose(new Pose(0, 0, Math.toRadians(0)));

        // Define lazy path suppliers — paths are rebuilt from current pose each time
        pathToScore = () -> follower.pathBuilder()
            .addPath(new BezierCurve(
                new Point(follower.getPose()),
                new Point(follower.getPose().getX() + 10, follower.getPose().getY() + 10, Point.CARTESIAN),
                new Point(SCORE_POSE)
            ))
            .setLinearHeadingInterpolation(follower.getPose().getHeading(), SCORE_POSE.getHeading())
            .build();

        pathToPickup = () -> follower.pathBuilder()
            .addPath(new BezierLine(
                new Point(follower.getPose()),
                new Point(PICKUP_POSE)
            ))
            .setLinearHeadingInterpolation(follower.getPose().getHeading(), PICKUP_POSE.getHeading())
            .build();

        pathToHang = () -> follower.pathBuilder()
            .addPath(new BezierCurve(
                new Point(follower.getPose()),
                new Point(50, 110, Point.CARTESIAN),
                new Point(HANG_POSE)
            ))
            .setLinearHeadingInterpolation(follower.getPose().getHeading(), HANG_POSE.getHeading())
            .build();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        // IMPORTANT: Call this once to initialize TeleOp driving
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        // ---- Slow Mode Toggle (left bumper) ---- //
        if (gamepad1.left_bumper && !previousLeftBumper) {
            slowModeActive = !slowModeActive;
            driveSpeed = slowModeActive ? 0.35 : 1.0;
        }
        previousLeftBumper = gamepad1.left_bumper;

        // ---- Automated Path Following ---- //
        // Press Y to auto-drive to scoring position
        if (gamepad1.y && !isAutoPathing) {
            isAutoPathing = true;
            follower.followPath(pathToScore.get(), true);
        }

        // Press A to auto-drive to pickup position
        if (gamepad1.a && !isAutoPathing) {
            isAutoPathing = true;
            follower.followPath(pathToPickup.get(), true);
        }

        // Press X to auto-drive to hang position
        if (gamepad1.x && !isAutoPathing) {
            isAutoPathing = true;
            follower.followPath(pathToHang.get(), true);
        }

        // Press B to cancel auto-pathing and return to manual control
        if (gamepad1.b && isAutoPathing) {
            isAutoPathing = false;
            follower.breakFollowing();
            follower.startTeleopDrive();
        }

        // Check if auto-path is complete
        if (isAutoPathing && !follower.isBusy()) {
            isAutoPathing = false;
            follower.startTeleopDrive();
        }

        // ---- Manual Driving (only when not auto-pathing) ---- //
        if (!isAutoPathing) {
            follower.setTeleOpDrive(
                -gamepad1.left_stick_y * driveSpeed,   // forward
                -gamepad1.left_stick_x * driveSpeed,   // strafe
                -gamepad1.right_stick_x * driveSpeed,  // turn
                true                                    // robot-centric (set false for field-centric)
            );
        }

        // ---- Always update the follower ---- //
        follower.update();

        // ---- Telemetry ---- //
        telemetry.addData("Mode", isAutoPathing ? "AUTO-PATH" : "MANUAL");
        telemetry.addData("Slow Mode", slowModeActive ? "ON (35%)" : "OFF (100%)");
        telemetry.addData("Drive Speed", "%.0f%%", driveSpeed * 100);
        telemetry.addData("Pose", "(%.1f, %.1f, %.1f deg)",
            follower.getPose().getX(),
            follower.getPose().getY(),
            Math.toDegrees(follower.getPose().getHeading())
        );
        telemetry.addData("Velocity", "%.1f in/s", follower.getVelocityMagnitude());
        telemetry.addLine();
        telemetry.addData("Controls", "Y=Score, A=Pickup, X=Hang, B=Cancel");
        telemetry.update();
    }

    @Override
    public void stop() {
        // Clean shutdown
    }
}
\`\`\`

## Key Patterns Explained

### Starting TeleOp Drive

You **must** call \`follower.startTeleopDrive()\` before using \`setTeleOpDrive()\`. This initializes the follower for manual control mode.

\`\`\`java
@Override
public void start() {
    follower.startTeleopDrive();
}
\`\`\`

### Manual Drive Input

\`\`\`java
follower.setTeleOpDrive(
    -gamepad1.left_stick_y,   // forward (negated: stick up = positive = forward)
    -gamepad1.left_stick_x,   // strafe  (negated: stick left = positive = strafe left)
    -gamepad1.right_stick_x,  // turn    (negated: stick left = positive = turn left)
    true                       // robotCentric: true = robot-relative, false = field-centric
);
\`\`\`

All stick values are **negated** because gamepad sticks report negative values when pushed up/left, but we want positive to mean forward/left.

### Slow Mode Toggle

\`\`\`java
private double driveSpeed = 1.0;
private boolean slowModeActive = false;
private boolean previousLeftBumper = false;

// In loop():
if (gamepad1.left_bumper && !previousLeftBumper) {
    slowModeActive = !slowModeActive;
    driveSpeed = slowModeActive ? 0.35 : 1.0;
}
previousLeftBumper = gamepad1.left_bumper;
\`\`\`

The \`previousLeftBumper\` pattern ensures the toggle only fires once per button press (edge detection).

### Supplier<PathChain> — Lazy Curve Generation

The \`Supplier<PathChain>\` pattern is **critical** for TeleOp automated paths. Instead of prebuilding paths (which would use stale starting positions), we generate them on-demand from the robot's current pose:

\`\`\`java
// Define the supplier (once, in init)
Supplier<PathChain> pathToTarget = () -> follower.pathBuilder()
    .addPath(new BezierLine(
        new Point(follower.getPose()),   // Always starts from CURRENT position
        new Point(targetPose)
    ))
    .setLinearHeadingInterpolation(follower.getPose().getHeading(), targetPose.getHeading())
    .build();

// Use the supplier (generates fresh path each time)
follower.followPath(pathToTarget.get(), true);
\`\`\`

### Switching Between Manual and Auto

When transitioning from auto-pathing back to manual driving:
\`\`\`java
follower.breakFollowing();       // Stop auto-path
follower.startTeleopDrive();     // Re-initialize manual control
\`\`\`

Always check \`isAutoPathing\` before sending manual drive commands, or they will interfere with the path follower.
`,

  callbacks: `# Pedro Pathing 2.0 - Path Callbacks

## Overview

Callbacks let you trigger actions at specific points during path following. Pedro supports two types:

1. **Parametric Callbacks** (RECOMMENDED) — Fire based on path progress (0.0 to 1.0 along the path).
2. **Temporal Callbacks** (NOT RECOMMENDED) — Fire after a specific elapsed time.

## Parametric Callbacks

Parametric callbacks fire when the robot reaches a specific parametric t-value along a path segment. The t-value ranges from 0.0 (start of path segment) to 1.0 (end of path segment).

### Syntax
\`\`\`java
.addParametricCallback(double tValue, Runnable callback)
\`\`\`

### Why Parametric Callbacks Are Recommended
- **Consistent** — They fire at the same physical position regardless of robot speed or battery voltage.
- **Predictable** — t=0.5 always means "halfway through this path segment" by distance.
- **Reliable** — No dependency on execution timing.

### Examples
\`\`\`java
PathChain scorePath = follower.pathBuilder()
    .addPath(new BezierLine(new Point(startPose), new Point(scorePose)))
    .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
    // Raise arm when 30% through the path
    .addParametricCallback(0.3, () -> {
        robot.arm.setTargetPosition(ARM_SCORE_POSITION);
        robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.arm.setPower(1.0);
    })
    // Open claw when 95% through the path (near the end)
    .addParametricCallback(0.95, () -> {
        robot.claw.setPosition(CLAW_OPEN);
    })
    .build();
\`\`\`

### Multi-Segment Path Callbacks
When a PathChain has multiple path segments, each segment has its own independent t-values (0.0–1.0). Callbacks are attached to the path segment they immediately follow:

\`\`\`java
PathChain multiPath = follower.pathBuilder()
    // First path segment
    .addPath(new BezierLine(new Point(startPose), new Point(midPose)))
    .setLinearHeadingInterpolation(startPose.getHeading(), midPose.getHeading())
    // This callback fires at t=0.5 of the FIRST segment
    .addParametricCallback(0.5, () -> {
        robot.intake.setPower(1.0);
    })

    // Second path segment
    .addPath(new BezierLine(new Point(midPose), new Point(endPose)))
    .setLinearHeadingInterpolation(midPose.getHeading(), endPose.getHeading())
    // This callback fires at t=0.8 of the SECOND segment
    .addParametricCallback(0.8, () -> {
        robot.arm.setTargetPosition(ARM_HIGH);
    })
    .build();
\`\`\`

## Temporal Callbacks

Temporal callbacks fire after a specified number of seconds have elapsed since the start of the path segment. They are **NOT recommended** because timing varies with battery voltage, friction, and robot weight.

### Syntax
\`\`\`java
.addTemporalCallback(double seconds, Runnable callback)
\`\`\`

### Why Temporal Callbacks Are Not Recommended
- **Inconsistent** — A path that takes 2 seconds on a full battery might take 3 seconds on a depleted battery.
- **Fragile** — Any change to path speed, constraints, or path shape can break timing.
- **Hard to tune** — You must re-tune timing values whenever anything else changes.

### Example (for reference only)
\`\`\`java
PathChain path = follower.pathBuilder()
    .addPath(new BezierLine(new Point(startPose), new Point(endPose)))
    .setLinearHeadingInterpolation(startPose.getHeading(), endPose.getHeading())
    // Fire 1.5 seconds after starting this path segment — NOT RECOMMENDED
    .addTemporalCallback(1.5, () -> {
        robot.arm.setTargetPosition(ARM_HIGH);
    })
    .build();
\`\`\`

## Pause / Resume for Mid-Path Actions

Sometimes you need the robot to stop mid-path, perform an action, and then continue. Use \`pausePathFollowing()\` and \`resumePathFollowing()\` for this.

### Pattern
\`\`\`java
// In your state machine:
case 5: // Following a path, waiting for mid-point to pause
    if (follower.getCurrentTValue() > 0.5 && follower.getCurrentPathNumber() == 0) {
        follower.pausePathFollowing();
        // Robot will hold its current position
        robot.intake.grabSample();
        setPathState(6);
    }
    break;

case 6: // Waiting for grab to complete, then resume
    if (pathTimer.getElapsedTimeSeconds() > 0.8) {
        follower.resumePathFollowing();
        setPathState(7);
    }
    break;

case 7: // Wait for the rest of the path to complete
    if (!follower.isBusy()) {
        setPathState(8);
    }
    break;
\`\`\`

### Notes on Pause / Resume
- \`pausePathFollowing()\` makes the robot **hold its current position** using PID. It does not coast.
- \`resumePathFollowing()\` continues from where the path was paused.
- The robot resumes from the same parametric position — it does not restart the path.

## Using Callbacks with Pause/Resume

You can combine parametric callbacks with pause/resume for sophisticated mid-path behavior:

\`\`\`java
// Use a flag to signal the state machine
private volatile boolean shouldPause = false;

PathChain pickupPath = follower.pathBuilder()
    .addPath(new BezierCurve(
        new Point(scorePose),
        new Point(30, 110, Point.CARTESIAN),
        new Point(pickupPose)
    ))
    .setLinearHeadingInterpolation(scorePose.getHeading(), pickupPose.getHeading())
    // Lower intake at 40% through
    .addParametricCallback(0.4, () -> {
        robot.intake.lower();
    })
    // Start intake spinning at 70%
    .addParametricCallback(0.7, () -> {
        robot.intake.spin(1.0);
    })
    .build();
\`\`\`

## Complete Callback Example

\`\`\`java
package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Point;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Constants;

@Config
@Autonomous(name = "Callback Auto Example", group = "Auto")
public class CallbackAutoExample extends LinearOpMode {

    private Follower follower;
    private Timer pathTimer;
    private int pathState = 0;

    // Mechanism state tracking
    private String lastAction = "none";

    // Poses
    private final Pose startPose = new Pose(9, 111, Math.toRadians(270));
    private final Pose scorePose = new Pose(14, 129, Math.toRadians(315));
    private final Pose pickupPose = new Pose(37, 121, Math.toRadians(0));
    private final Pose parkPose = new Pose(60, 96, Math.toRadians(90));

    // Paths
    private PathChain scorePreload;
    private PathChain grabFromGround;
    private PathChain scoreFromPickup;
    private PathChain parkPath;

    private void buildPaths() {
        // Score preload with arm raise callback
        scorePreload = follower.pathBuilder()
            .addPath(new BezierLine(new Point(startPose), new Point(scorePose)))
            .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
            // Raise arm early in the path so it's ready when we arrive
            .addParametricCallback(0.2, () -> {
                lastAction = "Raising arm to score position";
                // robot.arm.setPosition(ARM_SCORE);
            })
            // Prepare to release near the end
            .addParametricCallback(0.9, () -> {
                lastAction = "Preparing release";
                // robot.wrist.setPosition(WRIST_SCORE);
            })
            .build();

        // Pick up ground sample with intake deployment callback
        grabFromGround = follower.pathBuilder()
            .addPath(new BezierCurve(
                new Point(scorePose),
                new Point(30, 115, Point.CARTESIAN),
                new Point(pickupPose)
            ))
            .setLinearHeadingInterpolation(scorePose.getHeading(), pickupPose.getHeading())
            // Lower arm while driving
            .addParametricCallback(0.3, () -> {
                lastAction = "Lowering arm to pickup";
                // robot.arm.setPosition(ARM_PICKUP);
            })
            // Start intake spinning before arrival
            .addParametricCallback(0.7, () -> {
                lastAction = "Starting intake";
                // robot.intake.setPower(1.0);
            })
            .build();

        // Score from pickup with arm raise
        scoreFromPickup = follower.pathBuilder()
            .addPath(new BezierCurve(
                new Point(pickupPose),
                new Point(30, 115, Point.CARTESIAN),
                new Point(scorePose)
            ))
            .setLinearHeadingInterpolation(pickupPose.getHeading(), scorePose.getHeading())
            // Transfer sample
            .addParametricCallback(0.1, () -> {
                lastAction = "Transferring sample";
                // robot.intake.setPower(0);
                // robot.claw.close();
            })
            // Raise arm
            .addParametricCallback(0.4, () -> {
                lastAction = "Raising arm to score";
                // robot.arm.setPosition(ARM_SCORE);
            })
            .build();

        // Park with arm stow
        parkPath = follower.pathBuilder()
            .addPath(new BezierCurve(
                new Point(scorePose),
                new Point(40, 115, Point.CARTESIAN),
                new Point(parkPose)
            ))
            .setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading())
            .addParametricCallback(0.2, () -> {
                lastAction = "Stowing arm for park";
                // robot.arm.setPosition(ARM_STOW);
            })
            .build();
    }

    private void setPathState(int state) {
        pathState = state;
        pathTimer.resetTimer();
    }

    private void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Start — follow preload score path
                follower.followPath(scorePreload, true);
                setPathState(1);
                break;

            case 1: // Wait for path, score
                if (!follower.isBusy()) {
                    lastAction = "Scoring preload";
                    // robot.claw.open();
                    setPathState(2);
                }
                break;

            case 2: // Wait for score, go pickup
                if (pathTimer.getElapsedTimeSeconds() > 0.4) {
                    follower.followPath(grabFromGround, true);
                    setPathState(3);
                }
                break;

            case 3: // Wait for pickup path, grab
                if (!follower.isBusy()) {
                    lastAction = "Grabbing sample";
                    // robot.claw.close();
                    setPathState(4);
                }
                break;

            case 4: // Wait for grab, go score
                if (pathTimer.getElapsedTimeSeconds() > 0.4) {
                    follower.followPath(scoreFromPickup, true);
                    setPathState(5);
                }
                break;

            case 5: // Wait for score path, score
                if (!follower.isBusy()) {
                    lastAction = "Scoring sample";
                    // robot.claw.open();
                    setPathState(6);
                }
                break;

            case 6: // Wait for score, park
                if (pathTimer.getElapsedTimeSeconds() > 0.4) {
                    follower.followPath(parkPath, true);
                    setPathState(7);
                }
                break;

            case 7: // Wait for park
                if (!follower.isBusy()) {
                    lastAction = "Parked — auto complete";
                    setPathState(-1);
                }
                break;

            default:
                break;
        }
    }

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        pathTimer = new Timer();

        follower = new FollowerBuilder(Constants.followerConstants, hardwareMap)
            .mecanumDrivetrain(Constants.mecanumConstants)
            .pinpointLocalizer(Constants.pinpointConstants)
            .pathConstraints(Constants.pathConstraints)
            .build();

        follower.setStartingPose(startPose);
        buildPaths();

        waitForStart();

        pathTimer.resetTimer();

        while (opModeIsActive()) {
            follower.update();
            autonomousPathUpdate();

            telemetry.addData("State", pathState);
            telemetry.addData("Last Action", lastAction);
            telemetry.addData("Busy", follower.isBusy());
            telemetry.addData("T-Value", "%.3f", follower.getCurrentTValue());
            telemetry.addData("Path #", follower.getCurrentPathNumber());
            telemetry.addData("Pose", "(%.1f, %.1f, %.1f°)",
                follower.getPose().getX(),
                follower.getPose().getY(),
                Math.toDegrees(follower.getPose().getHeading()));
            telemetry.update();
        }
    }
}
\`\`\`

## Callback Best Practices

1. **Use parametric callbacks over temporal callbacks.** Parametric callbacks are deterministic; temporal ones are not.
2. **Keep callbacks lightweight.** Callbacks run on the main loop thread. Set a target position or flag; don't run blocking operations.
3. **Use early t-values for preparation.** Start raising an arm at t=0.2–0.3 so it's in position by the time the robot arrives.
4. **Use late t-values for actions at the destination.** t=0.9–0.95 is good for opening a claw right before arrival.
5. **Don't rely on callbacks for critical state transitions.** Use \`follower.isBusy()\` in your state machine for path completion; use callbacks for mechanism control only.
6. **Combine callbacks with state machines.** Callbacks handle mechanism movement; the state machine handles path sequencing and logic.
7. **Test callbacks thoroughly.** Use telemetry to verify callbacks fire at the expected positions.
`,
};
