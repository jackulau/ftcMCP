export const ROADRUNNER_KNOWLEDGE = {
  apiReference: `
# Road Runner 1.0 API Reference

Road Runner is a motion planning library for FTC robots that provides trajectory-based
autonomous movement with smooth, time-optimal paths. It uses a different paradigm from
Pedro Pathing — trajectories are pre-planned with velocity/acceleration constraints and
executed as time-parameterized curves.

---

## Actions System

Road Runner 1.0 uses an **Actions** system to compose robot behavior. Actions are the
fundamental unit of execution.

### SequentialAction
Runs actions one after another, waiting for each to complete before starting the next:

\`\`\`java
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Action;

Action sequence = new SequentialAction(
    drive.actionBuilder(startPose)
        .lineToX(48)
        .build(),
    liftAction,
    clawOpenAction
);
\`\`\`

### ParallelAction
Runs multiple actions simultaneously. Completes when ALL actions finish:

\`\`\`java
import com.acmerobotics.roadrunner.ParallelAction;

Action parallel = new ParallelAction(
    drive.actionBuilder(startPose)
        .lineToX(48)
        .build(),
    liftToHighAction
);
\`\`\`

### SleepAction
Pauses execution for a specified duration (seconds):

\`\`\`java
import com.acmerobotics.roadrunner.SleepAction;

Action wait = new SleepAction(0.5); // wait 500ms
\`\`\`

### Running Actions
Use \`Actions.runBlocking()\` in LinearOpMode or handle manually in iterative:

\`\`\`java
import com.acmerobotics.roadrunner.ftc.Actions;

// In LinearOpMode:
Actions.runBlocking(myAction);

// In iterative OpMode (manual update loop):
private List<Action> runningActions = new ArrayList<>();

// In loop():
TelemetryPacket packet = new TelemetryPacket();
List<Action> newActions = new ArrayList<>();
for (Action action : runningActions) {
    action.preview(packet.fieldOverlay());
    if (action.run(packet)) {
        newActions.add(action);
    }
}
runningActions = newActions;
FtcDashboard.getInstance().sendTelemetryPacket(packet);
\`\`\`

---

## TrajectoryActionBuilder

The \`TrajectoryActionBuilder\` is the primary way to construct trajectories.
Obtain one from your drive class via \`drive.actionBuilder(startPose)\`.

### Movement Methods

#### lineToX(x)
Drive forward/backward to a specific X coordinate (maintains current heading):

\`\`\`java
drive.actionBuilder(new Pose2d(0, 0, 0))
    .lineToX(48)   // drive to x=48
    .build();
\`\`\`

#### lineToY(y)
Drive to a specific Y coordinate (maintains current heading):

\`\`\`java
drive.actionBuilder(new Pose2d(0, 0, Math.toRadians(90)))
    .lineToY(48)   // drive to y=48
    .build();
\`\`\`

#### strafeTo(position)
Strafe to a specific (x, y) position while maintaining current heading:

\`\`\`java
drive.actionBuilder(startPose)
    .strafeTo(new Vector2d(48, 24))
    .build();
\`\`\`

#### splineTo(position, tangent)
Follow a smooth spline curve to a position with a specified tangent angle:

\`\`\`java
drive.actionBuilder(startPose)
    .splineTo(new Vector2d(48, 24), Math.toRadians(45))
    .build();
\`\`\`

#### turn(angle)
Turn in place by a relative angle (radians):

\`\`\`java
drive.actionBuilder(startPose)
    .turn(Math.toRadians(90))    // turn 90 degrees left
    .turn(Math.toRadians(-45))   // turn 45 degrees right
    .build();
\`\`\`

#### waitSeconds(seconds)
Pause the trajectory for a duration:

\`\`\`java
drive.actionBuilder(startPose)
    .lineToX(48)
    .waitSeconds(1.0)  // pause 1 second
    .lineToX(24)
    .build();
\`\`\`

### Chaining Methods
Methods can be chained to build complex paths:

\`\`\`java
Action trajectory = drive.actionBuilder(new Pose2d(0, 0, 0))
    .lineToX(36)
    .turn(Math.toRadians(90))
    .lineToY(36)
    .splineTo(new Vector2d(48, 48), Math.toRadians(0))
    .strafeTo(new Vector2d(24, 24))
    .waitSeconds(0.5)
    .lineToX(0)
    .build();
\`\`\`

### afterTime / afterDisp (Markers)
Execute actions at specific times or distances along a trajectory:

\`\`\`java
Action trajectory = drive.actionBuilder(startPose)
    .lineToX(48)
    .afterTime(0.5, liftUpAction)     // 0.5s after trajectory starts
    .afterDisp(20, clawOpenAction)    // after 20 inches of travel
    .build();
\`\`\`

---

## MecanumDrive Class Setup

Road Runner provides a \`MecanumDrive\` class that must be configured for your robot.
The quickstart project includes this pre-built. Key setup steps:

### Drive Configuration (in MecanumDrive.java)
\`\`\`java
public class MecanumDrive {
    // Motor names must match your robot configuration
    public static double WHEEL_RADIUS = 1.8898; // inches (96mm goBILDA mecanum)
    public static double GEAR_RATIO = 1.0;       // output/input
    public static double TRACK_WIDTH = 14.0;      // inches between left/right wheels
    
    // Drive motor max RPM
    public static double MAX_RPM = 435;           // goBILDA 435 RPM motor
    
    // These are PID coefficients for heading and translation
    public static double HEADING_KP = 8;
    public static double HEADING_KD = 1;
    public static double LATERAL_KP = 6;
    public static double LATERAL_KD = 1;
    public static double AXIAL_KP = 6;
    public static double AXIAL_KD = 1;
    
    private DcMotorEx leftFront, leftBack, rightFront, rightBack;
    
    public MecanumDrive(HardwareMap hardwareMap, Pose2d startPose) {
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        
        // Reverse motors on one side
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
    }
}
\`\`\`

### Using MecanumDrive in an OpMode
\`\`\`java
MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

Action traj = drive.actionBuilder(drive.pose)
    .lineToX(48)
    .build();

Actions.runBlocking(traj);
\`\`\`

---

## Coordinate System

Road Runner uses a **field-centered coordinate system** that differs from Pedro Pathing:

- **Origin**: Center of the field (0, 0)
- **X axis**: Positive X points toward the red alliance wall (→)
- **Y axis**: Positive Y points toward the audience/blue side (↑)
- **Angles**: 0 radians faces positive X (+→), counter-clockwise is positive
- **Units**: Inches for distance, radians for angles

### Key Differences from Pedro Pathing:
| Feature            | Road Runner              | Pedro Pathing           |
|--------------------|--------------------------|-------------------------|
| Origin             | Field center (0,0)       | Configurable start pose |
| Coordinate style   | Standard math (CCW +)    | Standard math (CCW +)   |
| Angle unit         | Radians                  | Radians                 |
| Position class     | Pose2d, Vector2d         | Pose, Point             |
| Path definition    | Trajectory (time-based)  | Path (follower-based)   |

### Pose2d and Vector2d
\`\`\`java
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));  // x, y, heading
Vector2d targetPos = new Vector2d(48, 24);                 // x, y only
\`\`\`

---

## MeepMeep Visualizer

MeepMeep is a desktop trajectory visualizer for Road Runner. It lets you preview
paths before deploying to the robot.

### Setup (separate module in Android Studio)
Add MeepMeep as a Java module in your FTC project:

\`\`\`java
// MeepMeepTesting.java - runs on your desktop, NOT on the robot
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800); // window size in pixels

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
            .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
            // maxVel, maxAccel, maxAngVel, maxAngAccel, trackWidth
            .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(0, 0, 0))
            .lineToX(48)
            .turn(Math.toRadians(90))
            .lineToY(48)
            .splineTo(new Vector2d(0, 0), Math.toRadians(180))
            .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
            .setDarkMode(true)
            .setBackgroundAlpha(0.95f)
            .addEntity(myBot)
            .start();
    }
}
\`\`\`

### Tips
- Test all trajectories in MeepMeep before deploying to the robot
- You can add multiple bots to simulate alliance partners
- Constraints in MeepMeep should match your robot's tuned constraints
- MeepMeep uses the same trajectory builder API as the real robot code

---

## Tuning Process Overview

Road Runner requires careful tuning for accurate path following. The quickstart
project includes tuning OpModes:

### Step-by-Step Tuning Order

1. **Dead Wheel / Odometry Calibration**
   - Measure wheel positions and radii precisely
   - Run \`ForwardPushTest\` — push robot forward, verify encoder ticks
   - Run \`LateralPushTest\` — push robot sideways, verify lateral encoder

2. **Drive Characterization (FeedForward)**
   - Run \`ForwardRampLogger\` — logs voltage vs. velocity
   - Run \`LateralRampLogger\` — same for strafing
   - Run \`AngularRampLogger\` — logs voltage vs. angular velocity
   - Use the Road Runner tuning web app to compute kS, kV, kA from logs

3. **Localization Test**
   - Run \`LocalizationTest\` — drive robot around manually
   - Verify the pose estimate on FTC Dashboard matches real-world position
   - Fix any encoder direction or sign issues

4. **PID / Feedback Tuning**
   - Run \`ManualFeedforwardTuner\` — fine-tune feedforward gains
   - Run trajectory tests and adjust heading/translation PID gains
   - Iterate until trajectories are accurate

5. **Full Trajectory Testing**
   - Run a test trajectory (e.g., a square) and measure error
   - Adjust gains and constraints as needed

### Tuning Tips
- Measure physical dimensions carefully (wheel radius, track width)
- Ensure motor directions are correct before tuning
- Use FTC Dashboard for real-time graph visualization during tuning
- Tune on the actual competition surface if possible

---

## Complete Working Autonomous Example

\`\`\`java
package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "RR Auto - Scoring", group = "Auto")
public class RoadRunnerAutoExample extends LinearOpMode {

    // Simple lift action — demonstrates custom Action implementation
    public class LiftToPosition implements Action {
        private final DcMotorEx liftMotor;
        private final int targetPosition;
        private boolean initialized = false;

        public LiftToPosition(DcMotorEx motor, int target) {
            this.liftMotor = motor;
            this.targetPosition = target;
        }

        @Override
        public boolean run(com.acmerobotics.dashboard.telemetry.TelemetryPacket packet) {
            if (!initialized) {
                liftMotor.setTargetPosition(targetPosition);
                liftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                liftMotor.setPower(1.0);
                initialized = true;
            }

            double currentPos = liftMotor.getCurrentPosition();
            packet.put("liftTarget", targetPosition);
            packet.put("liftPosition", currentPos);

            return Math.abs(currentPos - targetPosition) > 10;
            // return true = still running, false = done
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize drive at starting position
        Pose2d startPose = new Pose2d(-36, -60, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        // Initialize mechanisms
        DcMotorEx liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");
        liftMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        liftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        Servo claw = hardwareMap.get(Servo.class, "claw");
        claw.setPosition(0.0); // closed

        // Pre-build all trajectories during init
        Action driveToScoring = drive.actionBuilder(startPose)
            .splineTo(new Vector2d(-4, -36), Math.toRadians(90))
            .build();

        Action driveToPickup = drive.actionBuilder(new Pose2d(-4, -36, Math.toRadians(90)))
            .strafeTo(new Vector2d(-48, -36))
            .lineToY(-12)
            .build();

        Action driveToScoring2 = drive.actionBuilder(new Pose2d(-48, -12, Math.toRadians(90)))
            .splineTo(new Vector2d(-4, -36), Math.toRadians(90))
            .build();

        Action driveToPark = drive.actionBuilder(new Pose2d(-4, -36, Math.toRadians(90)))
            .strafeTo(new Vector2d(-48, -60))
            .build();

        // Build the full autonomous sequence
        Action fullAuto = new SequentialAction(
            // Score preloaded specimen
            new ParallelAction(
                driveToScoring,
                new LiftToPosition(liftMotor, 2000)
            ),
            new SleepAction(0.3),
            new com.acmerobotics.roadrunner.InstantAction(() -> claw.setPosition(1.0)),
            new SleepAction(0.3),

            // Drive to pickup
            new ParallelAction(
                driveToPickup,
                new LiftToPosition(liftMotor, 0)
            ),
            new com.acmerobotics.roadrunner.InstantAction(() -> claw.setPosition(0.0)),
            new SleepAction(0.3),

            // Score second piece
            new ParallelAction(
                driveToScoring2,
                new LiftToPosition(liftMotor, 2000)
            ),
            new SleepAction(0.3),
            new com.acmerobotics.roadrunner.InstantAction(() -> claw.setPosition(1.0)),
            new SleepAction(0.3),

            // Park
            new ParallelAction(
                driveToPark,
                new LiftToPosition(liftMotor, 0)
            )
        );

        telemetry.addLine("Road Runner Auto Ready");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        // Execute the entire autonomous routine
        Actions.runBlocking(fullAuto);

        telemetry.addLine("Autonomous Complete");
        telemetry.update();
    }
}
\`\`\`
`,
};
