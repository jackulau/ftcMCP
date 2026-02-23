export const EXAMPLES: Record<string, string> = {

"pedro-auto": `package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Config
@Autonomous(name = "Pedro Auto - 3 Sample", group = "Auto")
public class PedroAutoExample extends OpMode {

    // --- Dashboard-tunable poses ---
    public static double START_X = 9.0;
    public static double START_Y = 60.0;
    public static double START_HEADING = Math.toRadians(0);

    public static double SCORE_X = 38.0;
    public static double SCORE_Y = 68.0;
    public static double SCORE_HEADING = Math.toRadians(0);

    public static double SAMPLE1_X = 37.0;
    public static double SAMPLE1_Y = 30.0;
    public static double SAMPLE1_HEADING = Math.toRadians(0);

    public static double SAMPLE2_X = 37.0;
    public static double SAMPLE2_Y = 20.0;
    public static double SAMPLE2_HEADING = Math.toRadians(0);

    public static double SAMPLE3_X = 44.0;
    public static double SAMPLE3_Y = 20.0;
    public static double SAMPLE3_HEADING = Math.toRadians(30);

    public static double PARK_X = 10.0;
    public static double PARK_Y = 10.0;
    public static double PARK_HEADING = Math.toRadians(0);

    // Control point for curved paths
    public static double CURVE_CONTROL_X = 20.0;
    public static double CURVE_CONTROL_Y = 40.0;

    private Follower follower;
    private Timer pathTimer;
    private int pathState;

    // Path chains
    private PathChain scorePreload, grabSample1, scoreSample1;
    private PathChain grabSample2, scoreSample2;
    private PathChain grabSample3, scoreSample3;
    private PathChain park;

    private void buildPaths() {
        Pose startPose = new Pose(START_X, START_Y, START_HEADING);
        Pose scorePose = new Pose(SCORE_X, SCORE_Y, SCORE_HEADING);
        Pose sample1Pose = new Pose(SAMPLE1_X, SAMPLE1_Y, SAMPLE1_HEADING);
        Pose sample2Pose = new Pose(SAMPLE2_X, SAMPLE2_Y, SAMPLE2_HEADING);
        Pose sample3Pose = new Pose(SAMPLE3_X, SAMPLE3_Y, SAMPLE3_HEADING);
        Pose parkPose = new Pose(PARK_X, PARK_Y, PARK_HEADING);

        follower.setStartingPose(startPose);

        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(startPose),
                        new Point(scorePose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();

        grabSample1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(scorePose),
                        new Point(CURVE_CONTROL_X, CURVE_CONTROL_Y, Point.CARTESIAN),
                        new Point(sample1Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), sample1Pose.getHeading())
                .build();

        scoreSample1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(sample1Pose),
                        new Point(scorePose)))
                .setLinearHeadingInterpolation(sample1Pose.getHeading(), scorePose.getHeading())
                .build();

        grabSample2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(scorePose),
                        new Point(sample2Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), sample2Pose.getHeading())
                .build();

        scoreSample2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(sample2Pose),
                        new Point(scorePose)))
                .setLinearHeadingInterpolation(sample2Pose.getHeading(), scorePose.getHeading())
                .build();

        grabSample3 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(scorePose),
                        new Point(CURVE_CONTROL_X, CURVE_CONTROL_Y, Point.CARTESIAN),
                        new Point(sample3Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), sample3Pose.getHeading())
                .build();

        scoreSample3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(sample3Pose),
                        new Point(scorePose)))
                .setLinearHeadingInterpolation(sample3Pose.getHeading(), scorePose.getHeading())
                .build();

        park = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(scorePose),
                        new Point(CURVE_CONTROL_X, CURVE_CONTROL_Y, Point.CARTESIAN),
                        new Point(parkPose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading())
                .build();
    }

    private void setPathState(int state) {
        pathState = state;
        pathTimer.resetTimer();
    }

    private void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Drive to score preloaded sample
                if (!follower.isBusy()) {
                    follower.followPath(scorePreload, true);
                    setPathState(1);
                }
                break;

            case 1: // Wait to arrive at scoring position, then "score"
                if (!follower.isBusy()) {
                    // Score the preloaded sample (actuate mechanism here)
                    setPathState(2);
                }
                break;

            case 2: // Drive to grab sample 1
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    follower.followPath(grabSample1, true);
                    setPathState(3);
                }
                break;

            case 3: // Wait to arrive at sample 1, then grab
                if (!follower.isBusy()) {
                    // Grab sample 1 (actuate mechanism here)
                    setPathState(4);
                }
                break;

            case 4: // Drive to score sample 1
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    follower.followPath(scoreSample1, true);
                    setPathState(5);
                }
                break;

            case 5: // Wait to arrive, then score sample 1
                if (!follower.isBusy()) {
                    // Score sample 1
                    setPathState(6);
                }
                break;

            case 6: // Drive to grab sample 2
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    follower.followPath(grabSample2, true);
                    setPathState(7);
                }
                break;

            case 7: // Wait to arrive at sample 2, then grab
                if (!follower.isBusy()) {
                    // Grab sample 2
                    setPathState(8);
                }
                break;

            case 8: // Drive to score sample 2
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    follower.followPath(scoreSample2, true);
                    setPathState(9);
                }
                break;

            case 9: // Wait to arrive, then score sample 2
                if (!follower.isBusy()) {
                    // Score sample 2
                    setPathState(10);
                }
                break;

            case 10: // Drive to grab sample 3
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    follower.followPath(grabSample3, true);
                    setPathState(11);
                }
                break;

            case 11: // Wait to arrive at sample 3, then grab
                if (!follower.isBusy()) {
                    // Grab sample 3
                    setPathState(12);
                }
                break;

            case 12: // Drive to score sample 3
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    follower.followPath(scoreSample3, true);
                    setPathState(13);
                }
                break;

            case 13: // Wait to arrive, then score sample 3
                if (!follower.isBusy()) {
                    // Score sample 3
                    setPathState(14);
                }
                break;

            case 14: // Park
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    follower.followPath(park, true);
                    setPathState(15);
                }
                break;

            case 15: // Done
                // Autonomous complete
                break;
        }
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        pathState = 0;

        follower = new Follower(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        buildPaths();
    }

    @Override
    public void start() {
        pathTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        // Draw robot on FTC Dashboard field overlay
        Pose currentPose = follower.getPose();
        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();

        fieldOverlay.setStroke("#3F51B5");
        fieldOverlay.setStrokeWidth(1);
        fieldOverlay.strokeCircle(currentPose.getX(), currentPose.getY(), 9);
        double arrowX = currentPose.getX() + 12 * Math.cos(currentPose.getHeading());
        double arrowY = currentPose.getY() + 12 * Math.sin(currentPose.getHeading());
        fieldOverlay.strokeLine(currentPose.getX(), currentPose.getY(), arrowX, arrowY);

        FtcDashboard.getInstance().sendTelemetryPacket(packet);

        telemetry.addData("Path State", pathState);
        telemetry.addData("X", currentPose.getX());
        telemetry.addData("Y", currentPose.getY());
        telemetry.addData("Heading (deg)", Math.toDegrees(currentPose.getHeading()));
        telemetry.addData("Busy", follower.isBusy());
        telemetry.update();
    }
}`,

"pedro-teleop": `package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

@Config
@TeleOp(name = "Pedro TeleOp", group = "TeleOp")
public class PedroTeleOpExample extends OpMode {

    public static double NORMAL_SPEED = 1.0;
    public static double SLOW_SPEED = 0.4;

    private Follower follower;
    private double speedMultiplier = NORMAL_SPEED;
    private boolean slowModeActive = false;
    private boolean previousAState = false;

    private List<LynxModule> allHubs;
    private ElapsedTime loopTimer;

    @Override
    public void init() {
        // Set up manual bulk reads for performance
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        follower = new Follower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        loopTimer = new ElapsedTime();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        loopTimer.reset();
    }

    @Override
    public void loop() {
        // Clear bulk cache at the start of each loop
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

        // Toggle slow mode with gamepad1.a (rising edge detection)
        boolean currentAState = gamepad1.a;
        if (currentAState && !previousAState) {
            slowModeActive = !slowModeActive;
        }
        previousAState = currentAState;

        // Read speed multiplier from dashboard-tunable statics at point of use
        speedMultiplier = slowModeActive ? SLOW_SPEED : NORMAL_SPEED;

        // Drive using Pedro's built-in TeleOp drive
        follower.setTeleOpMovementVectors(
                -gamepad1.left_stick_y * speedMultiplier,
                -gamepad1.left_stick_x * speedMultiplier,
                -gamepad1.right_stick_x * speedMultiplier,
                true
        );
        follower.update();

        // Telemetry
        Pose currentPose = follower.getPose();
        telemetry.addData("--- Drive Info ---", "");
        telemetry.addData("Slow Mode", slowModeActive ? "ON" : "OFF");
        telemetry.addData("Speed Multiplier", "%.2f", speedMultiplier);
        telemetry.addData("--- Pose ---", "");
        telemetry.addData("X", "%.2f", currentPose.getX());
        telemetry.addData("Y", "%.2f", currentPose.getY());
        telemetry.addData("Heading (deg)", "%.2f", Math.toDegrees(currentPose.getHeading()));
        telemetry.addData("--- Performance ---", "");
        telemetry.addData("Loop Time (ms)", "%.1f", loopTimer.milliseconds());
        telemetry.update();

        loopTimer.reset();
    }
}`,

"pedro-constants": `package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.localization.constants.PinpointConstants;
import com.pedropathing.pathgen.MathFunctions;
import com.pedropathing.pathgen.PathConstraints;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {

    static {
        // --- Follower PID and Drive Constants ---
        FollowerConstants.mass = 13.0; // robot mass in kg

        FollowerConstants.translationalPIDFCoefficients.setCoefficients(0.15, 0, 0.01, 0);
        FollowerConstants.headingPIDFCoefficients.setCoefficients(2.0, 0, 0.1, 0);
        FollowerConstants.drivePIDFCoefficients.setCoefficients(0.01, 0, 0.0002, 0.6);

        FollowerConstants.zeroPowerAcceleration = -30;
        FollowerConstants.centripetalScaling = 0.0005;

        FollowerConstants.useSecondaryTranslationalPID = false;
        FollowerConstants.useSecondaryHeadingPID = false;
        FollowerConstants.useSecondaryDrivePID = false;

        // --- Mecanum Motor Configuration ---
        FollowerConstants.leftFrontMotorName = "leftFront";
        FollowerConstants.leftRearMotorName = "leftRear";
        FollowerConstants.rightFrontMotorName = "rightFront";
        FollowerConstants.rightRearMotorName = "rightRear";

        FollowerConstants.leftFrontMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.leftRearMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.rightFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
        FollowerConstants.rightRearMotorDirection = DcMotorSimple.Direction.FORWARD;

        // --- Pinpoint Localizer Configuration ---
        PinpointConstants.forwardY = 5.5; // forward encoder Y offset in inches
        PinpointConstants.strafeX = -3.0; // strafe encoder X offset in inches
        PinpointConstants.distanceUnit = DistanceUnit.INCH;
        PinpointConstants.hardwareMapName = "pinpoint";
        PinpointConstants.useYawScalar = false;
        PinpointConstants.yawScalar = 1.0;
        PinpointConstants.useCustomEncoderResolution = false;
        PinpointConstants.encoderResolution = PinpointConstants.GoBildaOdometryPods.goBILDA_4_BAR_POD;
        PinpointConstants.forwardEncoderDirection = PinpointConstants.EncoderDirection.FORWARD;
        PinpointConstants.strafeEncoderDirection = PinpointConstants.EncoderDirection.FORWARD;

        // --- Path Constraints ---
        PathConstraints.maxVelocity = 50;
        PathConstraints.maxAcceleration = 50;
        PathConstraints.maxAngularVelocity = Math.toRadians(300);
        PathConstraints.maxAngularAcceleration = Math.toRadians(300);
    }

    /**
     * Creates and returns a configured Follower instance.
     * Call this from your OpMode's init() method.
     *
     * @param hardwareMap the hardware map from the OpMode
     * @return a configured Follower
     */
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new Follower(hardwareMap);
    }
}`,

"dashboard-config": `package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Demonstrates @Config usage with FTC Dashboard.
 *
 * CRITICAL: Always read static fields at the POINT OF USE so that
 * dashboard edits take effect immediately. Never cache them in
 * local variables during init().
 */
@Config
@TeleOp(name = "Dashboard Config Demo", group = "Examples")
public class DashboardConfigExample extends OpMode {

    // --- Primitive types: editable from the dashboard ---
    public static double POWER = 0.5;
    public static int TARGET_POSITION = 1000;
    public static boolean USE_PID = true;
    public static String ALLIANCE_COLOR = "RED";

    // --- Enum: shows a dropdown in the dashboard ---
    public enum DriveMode {
        FIELD_CENTRIC,
        ROBOT_CENTRIC,
        TANK
    }
    public static DriveMode DRIVE_MODE = DriveMode.FIELD_CENTRIC;

    // --- Nested object: each field editable in the dashboard ---
    public static class PIDCoefficients {
        public double kP = 0.05;
        public double kI = 0.0;
        public double kD = 0.01;
        public double kF = 0.0;
    }
    public static PIDCoefficients pidCoeffs = new PIDCoefficients();

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Status", "Initialized - edit values on dashboard!");
        telemetry.update();
    }

    @Override
    public void loop() {
        // =====================================================
        // CORRECT: Read statics at the point of use.
        // Dashboard changes are reflected IMMEDIATELY each loop.
        // =====================================================
        double currentPower = POWER;           // fresh every loop
        int currentTarget = TARGET_POSITION;   // fresh every loop
        boolean usePid = USE_PID;              // fresh every loop

        // PID coefficients are also read fresh because pidCoeffs
        // is a static reference and its fields are public.
        double p = pidCoeffs.kP;
        double i = pidCoeffs.kI;
        double d = pidCoeffs.kD;
        double f = pidCoeffs.kF;

        // Use the values ...
        telemetry.addData("--- Current Values ---", "");
        telemetry.addData("Power", "%.2f", currentPower);
        telemetry.addData("Target Position", currentTarget);
        telemetry.addData("Use PID", usePid);
        telemetry.addData("Alliance", ALLIANCE_COLOR);
        telemetry.addData("Drive Mode", DRIVE_MODE);
        telemetry.addData("--- PID ---", "");
        telemetry.addData("kP", "%.4f", p);
        telemetry.addData("kI", "%.4f", i);
        telemetry.addData("kD", "%.4f", d);
        telemetry.addData("kF", "%.4f", f);

        // =====================================================
        // WRONG: Do NOT cache statics in instance fields in init()!
        //
        //   private double cachedPower;
        //
        //   public void init() {
        //       cachedPower = POWER;  // BUG: copies the value once!
        //   }
        //
        //   public void loop() {
        //       // cachedPower never changes even if you edit POWER
        //       // on the dashboard, because Java copied the double
        //       // value, not a reference to the static field.
        //       motor.setPower(cachedPower); // stale!
        //   }
        //
        // This is a "copy semantics" bug. Primitives are copied
        // by value, so the local variable is disconnected from
        // the static field after assignment.
        // =====================================================

        telemetry.update();
    }
}`,

"bulk-reads": `package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

/**
 * Demonstrates optimised hardware reads using MANUAL bulk caching.
 *
 * Without bulk caching every call to getPosition(), getCurrent(), etc.
 * triggers a separate USB transaction (~3 ms each). With MANUAL mode
 * a single clearBulkCache() call at the top of the loop fetches ALL
 * sensor data in one transaction, and subsequent reads come from cache.
 */
@Config
@TeleOp(name = "Bulk Reads Demo", group = "Examples")
public class BulkReadsExample extends OpMode {

    public static double SERVO_POS_A = 0.0;
    public static double SERVO_POS_B = 1.0;

    private List<LynxModule> allHubs;

    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backLeft;
    private DcMotorEx backRight;

    private Servo clawServo;

    private ElapsedTime loopTimer;
    private double maxLoopTime = 0;

    @Override
    public void init() {
        // --- Enable MANUAL bulk caching on every hub ---
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        // --- Motors ---
        frontLeft  = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight  = hardwareMap.get(DcMotorEx.class, "backRight");

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // --- Servos ---
        clawServo = hardwareMap.get(Servo.class, "claw");

        // --- Telemetry ---
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        loopTimer = new ElapsedTime();

        telemetry.addData("Status", "Initialized with MANUAL bulk reads");
        telemetry.update();
    }

    @Override
    public void loop() {
        // *** Clear bulk cache ONCE at the top of each loop ***
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

        loopTimer.reset();

        // --- Drive: simple tank drive for demo ---
        double drive = -gamepad1.left_stick_y;
        double turn  =  gamepad1.right_stick_x;

        frontLeft.setPower(drive + turn);
        backLeft.setPower(drive + turn);
        frontRight.setPower(drive - turn);
        backRight.setPower(drive - turn);

        // --- Servo: toggle with bumpers ---
        if (gamepad1.left_bumper) {
            clawServo.setPosition(SERVO_POS_A);
        } else if (gamepad1.right_bumper) {
            clawServo.setPosition(SERVO_POS_B);
        }

        // --- Read encoder positions (all from cache — fast!) ---
        int flPos = frontLeft.getCurrentPosition();
        int frPos = frontRight.getCurrentPosition();
        int blPos = backLeft.getCurrentPosition();
        int brPos = backRight.getCurrentPosition();

        double elapsed = loopTimer.milliseconds();
        if (elapsed > maxLoopTime) {
            maxLoopTime = elapsed;
        }

        // --- Telemetry ---
        telemetry.addData("--- Encoders ---", "");
        telemetry.addData("FL", flPos);
        telemetry.addData("FR", frPos);
        telemetry.addData("BL", blPos);
        telemetry.addData("BR", brPos);
        telemetry.addData("--- Servo ---", "");
        telemetry.addData("Claw Position", "%.2f", clawServo.getPosition());
        telemetry.addData("--- Performance ---", "");
        telemetry.addData("Loop Time (ms)", "%.2f", elapsed);
        telemetry.addData("Max Loop Time (ms)", "%.2f", maxLoopTime);
        telemetry.update();
    }
}`,

"subsystem": `package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Example hardware subsystem for a linear-slide lift with a
 * claw servo. Designed to be instantiated from an OpMode.
 *
 * Usage:
 *   LiftSubsystem lift = new LiftSubsystem(hardwareMap);
 *   lift.goToPosition(LiftSubsystem.HIGH_POSITION);
 *   lift.setClaw(LiftSubsystem.CLAW_OPEN);
 *   lift.update();
 */
@Config
public class LiftSubsystem {

    // --- Dashboard-tunable positions ---
    // Always read these statics at the POINT OF USE so dashboard
    // edits are reflected immediately.
    public static int HIGH_POSITION = 2800;
    public static int MID_POSITION = 1400;
    public static int LOW_POSITION = 400;
    public static int HOME_POSITION = 0;
    public static int POSITION_TOLERANCE = 15;

    public static double LIFT_POWER = 0.8;
    public static double HOLD_POWER = 0.15;

    public static double CLAW_OPEN = 0.6;
    public static double CLAW_CLOSED = 0.15;

    public static double kP = 0.005;
    public static double kF = 0.1;

    private final DcMotorEx liftMotor;
    private final Servo clawServo;

    private int targetPosition = 0;

    /**
     * Initializes the subsystem from the hardware map.
     *
     * @param hardwareMap the OpMode's hardware map
     */
    public LiftSubsystem(HardwareMap hardwareMap) {
        liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        clawServo = hardwareMap.get(Servo.class, "clawServo");
        clawServo.setPosition(CLAW_CLOSED);
    }

    /**
     * Sets the target encoder position for the lift.
     * Use the public static constants (HIGH_POSITION, etc.) which
     * are editable from the dashboard.
     */
    public void goToPosition(int position) {
        targetPosition = position;
    }

    /**
     * Software PID update — call this every loop iteration.
     * Reads kP and kF from statics at point of use so dashboard
     * changes take effect immediately.
     */
    public void update() {
        int currentPosition = liftMotor.getCurrentPosition();
        int error = targetPosition - currentPosition;

        // Read tunable coefficients fresh each loop
        double power = error * kP + Math.signum(error) * kF;
        power = Math.max(-LIFT_POWER, Math.min(LIFT_POWER, power));

        // Apply a small holding power when near target to resist gravity
        if (Math.abs(error) < POSITION_TOLERANCE && targetPosition > 50) {
            power = HOLD_POWER;
        }

        liftMotor.setPower(power);
    }

    /** Opens or closes the claw. */
    public void setClaw(double position) {
        clawServo.setPosition(position);
    }

    /** @return true if the lift is within tolerance of its target. */
    public boolean isAtTarget() {
        return Math.abs(liftMotor.getCurrentPosition() - targetPosition) < POSITION_TOLERANCE;
    }

    /** @return the current encoder position of the lift motor. */
    public int getCurrentPosition() {
        return liftMotor.getCurrentPosition();
    }

    /** @return the active target position. */
    public int getTargetPosition() {
        return targetPosition;
    }

    /** Sends subsystem telemetry to the driver station / dashboard. */
    public void outputTelemetry(Telemetry telemetry) {
        int currentPos = liftMotor.getCurrentPosition();
        telemetry.addData("--- Lift ---", "");
        telemetry.addData("Target", targetPosition);
        telemetry.addData("Current", currentPos);
        telemetry.addData("Error", targetPosition - currentPos);
        telemetry.addData("At Target", isAtTarget());
        telemetry.addData("Motor Power", "%.2f", liftMotor.getPower());
        telemetry.addData("Claw Pos", "%.2f", clawServo.getPosition());
    }
}`,

"pid-tuning": `package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

/**
 * PID tuning OpMode for a single motor with encoder feedback.
 *
 * Open the FTC Dashboard graph panel and add:
 *   - target
 *   - position
 *   - error
 *   - output
 *
 * Then adjust kP, kI, kD, kF live to tune the controller.
 */
@Config
@TeleOp(name = "PID Tuner", group = "Tuning")
public class PIDTuningExample extends OpMode {

    // --- Dashboard-tunable PID coefficients ---
    public static double kP = 0.01;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kF = 0.0;

    // --- Dashboard-tunable target ---
    public static int TARGET = 1000;
    public static double MAX_OUTPUT = 0.8;

    private DcMotorEx motor;

    private double integralSum = 0;
    private double lastError = 0;
    private ElapsedTime timer;

    private List<LynxModule> allHubs;

    @Override
    public void init() {
        // Bulk reads for performance
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        motor = hardwareMap.get(DcMotorEx.class, "liftMotor");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        timer = new ElapsedTime();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Status", "PID Tuner ready — adjust kP/kI/kD/kF on dashboard");
        telemetry.update();
    }

    @Override
    public void start() {
        timer.reset();
        integralSum = 0;
        lastError = 0;
    }

    @Override
    public void loop() {
        // Clear bulk cache
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

        // Read coefficients from dashboard statics at point of use
        double p = kP;
        double i = kI;
        double d = kD;
        double f = kF;
        int target = TARGET;

        int currentPosition = motor.getCurrentPosition();
        double error = target - currentPosition;

        double dt = timer.seconds();
        timer.reset();

        // Proportional
        double pTerm = p * error;

        // Integral (with windup guard)
        integralSum += error * dt;
        integralSum = Math.max(-1000, Math.min(1000, integralSum));
        double iTerm = i * integralSum;

        // Derivative
        double derivative = (dt > 0) ? (error - lastError) / dt : 0;
        double dTerm = d * derivative;
        lastError = error;

        // Feedforward
        double fTerm = f * Math.signum(error);

        // Total output, clamped
        double output = pTerm + iTerm + dTerm + fTerm;
        output = Math.max(-MAX_OUTPUT, Math.min(MAX_OUTPUT, output));

        motor.setPower(output);

        // Telemetry — these keys are graphable in the dashboard
        telemetry.addData("target", target);
        telemetry.addData("position", currentPosition);
        telemetry.addData("error", error);
        telemetry.addData("output", output);
        telemetry.addData("--- PID Terms ---", "");
        telemetry.addData("P term", "%.4f", pTerm);
        telemetry.addData("I term", "%.4f", iTerm);
        telemetry.addData("D term", "%.4f", dTerm);
        telemetry.addData("F term", "%.4f", fTerm);
        telemetry.addData("--- Coefficients ---", "");
        telemetry.addData("kP", p);
        telemetry.addData("kI", i);
        telemetry.addData("kD", d);
        telemetry.addData("kF", f);
        telemetry.update();
    }

    @Override
    public void stop() {
        motor.setPower(0);
    }
}`,

"vision-pipeline": `package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/**
 * Demonstrates VisionPortal with an AprilTag processor.
 * Streams the camera feed to the FTC Dashboard for remote viewing.
 */
@TeleOp(name = "Vision AprilTag Demo", group = "Vision")
public class VisionPipelineExample extends OpMode {

    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // --- AprilTag Processor ---
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .build();

        // --- Vision Portal ---
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTagProcessor)
                .enableLiveView(true)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();

        // Stream camera to FTC Dashboard
        FtcDashboard.getInstance().startCameraStream(visionPortal, 0);

        telemetry.addData("Status", "VisionPortal initialized");
        telemetry.addData("Camera State", visionPortal.getCameraState());
        telemetry.update();
    }

    @Override
    public void init_loop() {
        telemetry.addData("Camera State", visionPortal.getCameraState());

        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
        telemetry.addData("# Detections", detections.size());
        for (AprilTagDetection detection : detections) {
            telemetry.addLine(String.format("\\nTag ID %d (%s)", detection.id,
                    detection.metadata != null ? detection.metadata.name : "Unknown"));
        }
        telemetry.update();
    }

    @Override
    public void loop() {
        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();

        telemetry.addData("# AprilTags Detected", detections.size());

        for (AprilTagDetection detection : detections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\\n==== Tag ID %d \\\"%s\\\" ====",
                        detection.id, detection.metadata.name));
            } else {
                telemetry.addLine(String.format("\\n==== Tag ID %d (Unknown) ====",
                        detection.id));
            }

            telemetry.addLine(String.format("XYZ: %.1f, %.1f, %.1f  (inches)",
                    detection.ftcPose.x,
                    detection.ftcPose.y,
                    detection.ftcPose.z));
            telemetry.addLine(String.format("PRY: %.1f, %.1f, %.1f  (degrees)",
                    detection.ftcPose.pitch,
                    detection.ftcPose.roll,
                    detection.ftcPose.yaw));
            telemetry.addLine(String.format("Range: %.1f in", detection.ftcPose.range));
            telemetry.addLine(String.format("Bearing: %.1f deg", detection.ftcPose.bearing));
            telemetry.addLine(String.format("Elevation: %.1f deg", detection.ftcPose.elevation));
        }

        if (detections.isEmpty()) {
            telemetry.addData("Info", "No AprilTags detected");
        }

        telemetry.update();
    }

    @Override
    public void stop() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}`,

"custom-pid-drive": `package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Encoder-based autonomous driving with PID heading correction.
 * No pathing library required — uses the built-in motor encoders
 * and IMU for straight-line driving.
 */
@Config
@Autonomous(name = "Custom PID Drive", group = "Auto")
public class CustomPIDDriveExample extends LinearOpMode {

    // --- Dashboard-tunable PID ---
    public static double HEADING_KP = 0.02;
    public static double HEADING_KI = 0.0;
    public static double HEADING_KD = 0.005;

    // --- Dashboard-tunable drive parameters ---
    public static double DRIVE_SPEED = 0.4;
    public static double TURN_SPEED = 0.3;
    public static double COUNTS_PER_INCH = 537.7 / (4.0 * Math.PI); // goBILDA 312 RPM, 4" wheels
    public static double DRIVE_TIMEOUT = 5.0;

    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private IMU imu;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // --- Motor setup ---
        frontLeft  = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight  = hardwareMap.get(DcMotorEx.class, "backRight");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // --- IMU setup ---
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters imuParams = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(imuParams);
        imu.resetYaw();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        // --- Autonomous routine ---
        encoderDrive(DRIVE_SPEED, 24, 24, DRIVE_TIMEOUT);   // Forward 24 in
        encoderDrive(TURN_SPEED,  12, -12, DRIVE_TIMEOUT);  // Turn right
        encoderDrive(DRIVE_SPEED, 18, 18, DRIVE_TIMEOUT);   // Forward 18 in
        encoderDrive(DRIVE_SPEED, -12, -12, DRIVE_TIMEOUT); // Reverse 12 in

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }

    /**
     * Drive a specified distance using encoder counts with PID
     * heading correction from the IMU.
     *
     * @param speed       max motor power (0..1)
     * @param leftInches  distance for left side (negative = reverse)
     * @param rightInches distance for right side (negative = reverse)
     * @param timeout     max seconds before aborting
     */
    public void encoderDrive(double speed, double leftInches, double rightInches, double timeout) {
        if (!opModeIsActive()) return;

        // Read the dashboard-tunable constants fresh
        double countsPerInch = COUNTS_PER_INCH;

        int leftTarget  = frontLeft.getCurrentPosition()  + (int)(leftInches  * countsPerInch);
        int rightTarget = frontRight.getCurrentPosition() + (int)(rightInches * countsPerInch);

        frontLeft.setTargetPosition(leftTarget);
        backLeft.setTargetPosition(leftTarget);
        frontRight.setTargetPosition(rightTarget);
        backRight.setTargetPosition(rightTarget);

        setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);

        double targetHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double headingIntegral = 0;
        double lastHeadingError = 0;

        ElapsedTime runtime = new ElapsedTime();
        ElapsedTime pidTimer = new ElapsedTime();

        // Set initial power
        setAllMotorPower(Math.abs(speed));

        while (opModeIsActive()
                && runtime.seconds() < timeout
                && (frontLeft.isBusy() || frontRight.isBusy())) {

            double dt = pidTimer.seconds();
            pidTimer.reset();

            // PID heading correction — read coefficients fresh each iteration
            double currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double headingError = targetHeading - currentHeading;

            // Normalize to -180..180
            while (headingError > 180) headingError -= 360;
            while (headingError < -180) headingError += 360;

            headingIntegral += headingError * dt;
            headingIntegral = Range.clip(headingIntegral, -50, 50);

            double headingDerivative = (dt > 0) ? (headingError - lastHeadingError) / dt : 0;
            lastHeadingError = headingError;

            double correction = (HEADING_KP * headingError)
                              + (HEADING_KI * headingIntegral)
                              + (HEADING_KD * headingDerivative);

            double leftPower  = Range.clip(speed + correction, -1.0, 1.0);
            double rightPower = Range.clip(speed - correction, -1.0, 1.0);

            frontLeft.setPower(Math.abs(leftPower));
            backLeft.setPower(Math.abs(leftPower));
            frontRight.setPower(Math.abs(rightPower));
            backRight.setPower(Math.abs(rightPower));

            telemetry.addData("Target L/R", "%7d / %7d", leftTarget, rightTarget);
            telemetry.addData("Current L/R", "%7d / %7d",
                    frontLeft.getCurrentPosition(), frontRight.getCurrentPosition());
            telemetry.addData("Heading Target", "%.1f", targetHeading);
            telemetry.addData("Heading Current", "%.1f", currentHeading);
            telemetry.addData("Heading Error", "%.2f", headingError);
            telemetry.addData("Correction", "%.3f", correction);
            telemetry.addData("Power L/R", "%.2f / %.2f", leftPower, rightPower);
            telemetry.update();
        }

        setAllMotorPower(0);
        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void setMotorMode(DcMotor.RunMode mode) {
        frontLeft.setMode(mode);
        frontRight.setMode(mode);
        backLeft.setMode(mode);
        backRight.setMode(mode);
    }

    private void setAllMotorPower(double power) {
        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);
    }
}`,

"field-centric-drive": `package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.List;

/**
 * Field-centric mecanum TeleOp with IMU heading reference.
 *
 * The left stick controls translation relative to the FIELD,
 * not the robot. The right stick controls rotation. Press
 * gamepad1.back to reset the heading reference so "forward"
 * always means toward the opposing alliance wall.
 */
@Config
@TeleOp(name = "Field-Centric Mecanum", group = "TeleOp")
public class FieldCentricDriveExample extends OpMode {

    public static double NORMAL_SPEED = 1.0;
    public static double SLOW_SPEED = 0.35;

    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private IMU imu;

    private List<LynxModule> allHubs;
    private ElapsedTime loopTimer;

    private boolean slowModeActive = false;
    private boolean previousAState = false;

    @Override
    public void init() {
        // --- Manual bulk reads ---
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        // --- Motors ---
        frontLeft  = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight  = hardwareMap.get(DcMotorEx.class, "backRight");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // --- IMU ---
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters imuParams = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(imuParams);
        imu.resetYaw();

        // --- Telemetry ---
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        loopTimer = new ElapsedTime();

        telemetry.addData("Status", "Initialized — press Back to reset heading");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Clear bulk cache once per loop
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

        loopTimer.reset();

        // --- Reset heading reference ---
        if (gamepad1.back) {
            imu.resetYaw();
        }

        // --- Slow mode toggle (rising edge on A) ---
        boolean currentAState = gamepad1.a;
        if (currentAState && !previousAState) {
            slowModeActive = !slowModeActive;
        }
        previousAState = currentAState;

        // Read speed from dashboard-tunable statics at point of use
        double speedMultiplier = slowModeActive ? SLOW_SPEED : NORMAL_SPEED;

        // --- Read gamepad inputs ---
        double y  = -gamepad1.left_stick_y;  // forward is positive
        double x  =  gamepad1.left_stick_x;  // right is positive
        double rx =  gamepad1.right_stick_x; // clockwise is positive

        // --- Get heading from IMU ---
        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // --- Field-centric rotation matrix ---
        double rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
        double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);

        // Counteract imperfect strafing (optional 1.1 multiplier)
        rotX *= 1.1;

        // --- Mecanum kinematics ---
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1.0);
        double flPower = (rotY + rotX + rx) / denominator;
        double blPower = (rotY - rotX + rx) / denominator;
        double frPower = (rotY - rotX - rx) / denominator;
        double brPower = (rotY + rotX - rx) / denominator;

        // --- Apply speed multiplier and set power ---
        frontLeft.setPower(flPower * speedMultiplier);
        backLeft.setPower(blPower * speedMultiplier);
        frontRight.setPower(frPower * speedMultiplier);
        backRight.setPower(brPower * speedMultiplier);

        // --- Telemetry ---
        double loopMs = loopTimer.milliseconds();
        telemetry.addData("--- Drive ---", "");
        telemetry.addData("Mode", "Field-Centric");
        telemetry.addData("Slow Mode", slowModeActive ? "ON" : "OFF");
        telemetry.addData("Speed", "%.2f", speedMultiplier);
        telemetry.addData("--- Heading ---", "");
        telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(heading));
        telemetry.addData("--- Motor Powers ---", "");
        telemetry.addData("FL / FR", "%.2f / %.2f", flPower * speedMultiplier, frPower * speedMultiplier);
        telemetry.addData("BL / BR", "%.2f / %.2f", blPower * speedMultiplier, brPower * speedMultiplier);
        telemetry.addData("--- Performance ---", "");
        telemetry.addData("Loop Time (ms)", "%.2f", loopMs);
        telemetry.update();
    }
}`,

"command-teleop": `package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.custom.DefaultDriveCommand;
import org.firstinspires.ftc.teamcode.commands.custom.LiftToPositionCommand;
import org.firstinspires.ftc.teamcode.commands.group.ScoreHighBasketCommand;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;

/**
 * Command-based TeleOp using SolversLib.
 *
 * Controls:
 *   Driver (gamepad1):
 *     Left stick   = translate (field-centric or robot-centric)
 *     Right stick X = rotate
 *     A            = toggle slow mode
 *
 *   Operator (gamepad2):
 *     A       = toggle claw open/close
 *     Y       = score high basket macro (lift → open → lower)
 *     DPAD_UP = lift to high basket position
 *     DPAD_DN = lift to home position
 *     DPAD_LT = lift to low basket position
 *     LB      = manual lift down (while held)
 *     RB      = manual lift up (while held)
 */
@Config
@TeleOp(name = "Command TeleOp", group = "Competition")
public class CommandTeleOpExample extends CommandOpMode {

    public static double SLOW_MULTIPLIER = 0.35;
    public static double NORMAL_MULTIPLIER = 1.0;

    private boolean slowMode = false;

    @Override
    public void initialize() {
        // --- Bulk reads via scheduler ---
        CommandScheduler.getInstance().setBulkCacheMode(LynxModule.BulkCachingMode.MANUAL);

        // --- Telemetry to Dashboard ---
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // --- Gamepads ---
        GamepadEx driverGp = new GamepadEx(gamepad1);
        GamepadEx operatorGp = new GamepadEx(gamepad2);

        // --- Subsystems ---
        DriveSubsystem drive = new DriveSubsystem(hardwareMap);
        LiftSubsystem lift = new LiftSubsystem(hardwareMap);
        ClawSubsystem claw = new ClawSubsystem(hardwareMap);

        // --- Default Drive Command ---
        drive.setDefaultCommand(new DefaultDriveCommand(
            drive,
            () -> -driverGp.getLeftY()  * (slowMode ? SLOW_MULTIPLIER : NORMAL_MULTIPLIER),
            () ->  driverGp.getLeftX()  * (slowMode ? SLOW_MULTIPLIER : NORMAL_MULTIPLIER),
            () ->  driverGp.getRightX() * (slowMode ? SLOW_MULTIPLIER : NORMAL_MULTIPLIER)
        ));

        // --- Driver Bindings ---
        driverGp.getGamepadButton(GamepadKeys.Button.A)
            .whenPressed(new InstantCommand(() -> slowMode = !slowMode));

        // --- Operator Bindings ---

        // Claw toggle
        operatorGp.getGamepadButton(GamepadKeys.Button.A)
            .whenPressed(new InstantCommand(claw::toggle, claw));

        // Score macro: lift high → pause → open claw → pause → lower
        operatorGp.getGamepadButton(GamepadKeys.Button.Y)
            .whenPressed(new ScoreHighBasketCommand(lift, claw));

        // Lift presets
        operatorGp.getGamepadButton(GamepadKeys.Button.DPAD_UP)
            .whenPressed(new LiftToPositionCommand(lift, LiftSubsystem.HIGH_BASKET));

        operatorGp.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
            .whenPressed(new LiftToPositionCommand(lift, LiftSubsystem.HOME));

        operatorGp.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
            .whenPressed(new LiftToPositionCommand(lift, LiftSubsystem.LOW_BASKET));

        // Manual lift control (while held)
        operatorGp.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
            .whileHeld(new RunCommand(() -> lift.setManualPower(0.6), lift))
            .whenReleased(new InstantCommand(lift::exitManual, lift));

        operatorGp.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
            .whileHeld(new RunCommand(() -> lift.setManualPower(-0.4), lift))
            .whenReleased(new InstantCommand(lift::exitManual, lift));

        telemetry.addData("Status", "Command TeleOp Initialized");
        telemetry.update();
    }

    @Override
    public void run() {
        super.run(); // MUST call super to tick the scheduler

        telemetry.addData("Slow Mode", slowMode ? "ON" : "OFF");
        telemetry.update();
    }
}`,

"command-auto": `package org.firstinspires.ftc.teamcode.opmode.auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.custom.FollowPathCommand;
import org.firstinspires.ftc.teamcode.commands.custom.LiftToPositionCommand;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;

/**
 * Command-based autonomous using SolversLib + Pedro Pathing.
 *
 * This replaces the traditional FSM/switch-case pattern with a
 * declarative SequentialCommandGroup. Each step is a command,
 * and parallel actions use .alongWith().
 */
@Config
@Autonomous(name = "Command Auto - Left", group = "Auto")
public class CommandAutoExample extends CommandOpMode {

    // Dashboard-tunable poses
    public static double START_X = 9.0, START_Y = 60.0, START_HEADING = 0;
    public static double SCORE_X = 38.0, SCORE_Y = 68.0;
    public static double SAMPLE1_X = 37.0, SAMPLE1_Y = 30.0;
    public static double PARK_X = 10.0, PARK_Y = 10.0;

    private Follower follower;
    private LiftSubsystem lift;
    private ClawSubsystem claw;

    @Override
    public void initialize() {
        // Bulk reads
        CommandScheduler.getInstance().setBulkCacheMode(LynxModule.BulkCachingMode.MANUAL);

        // Telemetry
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Subsystems
        follower = new Follower(hardwareMap);
        lift = new LiftSubsystem(hardwareMap);
        claw = new ClawSubsystem(hardwareMap);

        // Starting pose
        Pose startPose = new Pose(START_X, START_Y, Math.toRadians(START_HEADING));
        follower.setStartingPose(startPose);

        // Build paths
        PathChain scorePreload = follower.pathBuilder()
            .addPath(new BezierLine(
                new Point(START_X, START_Y, Point.CARTESIAN),
                new Point(SCORE_X, SCORE_Y, Point.CARTESIAN)))
            .setLinearHeadingInterpolation(Math.toRadians(START_HEADING), 0)
            .build();

        PathChain grabSample1 = follower.pathBuilder()
            .addPath(new BezierLine(
                new Point(SCORE_X, SCORE_Y, Point.CARTESIAN),
                new Point(SAMPLE1_X, SAMPLE1_Y, Point.CARTESIAN)))
            .setLinearHeadingInterpolation(0, 0)
            .build();

        PathChain scoreSample1 = follower.pathBuilder()
            .addPath(new BezierLine(
                new Point(SAMPLE1_X, SAMPLE1_Y, Point.CARTESIAN),
                new Point(SCORE_X, SCORE_Y, Point.CARTESIAN)))
            .setLinearHeadingInterpolation(0, 0)
            .build();

        PathChain park = follower.pathBuilder()
            .addPath(new BezierLine(
                new Point(SCORE_X, SCORE_Y, Point.CARTESIAN),
                new Point(PARK_X, PARK_Y, Point.CARTESIAN)))
            .setLinearHeadingInterpolation(0, 0)
            .build();

        // Schedule the full autonomous as one command group
        schedule(new SequentialCommandGroup(
            // === Score preloaded sample ===
            // Drive to scoring position WHILE raising the lift
            new FollowPathCommand(follower, scorePreload, true)
                .alongWith(new LiftToPositionCommand(lift, LiftSubsystem.HIGH_BASKET)),
            // Wait for lift to finish reaching target
            new WaitUntilCommand(lift::atTarget),
            // Open claw to release
            new InstantCommand(claw::open, claw),
            new WaitCommand(300),

            // === Grab sample 1 ===
            // Drive to sample WHILE lowering lift to intake height
            new FollowPathCommand(follower, grabSample1, true)
                .alongWith(new LiftToPositionCommand(lift, LiftSubsystem.INTAKE)),
            // Close claw to grab
            new InstantCommand(claw::close, claw),
            new WaitCommand(200),

            // === Score sample 1 ===
            new FollowPathCommand(follower, scoreSample1, true)
                .alongWith(new LiftToPositionCommand(lift, LiftSubsystem.HIGH_BASKET)),
            new WaitUntilCommand(lift::atTarget),
            new InstantCommand(claw::open, claw),
            new WaitCommand(300),

            // === Park ===
            new FollowPathCommand(follower, park, false)
                .alongWith(new LiftToPositionCommand(lift, LiftSubsystem.HOME))
        ));

        telemetry.addData("Status", "Command Auto Initialized");
        telemetry.update();
    }

    @Override
    public void run() {
        super.run();

        // Telemetry
        Pose pose = follower.getPose();
        telemetry.addData("X", "%.1f", pose.getX());
        telemetry.addData("Y", "%.1f", pose.getY());
        telemetry.addData("Heading", "%.1f", Math.toDegrees(pose.getHeading()));
        telemetry.addData("Lift Pos", lift.getPosition());
        telemetry.addData("Lift Target", lift.getTarget());
        telemetry.update();
    }
}`,

"command-subsystem": `package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Command-based lift subsystem using SolversLib's SubsystemBase.
 *
 * PID control runs continuously in periodic() — commands just set the target.
 * All tunable constants are @Config public static for FTC Dashboard.
 *
 * Usage:
 *   LiftSubsystem lift = new LiftSubsystem(hardwareMap);
 *   // Register is called in constructor
 *
 *   // In a command:
 *   lift.setTarget(LiftSubsystem.HIGH_BASKET);
 *   // periodic() handles the PID automatically each scheduler tick
 */
@Config
public class LiftSubsystem extends SubsystemBase {

    // --- Dashboard-tunable PID coefficients ---
    public static double kP = 0.005;
    public static double kI = 0.0;
    public static double kD = 0.001;
    public static double kF = 0.12;    // gravity feedforward for linear slide

    // --- Dashboard-tunable position presets ---
    public static int HOME = 0;
    public static int INTAKE = 50;
    public static int LOW_BASKET = 1200;
    public static int HIGH_BASKET = 2600;
    public static int HIGH_CHAMBER = 1800;
    public static int TOLERANCE = 15;

    public static double MAX_POWER = 1.0;

    // --- Hardware ---
    private final DcMotorEx motor;

    // --- State ---
    private int targetPosition = 0;
    private boolean manualMode = false;
    private double manualPower = 0;

    // --- PID state ---
    private double integralSum = 0;
    private double lastError = 0;
    private final ElapsedTime timer = new ElapsedTime();

    public LiftSubsystem(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "liftMotor");
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Register with the CommandScheduler automatically
        register();
    }

    /**
     * Called EVERY scheduler loop — runs PID regardless of which command is active.
     * This ensures the mechanism is always actively controlled.
     */
    @Override
    public void periodic() {
        if (manualMode) {
            motor.setPower(manualPower);
            // Track position so we can hold when manual mode exits
            targetPosition = motor.getCurrentPosition();
            return;
        }

        // --- Software PID ---
        // Read @Config coefficients fresh each loop for live Dashboard tuning
        int current = motor.getCurrentPosition();
        double error = targetPosition - current;
        double dt = timer.seconds();
        timer.reset();

        // Proportional
        double pTerm = kP * error;

        // Integral (with anti-windup clamping)
        integralSum += error * dt;
        integralSum = Math.max(-500, Math.min(500, integralSum));
        double iTerm = kI * integralSum;

        // Derivative
        double dTerm = (dt > 0) ? kD * (error - lastError) / dt : 0;
        lastError = error;

        // Feedforward (constant gravity compensation for linear slides)
        double fTerm = (targetPosition > 50) ? kF : 0;

        // Total output, clamped
        double output = pTerm + iTerm + dTerm + fTerm;
        output = Math.max(-MAX_POWER, Math.min(MAX_POWER, output));
        motor.setPower(output);
    }

    // --- Target setters (called by commands) ---

    public void setTarget(int position) {
        manualMode = false;
        targetPosition = position;
        integralSum = 0; // reset integral on new target to prevent windup
    }

    public void goHome()       { setTarget(HOME); }
    public void goIntake()     { setTarget(INTAKE); }
    public void goLowBasket()  { setTarget(LOW_BASKET); }
    public void goHighBasket() { setTarget(HIGH_BASKET); }

    // --- Manual control (for joystick override) ---

    public void setManualPower(double power) {
        manualMode = true;
        manualPower = power;
    }

    public void exitManual() {
        manualMode = false;
        // Hold current position when switching back to PID
        targetPosition = motor.getCurrentPosition();
        integralSum = 0;
    }

    // --- Queries ---

    public boolean atTarget() {
        return !manualMode && Math.abs(motor.getCurrentPosition() - targetPosition) < TOLERANCE;
    }

    public int getPosition() {
        return motor.getCurrentPosition();
    }

    public int getTarget() {
        return targetPosition;
    }

    public boolean isManual() {
        return manualMode;
    }
}`

};
