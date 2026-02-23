export const FTCLIB_KNOWLEDGE = {
  apiReference: `
# FTCLib Command-Based Framework API Reference

FTCLib provides a command-based programming framework for FTC robots, inspired by
WPILib (FRC). It separates robot behavior into **Subsystems** (what the robot has)
and **Commands** (what the robot does), enabling clean, modular, and testable code.

---

## CommandOpMode

\`CommandOpMode\` replaces the standard \`OpMode\` or \`LinearOpMode\`. It manages the
command scheduler automatically and provides lifecycle hooks.

\`\`\`java
import com.arcrobotics.ftclib.command.CommandOpMode;

@TeleOp(name = "Command TeleOp")
public class MyTeleOp extends CommandOpMode {

    @Override
    public void initialize() {
        // Called once when INIT is pressed
        // Register subsystems, create commands, bind buttons here
    }

    // run() is called repeatedly after START — the command scheduler
    // is automatically invoked each loop. Override only if you need
    // additional per-loop logic:
    // @Override
    // public void run() {
    //     super.run();  // MUST call super to run the scheduler
    //     telemetry.update();
    // }
}
\`\`\`

### Key Methods
- \`initialize()\` — setup hardware, subsystems, and command bindings
- \`run()\` — called each loop; call \`super.run()\` to tick the scheduler
- \`schedule(Command...)\` — schedule commands for execution
- \`register(Subsystem...)\` — register subsystems with the scheduler
- \`reset()\` — reset the scheduler (called automatically)

---

## SubsystemBase

Subsystems represent physical mechanisms on the robot. Each subsystem has a
\`periodic()\` method that runs every scheduler loop, and only one command can
require a subsystem at a time (preventing conflicts).

\`\`\`java
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LiftSubsystem extends SubsystemBase {

    private final DcMotorEx liftMotor;
    private int targetPosition = 0;

    public LiftSubsystem(HardwareMap hardwareMap) {
        liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");
        liftMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        liftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Register this subsystem with the scheduler
        register();
    }

    @Override
    public void periodic() {
        // Called every loop iteration — use for telemetry, PID updates, etc.
        // This runs regardless of what command is using the subsystem
    }

    public void setTargetPosition(int position) {
        targetPosition = position;
        liftMotor.setTargetPosition(position);
        liftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(1.0);
    }

    public int getCurrentPosition() {
        return liftMotor.getCurrentPosition();
    }

    public boolean atTarget() {
        return Math.abs(liftMotor.getCurrentPosition() - targetPosition) < 15;
    }

    public void stop() {
        liftMotor.setPower(0);
    }
}
\`\`\`

### Key Points
- Call \`register()\` in the constructor OR in your OpMode's \`initialize()\`
- \`periodic()\` is called every scheduler loop — use for continuous updates
- Subsystems enforce mutual exclusion: only one command can require a subsystem at a time
- Default commands run when no other command requires the subsystem

---

## CommandBase

Commands define discrete actions that operate on subsystems. They have a clear
lifecycle: \`initialize → execute (loop) → end\`.

\`\`\`java
import com.arcrobotics.ftclib.command.CommandBase;

public class LiftToPositionCommand extends CommandBase {

    private final LiftSubsystem lift;
    private final int targetPosition;

    public LiftToPositionCommand(LiftSubsystem lift, int targetPosition) {
        this.lift = lift;
        this.targetPosition = targetPosition;

        // Declare subsystem requirements — prevents conflicting commands
        addRequirements(lift);
    }

    @Override
    public void initialize() {
        // Called once when the command starts
        lift.setTargetPosition(targetPosition);
    }

    @Override
    public void execute() {
        // Called every loop while the command is running
        // Use for continuous control, telemetry updates, etc.
    }

    @Override
    public boolean isFinished() {
        // Return true when the command should stop
        return lift.atTarget();
    }

    @Override
    public void end(boolean interrupted) {
        // Called once when the command ends (interrupted = true if cancelled)
        if (interrupted) {
            lift.stop();
        }
    }
}
\`\`\`

### Built-in Command Types

\`\`\`java
import com.arcrobotics.ftclib.command.*;

// InstantCommand — runs once, immediately finishes
new InstantCommand(() -> claw.setPosition(1.0), clawSubsystem);

// RunCommand — runs execute() repeatedly, never finishes on its own
new RunCommand(() -> drive.arcadeDrive(gamepad1.left_stick_y, gamepad1.right_stick_x), driveSubsystem);

// WaitCommand — waits for a duration (milliseconds)
new WaitCommand(500);

// SequentialCommandGroup — runs commands in sequence
new SequentialCommandGroup(
    new LiftToPositionCommand(lift, 2000),
    new WaitCommand(300),
    new InstantCommand(() -> claw.setPosition(1.0), clawSubsystem)
);

// ParallelCommandGroup — runs commands in parallel, finishes when ALL complete
new ParallelCommandGroup(
    new LiftToPositionCommand(lift, 2000),
    new DriveToPositionCommand(drive, targetPose)
);

// ParallelRaceGroup — runs in parallel, finishes when ANY ONE completes
new ParallelRaceGroup(
    new DriveToPositionCommand(drive, targetPose),
    new WaitCommand(5000)  // 5-second timeout
);

// ParallelDeadlineGroup — finishes when the FIRST (deadline) command completes
new ParallelDeadlineGroup(
    new DriveToPositionCommand(drive, targetPose), // deadline
    new RunCommand(() -> lift.holdPosition())       // runs alongside
);

// ConditionalCommand — choose a command based on a condition
new ConditionalCommand(
    new ScoreHighCommand(lift, claw),
    new ScoreLowCommand(lift, claw),
    () -> scoringHigh  // boolean supplier
);
\`\`\`

---

## GamepadEx

\`GamepadEx\` wraps the standard FTC \`Gamepad\` with enhanced features like
edge detection (detecting button press/release transitions) and normalized stick values.

\`\`\`java
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

public class MyTeleOp extends CommandOpMode {

    private GamepadEx driverGamepad;
    private GamepadEx operatorGamepad;

    @Override
    public void initialize() {
        driverGamepad = new GamepadEx(gamepad1);
        operatorGamepad = new GamepadEx(gamepad2);
    }

    @Override
    public void run() {
        super.run(); // must call super to run scheduler

        // Stick values (already normalized -1 to 1)
        double leftX = driverGamepad.getLeftX();   // left stick horizontal
        double leftY = driverGamepad.getLeftY();    // left stick vertical
        double rightX = driverGamepad.getRightX();  // right stick horizontal
        double rightY = driverGamepad.getRightY();  // right stick vertical

        // Trigger values (0 to 1)
        double leftTrigger = driverGamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
        double rightTrigger = driverGamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);

        // Button state queries
        boolean aPressed = driverGamepad.getButton(GamepadKeys.Button.A);

        // Edge detection (must call readButtons() each loop or use GamepadEx in CommandOpMode)
        boolean aJustPressed = driverGamepad.wasJustPressed(GamepadKeys.Button.A);
        boolean aJustReleased = driverGamepad.wasJustReleased(GamepadKeys.Button.A);

        telemetry.update();
    }
}
\`\`\`

### Available Buttons (GamepadKeys.Button)
- \`A\`, \`B\`, \`X\`, \`Y\`
- \`LEFT_BUMPER\`, \`RIGHT_BUMPER\`
- \`DPAD_UP\`, \`DPAD_DOWN\`, \`DPAD_LEFT\`, \`DPAD_RIGHT\`
- \`LEFT_STICK_BUTTON\`, \`RIGHT_STICK_BUTTON\`
- \`START\`, \`BACK\`

### Available Triggers (GamepadKeys.Trigger)
- \`LEFT_TRIGGER\`, \`RIGHT_TRIGGER\`

---

## Trigger-Based Command Binding

The most powerful feature of FTCLib's command framework is **trigger-based binding**.
Instead of checking button states in a loop, you declaratively bind commands to
gamepad inputs.

\`\`\`java
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

@TeleOp(name = "Command TeleOp")
public class CommandTeleOp extends CommandOpMode {

    private GamepadEx driverGamepad;
    private GamepadEx operatorGamepad;
    private LiftSubsystem lift;
    private ClawSubsystem claw;
    private DriveSubsystem drive;

    @Override
    public void initialize() {
        driverGamepad = new GamepadEx(gamepad1);
        operatorGamepad = new GamepadEx(gamepad2);

        lift = new LiftSubsystem(hardwareMap);
        claw = new ClawSubsystem(hardwareMap);
        drive = new DriveSubsystem(hardwareMap);

        // --- Button Bindings ---

        // whenPressed — triggers once when button is first pressed
        operatorGamepad.getGamepadButton(GamepadKeys.Button.A)
            .whenPressed(new InstantCommand(() -> claw.toggle(), claw));

        // whenReleased — triggers once when button is released
        operatorGamepad.getGamepadButton(GamepadKeys.Button.B)
            .whenReleased(new InstantCommand(() -> lift.stop(), lift));

        // whileHeld — runs repeatedly while button is held, cancelled on release
        operatorGamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP)
            .whileHeld(new RunCommand(() -> lift.manualUp(), lift));

        operatorGamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
            .whileHeld(new RunCommand(() -> lift.manualDown(), lift));

        // toggleWhenPressed — toggles between two commands on each press
        operatorGamepad.getGamepadButton(GamepadKeys.Button.Y)
            .toggleWhenPressed(
                new LiftToPositionCommand(lift, 2000),  // first press
                new LiftToPositionCommand(lift, 0)       // second press
            );

        // whenPressed with a command group
        operatorGamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
            .whenPressed(new SequentialCommandGroup(
                new LiftToPositionCommand(lift, 2000),
                new WaitCommand(200),
                new InstantCommand(() -> claw.open(), claw)
            ));

        // cancelWhenPressed — cancels a specific command when pressed
        operatorGamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
            .cancelWhenPressed(currentAutoCommand);

        // --- Default Commands ---
        // Runs when no other command requires the subsystem
        drive.setDefaultCommand(new RunCommand(
            () -> drive.mecanumDrive(
                driverGamepad.getLeftY(),
                driverGamepad.getLeftX(),
                driverGamepad.getRightX()
            ),
            drive
        ));
    }
}
\`\`\`

### Binding Methods Summary
| Method                | When it triggers                          |
|-----------------------|-------------------------------------------|
| \`whenPressed(cmd)\`    | Once when button transitions to pressed   |
| \`whenReleased(cmd)\`   | Once when button transitions to released  |
| \`whileHeld(cmd)\`      | Repeatedly while held; cancelled on release|
| \`toggleWhenPressed()\` | Alternates between two commands each press |
| \`cancelWhenPressed()\` | Cancels the specified command on press     |

---

## Integration with Pedro Pathing and Road Runner

FTCLib's command framework integrates cleanly with both Pedro Pathing and Road Runner
for command-based autonomous routines.

### With Pedro Pathing

\`\`\`java
import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;

public class FollowPathCommand extends CommandBase {

    private final Follower follower;
    private final Path path;

    public FollowPathCommand(Follower follower, Path path) {
        this.follower = follower;
        this.path = path;
    }

    @Override
    public void initialize() {
        follower.followPath(path, true);
    }

    @Override
    public void execute() {
        follower.update();
    }

    @Override
    public boolean isFinished() {
        return !follower.isBusy();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            follower.breakFollowing();
        }
    }
}

// Usage in autonomous CommandOpMode:
// schedule(new SequentialCommandGroup(
//     new FollowPathCommand(follower, scorePath),
//     new LiftToPositionCommand(lift, HIGH_POS),
//     new InstantCommand(() -> claw.open(), clawSubsystem),
//     new FollowPathCommand(follower, pickupPath)
// ));
\`\`\`

### With Road Runner

\`\`\`java
import com.arcrobotics.ftclib.command.CommandBase;
import com.acmerobotics.roadrunner.Action;

public class RoadRunnerActionCommand extends CommandBase {

    private final Action action;
    private boolean finished = false;

    public RoadRunnerActionCommand(Action action) {
        this.action = action;
    }

    @Override
    public void initialize() {
        finished = false;
    }

    @Override
    public void execute() {
        com.acmerobotics.dashboard.telemetry.TelemetryPacket packet =
            new com.acmerobotics.dashboard.telemetry.TelemetryPacket();
        finished = !action.run(packet);
        com.acmerobotics.dashboard.FtcDashboard.getInstance()
            .sendTelemetryPacket(packet);
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}

// Usage:
// Action trajectory = drive.actionBuilder(startPose)
//     .lineToX(48)
//     .build();
// schedule(new SequentialCommandGroup(
//     new RoadRunnerActionCommand(trajectory),
//     new LiftToPositionCommand(lift, HIGH_POS)
// ));
\`\`\`

---

## Complete Working Example

\`\`\`java
package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "FTCLib Command Example", group = "Examples")
public class FTCLibCommandExample extends CommandOpMode {

    // --- Subsystems ---
    public static class DriveSubsystem extends SubsystemBase {
        private final DcMotorEx fl, fr, bl, br;

        public DriveSubsystem(HardwareMap hwMap) {
            fl = hwMap.get(DcMotorEx.class, "leftFront");
            fr = hwMap.get(DcMotorEx.class, "rightFront");
            bl = hwMap.get(DcMotorEx.class, "leftBack");
            br = hwMap.get(DcMotorEx.class, "rightBack");

            fl.setDirection(DcMotorSimple.Direction.REVERSE);
            bl.setDirection(DcMotorSimple.Direction.REVERSE);

            fl.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            fr.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            bl.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            br.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

            register();
        }

        public void mecanumDrive(double forward, double strafe, double turn) {
            double flPower = forward + strafe + turn;
            double frPower = forward - strafe - turn;
            double blPower = forward - strafe + turn;
            double brPower = forward + strafe - turn;

            double max = Math.max(1.0, Math.max(
                Math.max(Math.abs(flPower), Math.abs(frPower)),
                Math.max(Math.abs(blPower), Math.abs(brPower))
            ));

            fl.setPower(flPower / max);
            fr.setPower(frPower / max);
            bl.setPower(blPower / max);
            br.setPower(brPower / max);
        }

        public void stop() {
            mecanumDrive(0, 0, 0);
        }
    }

    public static class LiftSubsystem extends SubsystemBase {
        private final DcMotorEx motor;
        private int target = 0;

        public static final int GROUND = 0;
        public static final int LOW = 1000;
        public static final int HIGH = 2000;

        public LiftSubsystem(HardwareMap hwMap) {
            motor = hwMap.get(DcMotorEx.class, "liftMotor");
            motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            register();
        }

        public void setTarget(int position) {
            target = position;
            motor.setTargetPosition(position);
            motor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            motor.setPower(1.0);
        }

        public boolean atTarget() {
            return Math.abs(motor.getCurrentPosition() - target) < 15;
        }

        public int getPosition() { return motor.getCurrentPosition(); }

        public void stop() { motor.setPower(0); }
    }

    public static class ClawSubsystem extends SubsystemBase {
        private final Servo servo;
        private boolean isOpen = false;

        private static final double OPEN_POS = 0.6;
        private static final double CLOSED_POS = 0.0;

        public ClawSubsystem(HardwareMap hwMap) {
            servo = hwMap.get(Servo.class, "claw");
            servo.setPosition(CLOSED_POS);
            register();
        }

        public void open()   { servo.setPosition(OPEN_POS);   isOpen = true; }
        public void close()  { servo.setPosition(CLOSED_POS);  isOpen = false; }
        public void toggle() { if (isOpen) close(); else open(); }
    }

    // --- Commands ---
    public static class LiftToPositionCmd extends CommandBase {
        private final LiftSubsystem lift;
        private final int target;

        public LiftToPositionCmd(LiftSubsystem lift, int target) {
            this.lift = lift;
            this.target = target;
            addRequirements(lift);
        }

        @Override public void initialize() { lift.setTarget(target); }
        @Override public boolean isFinished() { return lift.atTarget(); }
        @Override public void end(boolean interrupted) { if (interrupted) lift.stop(); }
    }

    // --- OpMode ---
    @Override
    public void initialize() {
        GamepadEx driverGp = new GamepadEx(gamepad1);
        GamepadEx operatorGp = new GamepadEx(gamepad2);

        DriveSubsystem drive = new DriveSubsystem(hardwareMap);
        LiftSubsystem lift = new LiftSubsystem(hardwareMap);
        ClawSubsystem claw = new ClawSubsystem(hardwareMap);

        // Drive default command — always runs unless overridden
        drive.setDefaultCommand(new RunCommand(
            () -> drive.mecanumDrive(
                -driverGp.getLeftY(),
                driverGp.getLeftX(),
                driverGp.getRightX()
            ),
            drive
        ));

        // Operator bindings
        operatorGp.getGamepadButton(GamepadKeys.Button.A)
            .whenPressed(new InstantCommand(claw::toggle, claw));

        operatorGp.getGamepadButton(GamepadKeys.Button.DPAD_UP)
            .whenPressed(new LiftToPositionCmd(lift, LiftSubsystem.HIGH));

        operatorGp.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
            .whenPressed(new LiftToPositionCmd(lift, LiftSubsystem.GROUND));

        operatorGp.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
            .whenPressed(new LiftToPositionCmd(lift, LiftSubsystem.LOW));

        // Score macro: lift high → wait → open claw → lower
        operatorGp.getGamepadButton(GamepadKeys.Button.Y)
            .whenPressed(new SequentialCommandGroup(
                new LiftToPositionCmd(lift, LiftSubsystem.HIGH),
                new WaitCommand(200),
                new InstantCommand(claw::open, claw),
                new WaitCommand(300),
                new LiftToPositionCmd(lift, LiftSubsystem.GROUND)
            ));

        telemetry.addLine("FTCLib Command Example Initialized");
        telemetry.update();
    }
}
\`\`\`
`,
};
