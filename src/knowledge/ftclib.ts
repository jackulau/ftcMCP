export const FTCLIB_KNOWLEDGE = {

  // ── Section 1: Setup & Installation ─────────────────────────────────────
  setup: `
# SolversLib (Command-Based Framework) — Setup & Installation

## SolversLib vs FTCLib

**SolversLib** is the actively maintained fork of FTCLib, developed by Seattle Solvers.
It adds significant new features and fixes bugs that FTCLib has not addressed.

**CRITICAL: SolversLib and FTCLib CANNOT coexist in the same project.**
They share the same class names in different packages. Pick one — SolversLib is recommended.

| Feature | FTCLib | SolversLib |
|---|---|---|
| Package | com.arcrobotics.ftclib | com.seattlesolvers.solverslib |
| Maven | Maven Central | repo.dairy.foundation |
| Maintained | Inactive | Active (2024+) |
| Bulk caching | Manual only | Built into CommandScheduler |
| CallbackCommand | No | Yes |
| UninterruptibleCommand | No | Yes |
| DeferredCommand | No | Yes |
| Commands utility class | No | Yes |
| SlewRateLimiter | No | Yes |
| PlayStation aliases | No | Yes |
| initialize_loop() | No | Yes |

## Gradle Installation (SolversLib)

In \`build.dependencies.gradle\`:

\`\`\`groovy
repositories {
    maven { url = "https://repo.dairy.foundation/releases" }
}

dependencies {
    implementation "org.solverslib:core:0.3.4"
}
\`\`\`

**No other repositories needed** — SolversLib is hosted on Dairy Foundation's Maven.

## Gradle Installation (FTCLib — Legacy)

In \`build.dependencies.gradle\`:

\`\`\`groovy
// FTCLib uses Maven Central — no extra repo needed
dependencies {
    implementation 'org.ftclib.ftclib:core:2.1.1'
}
\`\`\`

## Import Mapping (FTCLib → SolversLib)

If migrating from FTCLib to SolversLib, change ALL imports:

| FTCLib Import | SolversLib Import |
|---|---|
| com.arcrobotics.ftclib.command.* | com.seattlesolvers.solverslib.command.* |
| com.arcrobotics.ftclib.gamepad.* | com.seattlesolvers.solverslib.gamepad.* |
| com.arcrobotics.ftclib.hardware.* | com.seattlesolvers.solverslib.hardware.* |
| com.arcrobotics.ftclib.controller.* | com.seattlesolvers.solverslib.controller.* |

**The class names and APIs are identical** — only the package prefix changes.
A global find-and-replace of \`com.arcrobotics.ftclib\` → \`com.seattlesolvers.solverslib\` handles migration.
`,

  // ── Section 2: Core API Reference ───────────────────────────────────────
  apiReference: `
# SolversLib Command-Based Framework — Core API Reference

The command-based paradigm separates robot behavior into **Subsystems** (hardware capabilities)
and **Commands** (actions that use subsystems). A **CommandScheduler** orchestrates execution,
ensuring no two commands use the same subsystem simultaneously.

---

## CommandOpMode

Replaces \`OpMode\` or \`LinearOpMode\`. Manages the CommandScheduler automatically.

\`\`\`java
import com.seattlesolvers.solverslib.command.CommandOpMode;

@TeleOp(name = "Command TeleOp")
public class MyTeleOp extends CommandOpMode {

    @Override
    public void initialize() {
        // Called once when INIT is pressed
        // Register subsystems, create commands, bind buttons here
    }

    // Optional: called repeatedly between INIT press and START press
    @Override
    public void initialize_loop() {
        // SolversLib-only! Use for vision processing, sensor checks
        // during the init phase before match starts
    }

    // run() is called after START — scheduler runs automatically
    // Override only if needed:
    // @Override
    // public void run() {
    //     super.run();  // MUST call super to tick the scheduler
    //     telemetry.update();
    // }
}
\`\`\`

### CommandOpMode Key Methods
| Method | Description |
|---|---|
| \`initialize()\` | Setup hardware, subsystems, bindings. Called once on INIT. |
| \`initialize_loop()\` | **SolversLib-only.** Loops between INIT and START. Use for vision/sensors. |
| \`run()\` | Called each loop after START. Call \`super.run()\` to tick scheduler. |
| \`schedule(Command...)\` | Schedule commands for execution. |
| \`register(Subsystem...)\` | Register subsystems with the scheduler. |
| \`reset()\` | Reset the scheduler (called automatically on stop). |

---

## SubsystemBase

Represents a physical mechanism. Only one command can require a subsystem at a time.

\`\`\`java
import com.seattlesolvers.solverslib.command.SubsystemBase;

public class LiftSubsystem extends SubsystemBase {

    private final DcMotorEx motor;
    private int targetPosition = 0;
    private double kP = 0.005;
    private double kF = 0.1;

    public LiftSubsystem(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "liftMotor");
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        // Registers this subsystem with the CommandScheduler
        register();
    }

    @Override
    public void periodic() {
        // Called EVERY scheduler loop — regardless of which command is active
        // Use for PID control, telemetry, continuous state updates
        int current = motor.getCurrentPosition();
        int error = targetPosition - current;
        double power = error * kP + Math.signum(error) * kF;
        motor.setPower(Math.max(-1, Math.min(1, power)));
    }

    public void setTarget(int position) { targetPosition = position; }
    public boolean atTarget() {
        return Math.abs(motor.getCurrentPosition() - targetPosition) < 15;
    }
    public int getPosition() { return motor.getCurrentPosition(); }
    public void stop() { motor.setPower(0); }
}
\`\`\`

### Key Subsystem Patterns

**PID in periodic():** The subsystem's \`periodic()\` runs the PID loop every tick.
Commands just set the target — they don't control the motor directly. This means
the mechanism is always actively controlled even between commands.

**register() in constructor:** Calling \`register()\` inside the constructor automatically
registers the subsystem with the scheduler. Alternatively, call \`register(subsystem)\`
in your OpMode's \`initialize()\`.

**Default commands:** A default command runs whenever no other command requires the
subsystem. Typically used for drive control in TeleOp.

---

## CommandBase — Command Lifecycle

Commands define actions. Lifecycle: \`initialize → execute (loop) → end\`.

\`\`\`java
import com.seattlesolvers.solverslib.command.CommandBase;

public class LiftToPositionCommand extends CommandBase {

    private final LiftSubsystem lift;
    private final int targetPosition;

    public LiftToPositionCommand(LiftSubsystem lift, int targetPosition) {
        this.lift = lift;
        this.targetPosition = targetPosition;
        addRequirements(lift);  // Declare subsystem dependency
    }

    @Override
    public void initialize() {
        // Called ONCE when the command starts
        lift.setTarget(targetPosition);
    }

    @Override
    public void execute() {
        // Called EVERY loop while running — for telemetry, adjustments, etc.
        // PID runs in subsystem.periodic(), so often this is empty
    }

    @Override
    public boolean isFinished() {
        // Return true when the command should stop
        return lift.atTarget();
    }

    @Override
    public void end(boolean interrupted) {
        // Called once when the command ends
        // interrupted = true if another command took over
        if (interrupted) {
            lift.stop();
        }
    }
}
\`\`\`

### Command Types (Built-in)

\`\`\`java
import com.seattlesolvers.solverslib.command.*;

// --- InstantCommand: runs once, immediately finishes ---
new InstantCommand(() -> claw.open(), clawSubsystem);

// --- RunCommand: runs execute() forever, never finishes on its own ---
new RunCommand(() -> drive.mecanumDrive(gp.getLeftY(), gp.getLeftX(), gp.getRightX()), driveSubsystem);

// --- WaitCommand: waits for a duration (milliseconds) ---
new WaitCommand(500);

// --- WaitUntilCommand: waits until a condition is true ---
new WaitUntilCommand(() -> lift.atTarget());

// --- ConditionalCommand: choose between two commands ---
new ConditionalCommand(
    new ScoreHighCommand(lift, claw),   // if true
    new ScoreLowCommand(lift, claw),    // if false
    () -> scoringHigh                    // boolean supplier
);

// --- FunctionalCommand: inline command with all lifecycle methods ---
new FunctionalCommand(
    () -> motor.setPower(0.5),           // initialize
    () -> {},                             // execute
    interrupted -> motor.setPower(0),     // end
    () -> motor.getCurrentPosition() > 1000,  // isFinished
    subsystem                             // requirements
);

// --- SelectCommand: choose from a map of commands ---
new SelectCommand(
    Map.of(
        "high", new LiftToPositionCommand(lift, HIGH),
        "low", new LiftToPositionCommand(lift, LOW)
    ),
    () -> currentLevel  // supplier returning a map key
);

// --- PrintCommand: telemetry debug ---
new PrintCommand("State reached: scoring");
\`\`\`

### SolversLib-Only Command Types

\`\`\`java
// --- CallbackCommand: fire a callback when conditions are met ---
new CallbackCommand(
    () -> gamepad1.a,        // condition supplier
    () -> claw.toggle()      // action to run when condition is true
);

// --- UninterruptibleCommand: cannot be interrupted by other commands ---
new UninterruptibleCommand(new ScoreSequence(lift, claw));
// Even if another command requires the same subsystem, this won't be interrupted

// --- DeferredCommand: lazily creates the command at schedule time ---
new DeferredCommand(
    () -> new LiftToPositionCommand(lift, getCurrentTarget()),
    Set.of(lift)  // requirements declared upfront
);
// The lambda runs when the command is scheduled, not when it's constructed
// Useful when the target isn't known at construction time

// --- Commands utility class: fluent command construction ---
Commands.run(() -> intake.spin(), intakeSubsystem);
Commands.runOnce(() -> claw.toggle(), clawSubsystem);
Commands.waitSeconds(0.5);
Commands.waitUntil(() -> lift.atTarget());
Commands.idle();  // does nothing, never finishes (placeholder)
Commands.none();  // does nothing, immediately finishes
Commands.sequence(cmd1, cmd2, cmd3);
Commands.parallel(cmd1, cmd2);
Commands.deadline(deadlineCmd, cmd1, cmd2);
Commands.race(cmd1, cmd2);
Commands.either(trueCmd, falseCmd, () -> condition);
Commands.select(Map.of("a", cmd1, "b", cmd2), () -> key);
\`\`\`

---

## Command Groups — Composition

\`\`\`java
// --- SequentialCommandGroup: run in sequence ---
new SequentialCommandGroup(
    new LiftToPositionCommand(lift, HIGH_POS),
    new WaitCommand(300),
    new InstantCommand(() -> claw.open(), clawSubsystem)
);

// --- ParallelCommandGroup: run in parallel, finish when ALL complete ---
new ParallelCommandGroup(
    new LiftToPositionCommand(lift, HIGH_POS),
    new DriveToPositionCommand(drive, targetPose)
);

// --- ParallelRaceGroup: run in parallel, finish when ANY ONE completes ---
new ParallelRaceGroup(
    new DriveToPositionCommand(drive, targetPose),
    new WaitCommand(5000)  // 5-second timeout
);

// --- ParallelDeadlineGroup: finish when the FIRST (deadline) command completes ---
new ParallelDeadlineGroup(
    new DriveToPositionCommand(drive, targetPose), // deadline
    new RunCommand(() -> lift.holdPosition())       // runs alongside, cancelled when deadline ends
);
\`\`\`

### Fluent Composition API (on any Command)

\`\`\`java
// .andThen() — sequential chaining
liftUp.andThen(openClaw).andThen(liftDown);

// .alongWith() — run in parallel with another command
liftUp.alongWith(driveForward);

// .raceWith() — race: finish when either completes
driveForward.raceWith(new WaitCommand(3000));

// .withTimeout() — add a timeout (shorthand for raceWith WaitCommand)
driveForward.withTimeout(5000);

// .interruptOn() — cancel when condition becomes true
holdPosition.interruptOn(() -> gamepad1.b);

// .unless() — skip the command if condition is true
new ScoreHighCommand(lift, claw).unless(() -> !hasGamePiece);

// .perpetually() — restart the command when it finishes
new BlinkLEDCommand().perpetually();
\`\`\`
`,

  // ── Section 3: Subsystem Patterns ───────────────────────────────────────
  subsystemPatterns: `
# Command-Based Subsystem Design Patterns

## Pattern 1: PID-Controlled Subsystem with Manual/Auto Toggle

The standard pattern for mechanisms like lifts, arms, and turrets. PID runs
continuously in \`periodic()\`. Commands just set the target position.

\`\`\`java
@Config
public class LiftSubsystem extends SubsystemBase {

    // Dashboard-tunable constants
    public static double kP = 0.005;
    public static double kI = 0.0;
    public static double kD = 0.001;
    public static double kF = 0.12;   // gravity feedforward
    public static int TOLERANCE = 15;

    // Position presets (dashboard-tunable)
    public static int HOME = 0;
    public static int INTAKE = 50;
    public static int LOW_BASKET = 1200;
    public static int HIGH_BASKET = 2600;
    public static int HIGH_CHAMBER = 1800;

    private final DcMotorEx motor;
    private int targetPosition = 0;
    private boolean manualMode = false;
    private double manualPower = 0;

    // PID state
    private double integralSum = 0;
    private double lastError = 0;
    private final ElapsedTime timer = new ElapsedTime();

    public LiftSubsystem(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "liftMotor");
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        register();
    }

    @Override
    public void periodic() {
        if (manualMode) {
            motor.setPower(manualPower);
            targetPosition = motor.getCurrentPosition(); // track position
            return;
        }

        // Software PID — read coefficients fresh from @Config statics
        int current = motor.getCurrentPosition();
        double error = targetPosition - current;
        double dt = timer.seconds();
        timer.reset();

        // P
        double pTerm = kP * error;

        // I (with anti-windup)
        integralSum += error * dt;
        integralSum = Math.max(-500, Math.min(500, integralSum));
        double iTerm = kI * integralSum;

        // D
        double dTerm = (dt > 0) ? kD * (error - lastError) / dt : 0;
        lastError = error;

        // F (gravity compensation — constant for linear slides)
        double fTerm = (targetPosition > 50) ? kF : 0;

        double output = pTerm + iTerm + dTerm + fTerm;
        output = Math.max(-1.0, Math.min(1.0, output));
        motor.setPower(output);
    }

    // --- Target setters ---
    public void setTarget(int position) {
        manualMode = false;
        targetPosition = position;
        integralSum = 0; // reset integral on new target
    }
    public void goHome()       { setTarget(HOME); }
    public void goIntake()     { setTarget(INTAKE); }
    public void goLowBasket()  { setTarget(LOW_BASKET); }
    public void goHighBasket() { setTarget(HIGH_BASKET); }
    public void goHighChamber(){ setTarget(HIGH_BASKET); }

    // --- Manual control ---
    public void setManualPower(double power) {
        manualMode = true;
        manualPower = power;
    }
    public void exitManual() {
        manualMode = false;
        targetPosition = motor.getCurrentPosition(); // hold current position
    }

    // --- Queries ---
    public boolean atTarget() {
        return !manualMode && Math.abs(motor.getCurrentPosition() - targetPosition) < TOLERANCE;
    }
    public int getPosition() { return motor.getCurrentPosition(); }
    public int getTarget() { return targetPosition; }
}
\`\`\`

## Pattern 2: Servo Subsystem with Named Positions

\`\`\`java
@Config
public class ClawSubsystem extends SubsystemBase {

    public static double OPEN_POS = 0.65;
    public static double CLOSED_POS = 0.10;
    public static double HALF_POS = 0.40;

    private final Servo servo;
    private boolean isOpen = false;

    public ClawSubsystem(HardwareMap hardwareMap) {
        servo = hardwareMap.get(Servo.class, "claw");
        close();
        register();
    }

    public void open()   { servo.setPosition(OPEN_POS);   isOpen = true; }
    public void close()  { servo.setPosition(CLOSED_POS);  isOpen = false; }
    public void half()   { servo.setPosition(HALF_POS); }
    public void toggle() { if (isOpen) close(); else open(); }
    public boolean isOpen() { return isOpen; }
}
\`\`\`

## Pattern 3: Voltage-Compensated Motor

Battery voltage drops during a match. Voltage compensation keeps mechanism
behavior consistent as the battery drains.

\`\`\`java
@Config
public class IntakeSubsystem extends SubsystemBase {

    public static double INTAKE_POWER = 0.9;
    public static double OUTTAKE_POWER = -0.6;
    public static double NOMINAL_VOLTAGE = 12.0;

    private final DcMotorEx motor;
    private final VoltageSensor voltageSensor;

    public IntakeSubsystem(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "intake");
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        register();
    }

    /** Compensates motor power for current battery voltage. */
    private double compensate(double power) {
        double voltage = voltageSensor.getVoltage();
        return power * (NOMINAL_VOLTAGE / voltage);
    }

    public void intake()  { motor.setPower(compensate(INTAKE_POWER)); }
    public void outtake() { motor.setPower(compensate(OUTTAKE_POWER)); }
    public void stop()    { motor.setPower(0); }
}
\`\`\`

## Pattern 4: Supplier-Based Cross-Subsystem Dependency

When one subsystem needs data from another (e.g., arm angle depends on lift height),
use \`Supplier<>\` to avoid direct coupling.

\`\`\`java
public class ArmSubsystem extends SubsystemBase {

    private final Supplier<Integer> liftPositionSupplier;
    // ...

    public ArmSubsystem(HardwareMap hardwareMap, Supplier<Integer> liftPositionSupplier) {
        this.liftPositionSupplier = liftPositionSupplier;
        // ... motor init
        register();
    }

    @Override
    public void periodic() {
        int liftPos = liftPositionSupplier.get(); // read from lift without direct dependency
        // Adjust arm behavior based on lift height
        if (liftPos > 1000) {
            // lift is high — limit arm range for safety
        }
    }
}

// In OpMode.initialize():
LiftSubsystem lift = new LiftSubsystem(hardwareMap);
ArmSubsystem arm = new ArmSubsystem(hardwareMap, lift::getPosition);
\`\`\`

## Pattern 5: Robot Container Class

Holds all subsystems as a single object passed to OpModes. Provides
a clean initialization point and cross-subsystem wiring.

\`\`\`java
public abstract class Robot {
    public final LiftSubsystem lift;
    public final ClawSubsystem claw;
    public final IntakeSubsystem intake;
    public final DriveSubsystem drive;

    public Robot(HardwareMap hardwareMap) {
        // Initialize subsystems in dependency order
        drive = new DriveSubsystem(hardwareMap);
        lift = new LiftSubsystem(hardwareMap);
        claw = new ClawSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
    }

    /** Called once for common hardware setup (bulk reads, etc.) */
    public void init() {
        List<LynxModule> hubs = drive.hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }
}
\`\`\`
`,

  // ── Section 4: Command Patterns ─────────────────────────────────────────
  commandPatterns: `
# Command Design Patterns for FTC

## Pattern 1: Atomic Command (Set Target, Wait for Convergence)

The most common command pattern. Sets a target in \`initialize()\`,
then waits for the subsystem's PID loop (in \`periodic()\`) to converge.

\`\`\`java
public class LiftToPositionCommand extends CommandBase {
    private final LiftSubsystem lift;
    private final int target;

    public LiftToPositionCommand(LiftSubsystem lift, int target) {
        this.lift = lift;
        this.target = target;
        addRequirements(lift);
    }

    @Override public void initialize() { lift.setTarget(target); }
    @Override public boolean isFinished() { return lift.atTarget(); }
    @Override public void end(boolean interrupted) {
        if (interrupted) lift.setTarget(lift.getPosition()); // hold current
    }
}
\`\`\`

## Pattern 2: Timed Command (Set Hardware, Wait for Duration)

For mechanisms without feedback (servos, simple motors). Set the output,
wait a fixed time, then stop.

\`\`\`java
public class IntakeForTimeCommand extends CommandBase {
    private final IntakeSubsystem intake;
    private final double power;
    private final double seconds;
    private final ElapsedTime timer = new ElapsedTime();

    public IntakeForTimeCommand(IntakeSubsystem intake, double power, double seconds) {
        this.intake = intake;
        this.power = power;
        this.seconds = seconds;
        addRequirements(intake);
    }

    @Override public void initialize() {
        intake.setPower(power);
        timer.reset();
    }
    @Override public boolean isFinished() {
        return timer.seconds() >= seconds;
    }
    @Override public void end(boolean interrupted) {
        intake.stop();
    }
}
\`\`\`

## Pattern 3: Instant Command (Fire-and-Forget)

For immediate actions with no waiting. Use \`InstantCommand\` directly.

\`\`\`java
// Toggle claw on button press
new InstantCommand(() -> claw.toggle(), claw);

// Set a specific position
new InstantCommand(() -> lift.setTarget(HIGH_POS), lift);

// Multiple subsystems (pass all as requirements)
new InstantCommand(() -> {
    claw.open();
    wrist.setPosition(SCORE_POS);
}, claw, wrist);
\`\`\`

## Pattern 4: Persistent Drive Command (Default Command)

Never finishes on its own — used as a default command for the drive subsystem.
Reads gamepad input every loop.

\`\`\`java
public class DefaultDriveCommand extends CommandBase {
    private final DriveSubsystem drive;
    private final Supplier<Double> forwardSupplier;
    private final Supplier<Double> strafeSupplier;
    private final Supplier<Double> turnSupplier;

    public DefaultDriveCommand(
            DriveSubsystem drive,
            Supplier<Double> forward,
            Supplier<Double> strafe,
            Supplier<Double> turn) {
        this.drive = drive;
        this.forwardSupplier = forward;
        this.strafeSupplier = strafe;
        this.turnSupplier = turn;
        addRequirements(drive);
    }

    @Override
    public void execute() {
        drive.mecanumDrive(
            forwardSupplier.get(),
            strafeSupplier.get(),
            turnSupplier.get()
        );
    }

    @Override
    public boolean isFinished() {
        return false; // NEVER finishes — runs as default command
    }
}

// Usage: set as default command
drive.setDefaultCommand(new DefaultDriveCommand(
    drive,
    () -> -driverGp.getLeftY(),
    () ->  driverGp.getLeftX(),
    () ->  driverGp.getRightX()
));
\`\`\`

## Pattern 5: Score Macro (Sequential Command Group)

Compose complex multi-step actions from atomic commands.

\`\`\`java
public class ScoreHighBasketCommand extends SequentialCommandGroup {
    public ScoreHighBasketCommand(LiftSubsystem lift, ClawSubsystem claw) {
        addCommands(
            // 1. Raise lift to high basket
            new LiftToPositionCommand(lift, LiftSubsystem.HIGH_BASKET),
            // 2. Brief pause for stability
            new WaitCommand(200),
            // 3. Open claw to release sample
            new InstantCommand(claw::open, claw),
            // 4. Wait for sample to fall
            new WaitCommand(300),
            // 5. Lower lift to home
            new LiftToPositionCommand(lift, LiftSubsystem.HOME)
        );
    }
}

// Usage:
operatorGp.getGamepadButton(GamepadKeys.Button.Y)
    .whenPressed(new ScoreHighBasketCommand(lift, claw));
\`\`\`

## Pattern 6: Parallel Actions with Timeout

\`\`\`java
// Drive while lifting — both run simultaneously, finish when BOTH are done
new ParallelCommandGroup(
    new FollowPathCommand(follower, scorePath),
    new LiftToPositionCommand(lift, HIGH_POS)
);

// Drive with a timeout — finish when EITHER completes (race)
new FollowPathCommand(follower, scorePath)
    .withTimeout(4000); // 4 second max

// Drive with a gate — wait for lift before opening claw
new SequentialCommandGroup(
    new FollowPathCommand(follower, scorePath)
        .alongWith(new LiftToPositionCommand(lift, HIGH_POS)),
    new WaitUntilCommand(() -> lift.atTarget()),
    new InstantCommand(claw::open, claw)
);
\`\`\`

## Pattern 7: Conditional Branching

\`\`\`java
// Score high or low based on a toggle
new ConditionalCommand(
    new ScoreHighBasketCommand(lift, claw),
    new ScoreLowBasketCommand(lift, claw),
    () -> scoreHigh  // boolean supplier
);

// Skip command if precondition not met
new ScoreHighBasketCommand(lift, claw)
    .unless(() -> !hasGamePiece);

// Cancel running command on condition
new LiftToPositionCommand(lift, HIGH_POS)
    .interruptOn(() -> gamepad1.b);  // emergency stop on B press
\`\`\`
`,

  // ── Section 5: Triggers & GamepadEx ─────────────────────────────────────
  triggers: `
# Trigger System & GamepadEx

## GamepadEx — Enhanced Gamepad Wrapper

\`GamepadEx\` wraps the standard FTC \`Gamepad\` with edge detection (press/release
transitions) and normalized stick values.

\`\`\`java
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

GamepadEx driverGp = new GamepadEx(gamepad1);
GamepadEx operatorGp = new GamepadEx(gamepad2);
\`\`\`

### Stick & Trigger Inputs
\`\`\`java
double leftX  = driverGp.getLeftX();    // left stick horizontal  (-1 to 1)
double leftY  = driverGp.getLeftY();    // left stick vertical    (-1 to 1)
double rightX = driverGp.getRightX();   // right stick horizontal (-1 to 1)
double rightY = driverGp.getRightY();   // right stick vertical   (-1 to 1)

double lt = driverGp.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);   // 0 to 1
double rt = driverGp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);  // 0 to 1
\`\`\`

**Note:** \`getLeftY()\` is already properly oriented — positive = forward.
Unlike raw \`gamepad1.left_stick_y\` which is inverted (positive = backward).

### Button State Queries
\`\`\`java
boolean isPressed = driverGp.getButton(GamepadKeys.Button.A);
boolean justPressed = driverGp.wasJustPressed(GamepadKeys.Button.A);
boolean justReleased = driverGp.wasJustReleased(GamepadKeys.Button.A);
\`\`\`

### Available Buttons (GamepadKeys.Button)
- \`A\`, \`B\`, \`X\`, \`Y\`
- \`LEFT_BUMPER\`, \`RIGHT_BUMPER\`
- \`DPAD_UP\`, \`DPAD_DOWN\`, \`DPAD_LEFT\`, \`DPAD_RIGHT\`
- \`LEFT_STICK_BUTTON\`, \`RIGHT_STICK_BUTTON\`
- \`START\`, \`BACK\`

### SolversLib PlayStation Aliases
SolversLib adds PlayStation button name aliases for teams using PS4/PS5 controllers:
- \`CROSS\` (= A), \`CIRCLE\` (= B), \`SQUARE\` (= X), \`TRIANGLE\` (= Y)
- \`L1\` (= LEFT_BUMPER), \`R1\` (= RIGHT_BUMPER)
- \`L3\` (= LEFT_STICK_BUTTON), \`R3\` (= RIGHT_STICK_BUTTON)
- \`OPTIONS\` (= START), \`SHARE\` (= BACK), \`TOUCHPAD\`

---

## Trigger-Based Command Binding

The most powerful feature of the command framework. Declaratively bind commands
to gamepad inputs instead of writing if/else chains.

\`\`\`java
@Override
public void initialize() {
    GamepadEx operatorGp = new GamepadEx(gamepad2);

    // --- whenPressed: triggers ONCE when button transitions from released → pressed ---
    operatorGp.getGamepadButton(GamepadKeys.Button.A)
        .whenPressed(new InstantCommand(() -> claw.toggle(), claw));

    // --- whenReleased: triggers ONCE when button transitions from pressed → released ---
    operatorGp.getGamepadButton(GamepadKeys.Button.B)
        .whenReleased(new InstantCommand(() -> lift.stop(), lift));

    // --- whileHeld: command runs repeatedly while held, CANCELLED on release ---
    operatorGp.getGamepadButton(GamepadKeys.Button.DPAD_UP)
        .whileHeld(new RunCommand(() -> lift.setManualPower(0.5), lift));

    operatorGp.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
        .whileHeld(new RunCommand(() -> lift.setManualPower(-0.5), lift));

    // --- toggleWhenPressed: alternates between two commands each press ---
    operatorGp.getGamepadButton(GamepadKeys.Button.Y)
        .toggleWhenPressed(
            new LiftToPositionCommand(lift, HIGH_POS),  // 1st press
            new LiftToPositionCommand(lift, HOME_POS)    // 2nd press
        );

    // --- whenPressed with a command group (score macro) ---
    operatorGp.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
        .whenPressed(new SequentialCommandGroup(
            new LiftToPositionCommand(lift, HIGH_POS),
            new WaitCommand(200),
            new InstantCommand(claw::open, claw)
        ));

    // --- cancelWhenPressed: cancel a specific running command ---
    operatorGp.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
        .cancelWhenPressed(currentAutoCommand);
}
\`\`\`

### Binding Methods Summary

| Method | Triggers When | Command Behavior |
|---|---|---|
| \`whenPressed(cmd)\` | Button pressed (edge) | Schedules command once |
| \`whenReleased(cmd)\` | Button released (edge) | Schedules command once |
| \`whileHeld(cmd)\` | Button held | Runs while held, cancelled on release |
| \`toggleWhenPressed(cmd1, cmd2)\` | Button pressed (edge) | Alternates between two commands |
| \`cancelWhenPressed(cmd)\` | Button pressed (edge) | Cancels the specified command |

### Trigger Bindings (Analog)

\`\`\`java
// Bind to trigger axis (threshold-based)
new com.seattlesolvers.solverslib.command.button.Trigger(
    () -> operatorGp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5
).whenActive(new IntakeCommand(intake));
\`\`\`

---

## Default Commands

A default command runs whenever no other command requires its subsystem.
When another command finishes, the default command resumes automatically.

\`\`\`java
// Mecanum drive as default — always runs unless overridden
drive.setDefaultCommand(new RunCommand(
    () -> drive.mecanumDrive(
        -driverGp.getLeftY(),
        driverGp.getLeftX(),
        driverGp.getRightX()
    ),
    drive
));

// Lift holds position by default
lift.setDefaultCommand(new RunCommand(
    () -> lift.holdCurrent(),
    lift
));
\`\`\`
`,

  // ── Section 6: Project Organization ─────────────────────────────────────
  organization: `
# Command-Based Project Organization

## Recommended Directory Structure

\`\`\`
TeamCode/src/main/java/org/firstinspires/ftc/teamcode/
├── Constants/
│   ├── LiftConstants.java       // @Config: lift positions, PID gains
│   ├── DriveConstants.java      // @Config: drive speeds, heading PID
│   ├── ClawConstants.java       // @Config: servo positions
│   └── IntakeConstants.java     // @Config: motor powers, timing
│
├── subsystems/
│   ├── LiftSubsystem.java       // extends SubsystemBase
│   ├── ClawSubsystem.java       // extends SubsystemBase
│   ├── IntakeSubsystem.java     // extends SubsystemBase
│   └── DriveSubsystem.java      // extends SubsystemBase
│
├── commands/
│   ├── custom/                   // Atomic commands (one per file)
│   │   ├── LiftToPositionCommand.java
│   │   ├── IntakeForTimeCommand.java
│   │   ├── DefaultDriveCommand.java
│   │   └── FollowPathCommand.java
│   │
│   └── group/                    // Composite commands (SequentialCommandGroup, etc.)
│       ├── ScoreHighBasketCommand.java
│       ├── PickupSampleCommand.java
│       └── AutoScoreSequence.java
│
├── opmode/
│   ├── teleop/
│   │   ├── MainTeleOp.java       // extends CommandOpMode
│   │   └── TestTeleOp.java
│   │
│   └── auton/
│       ├── LeftAutoOp.java        // extends CommandOpMode
│       ├── RightAutoOp.java
│       └── ParkAutoOp.java
│
├── lib/
│   ├── Robot.java                 // Robot container (holds all subsystems)
│   ├── PIDController.java         // Custom PID if not using SolversLib's
│   └── Alliance.java              // Enum: RED, BLUE
│
└── paths/
    ├── LeftAutoPaths.java         // Pedro PathChain definitions
    └── RightAutoPaths.java
\`\`\`

## Key Organizational Principles

### 1. Constants in Separate @Config Classes
Put all tunable values in dedicated Constants classes with \`@Config\` annotation.
This makes them editable via FTC Dashboard and keeps subsystem code clean.

\`\`\`java
@Config
public class LiftConstants {
    public static double kP = 0.005;
    public static double kI = 0.0;
    public static double kD = 0.001;
    public static double kF = 0.12;

    public static int HOME = 0;
    public static int INTAKE = 50;
    public static int LOW_BASKET = 1200;
    public static int HIGH_BASKET = 2600;
    public static int TOLERANCE = 15;
}
\`\`\`

Subsystems then read these at point of use:
\`\`\`java
@Override
public void periodic() {
    double error = targetPosition - motor.getCurrentPosition();
    double power = LiftConstants.kP * error + LiftConstants.kF;
    // ...
}
\`\`\`

### 2. One Command Per File (for custom commands)
Each atomic command gets its own file in \`commands/custom/\`.
Command groups that compose them go in \`commands/group/\`.

### 3. Robot Container as Single Entry Point
The \`Robot\` class constructs all subsystems and provides them to OpModes.
This ensures consistent initialization and enables cross-subsystem wiring.

\`\`\`java
public class Robot {
    public final DriveSubsystem drive;
    public final LiftSubsystem lift;
    public final ClawSubsystem claw;

    public Robot(HardwareMap hardwareMap) {
        drive = new DriveSubsystem(hardwareMap);
        lift = new LiftSubsystem(hardwareMap);
        claw = new ClawSubsystem(hardwareMap);

        // Enable bulk reads
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }
}
\`\`\`

### 4. OpModes are Thin
OpModes should contain almost no logic — just:
1. Construct the Robot
2. Bind commands to buttons
3. Set default commands

All logic lives in subsystems and commands.

\`\`\`java
@TeleOp(name = "Main TeleOp")
public class MainTeleOp extends CommandOpMode {
    @Override
    public void initialize() {
        Robot robot = new Robot(hardwareMap);
        GamepadEx driverGp = new GamepadEx(gamepad1);
        GamepadEx operatorGp = new GamepadEx(gamepad2);

        // Default drive
        robot.drive.setDefaultCommand(new DefaultDriveCommand(
            robot.drive,
            () -> -driverGp.getLeftY(),
            () ->  driverGp.getLeftX(),
            () ->  driverGp.getRightX()
        ));

        // Operator bindings
        operatorGp.getGamepadButton(GamepadKeys.Button.A)
            .whenPressed(new InstantCommand(robot.claw::toggle, robot.claw));

        operatorGp.getGamepadButton(GamepadKeys.Button.Y)
            .whenPressed(new ScoreHighBasketCommand(robot.lift, robot.claw));

        operatorGp.getGamepadButton(GamepadKeys.Button.DPAD_UP)
            .whenPressed(new LiftToPositionCommand(robot.lift, LiftConstants.HIGH_BASKET));

        operatorGp.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
            .whenPressed(new LiftToPositionCommand(robot.lift, LiftConstants.HOME));
    }
}
\`\`\`
`,

  // ── Section 7: Pedro Pathing Integration ────────────────────────────────
  pedroIntegration: `
# Integrating Command-Based with Pedro Pathing

## FollowPathCommand — Wrapping Pedro in a Command

\`\`\`java
import com.seattlesolvers.solverslib.command.CommandBase;
import com.pedropathing.follower.Follower;
import com.pedropathing.pathgen.PathChain;

public class FollowPathCommand extends CommandBase {

    private final Follower follower;
    private final PathChain pathChain;
    private final boolean holdEnd;

    /**
     * @param follower  the Pedro Follower instance
     * @param pathChain the path to follow
     * @param holdEnd   true to hold position at the end
     */
    public FollowPathCommand(Follower follower, PathChain pathChain, boolean holdEnd) {
        this.follower = follower;
        this.pathChain = pathChain;
        this.holdEnd = holdEnd;
        // Note: no addRequirements() because Pedro manages its own motors.
        // If you have a DriveSubsystem wrapper, add that as a requirement.
    }

    @Override
    public void initialize() {
        follower.followPath(pathChain, holdEnd);
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
\`\`\`

## Command-Based Autonomous with Pedro

\`\`\`java
@Autonomous(name = "Command Auto - Left")
public class LeftAutoOp extends CommandOpMode {

    private Follower follower;
    private LiftSubsystem lift;
    private ClawSubsystem claw;

    // Paths
    private PathChain scorePreload, grabSample1, scoreSample1, park;

    @Override
    public void initialize() {
        follower = new Follower(hardwareMap);
        lift = new LiftSubsystem(hardwareMap);
        claw = new ClawSubsystem(hardwareMap);

        // Set starting pose
        follower.setStartingPose(new Pose(9, 60, Math.toRadians(0)));

        // Build paths (use @Config statics for positions)
        scorePreload = follower.pathBuilder()
            .addPath(new BezierLine(
                new Point(9, 60, Point.CARTESIAN),
                new Point(38, 68, Point.CARTESIAN)))
            .setLinearHeadingInterpolation(0, 0)
            .build();

        grabSample1 = follower.pathBuilder()
            .addPath(new BezierLine(
                new Point(38, 68, Point.CARTESIAN),
                new Point(37, 30, Point.CARTESIAN)))
            .setLinearHeadingInterpolation(0, 0)
            .build();

        scoreSample1 = follower.pathBuilder()
            .addPath(new BezierLine(
                new Point(37, 30, Point.CARTESIAN),
                new Point(38, 68, Point.CARTESIAN)))
            .setLinearHeadingInterpolation(0, 0)
            .build();

        park = follower.pathBuilder()
            .addPath(new BezierLine(
                new Point(38, 68, Point.CARTESIAN),
                new Point(10, 10, Point.CARTESIAN)))
            .setLinearHeadingInterpolation(0, 0)
            .build();

        // Schedule the full autonomous sequence
        schedule(new SequentialCommandGroup(
            // Score preloaded sample
            new FollowPathCommand(follower, scorePreload, true)
                .alongWith(new LiftToPositionCommand(lift, HIGH_POS)),
            new WaitCommand(200),
            new InstantCommand(claw::open, claw),
            new WaitCommand(300),

            // Grab sample 1
            new FollowPathCommand(follower, grabSample1, true)
                .alongWith(new LiftToPositionCommand(lift, INTAKE_POS)),
            new InstantCommand(claw::close, claw),
            new WaitCommand(200),

            // Score sample 1
            new FollowPathCommand(follower, scoreSample1, true)
                .alongWith(new LiftToPositionCommand(lift, HIGH_POS)),
            new WaitCommand(200),
            new InstantCommand(claw::open, claw),
            new WaitCommand(300),

            // Park
            new FollowPathCommand(follower, park, false)
                .alongWith(new LiftToPositionCommand(lift, HOME_POS))
        ));
    }
}
\`\`\`

### Key Integration Points

1. **follower.update() in execute():** The \`FollowPathCommand.execute()\` calls
   \`follower.update()\` every tick, which is REQUIRED for Pedro to work.

2. **Parallel actions with .alongWith():** Lift to scoring position WHILE driving
   to the scoring location. This saves time — both actions complete simultaneously.

3. **WaitCommand for mechanism settling:** Brief pauses between actions let servos
   complete their motion. Servos have no feedback, so time-based waiting is necessary.

4. **Subsystem requirements prevent conflicts:** Because \`LiftToPositionCommand\`
   requires the lift subsystem, two lift commands can't run simultaneously.

5. **initialize_loop() for vision:** If you need to detect game element position
   before the match starts (camera processing), use SolversLib's \`initialize_loop()\`:

\`\`\`java
@Override
public void initialize_loop() {
    // Runs between INIT and START — process camera, update telemetry
    detectedPosition = visionProcessor.getResult();
    telemetry.addData("Detected", detectedPosition);
    telemetry.update();
}
\`\`\`

## Road Runner Integration

\`\`\`java
import com.seattlesolvers.solverslib.command.CommandBase;
import com.acmerobotics.roadrunner.Action;

public class RoadRunnerActionCommand extends CommandBase {
    private final Action action;
    private boolean finished = false;

    public RoadRunnerActionCommand(Action action) {
        this.action = action;
    }

    @Override public void initialize() { finished = false; }

    @Override public void execute() {
        TelemetryPacket packet = new TelemetryPacket();
        finished = !action.run(packet);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }

    @Override public boolean isFinished() { return finished; }
}
\`\`\`
`,

  // ── Section 8: SolversLib Extras ────────────────────────────────────────
  extras: `
# SolversLib Additional Features

## SlewRateLimiter — Smooth Motor Acceleration

Limits the rate of change of a value per second. Use to smooth motor power
transitions and prevent mechanical shock.

\`\`\`java
import com.seattlesolvers.solverslib.util.SlewRateLimiter;

// Limit to 2.0 units/second change rate
SlewRateLimiter limiter = new SlewRateLimiter(2.0);

// In loop:
double rawPower = -gamepad1.left_stick_y;
double smoothPower = limiter.calculate(rawPower);
motor.setPower(smoothPower);
\`\`\`

This prevents instant 0→1 power jumps that can:
- Strip gears
- Cause wheel slippage
- Upset path following

## PIDController & PIDFController

SolversLib provides built-in PID controllers so you don't need to write your own.

\`\`\`java
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.controller.PIDFController;

// Simple PID
PIDController pid = new PIDController(0.01, 0, 0.001);
pid.setSetPoint(1000);   // target position
pid.setTolerance(15);    // acceptable error

// In loop:
double output = pid.calculate(motor.getCurrentPosition());
motor.setPower(output);

// Check if at target:
if (pid.atSetPoint()) { /* arrived */ }

// PIDF (with feedforward)
PIDFController pidf = new PIDFController(0.01, 0, 0.001, 0.1);
pidf.setSetPoint(2000);
double output = pidf.calculate(motor.getCurrentPosition());

// IMPORTANT: If using @Config live tuning, update coefficients each loop:
pidf.setPIDF(kP, kI, kD, kF);  // read from static fields
\`\`\`

## Bulk Caching in CommandScheduler

SolversLib's CommandScheduler can automatically manage bulk read caching,
calling \`clearBulkCache()\` at the start of each scheduler loop.

\`\`\`java
// In initialize():
CommandScheduler.getInstance().setBulkCacheMode(LynxModule.BulkCachingMode.MANUAL);
\`\`\`

This eliminates the need to manually call \`clearBulkCache()\` in your loop —
the scheduler handles it automatically. It's equivalent to:

\`\`\`java
// What the scheduler does internally each tick:
for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
    hub.clearBulkCache();
}
// ... then runs periodic() on all subsystems and executes commands
\`\`\`

## SquIDController (Square Root PID)

An alternative to standard PID that uses the square root of the error for
the proportional term. This gives more aggressive response far from target
and gentler response near target — often better for FTC mechanisms.

\`\`\`java
// SquID: P term = kP * sqrt(|error|) * sign(error)
// Standard: P term = kP * error
//
// Benefits:
// - More responsive at large errors (faster approach)
// - Gentler near target (less overshoot)
// - Often requires less D term to stabilize
\`\`\`

## MecanumDrive Helper

SolversLib includes a MecanumDrive utility class:

\`\`\`java
import com.seattlesolvers.solverslib.drivebase.MecanumDrive;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

Motor fl = new Motor(hardwareMap, "frontLeft");
Motor fr = new Motor(hardwareMap, "frontRight");
Motor bl = new Motor(hardwareMap, "backLeft");
Motor br = new Motor(hardwareMap, "backRight");

MecanumDrive drive = new MecanumDrive(fl, fr, bl, br);

// Robot-centric
drive.driveRobotCentric(strafeSpeed, forwardSpeed, turnSpeed);

// Field-centric (requires heading from IMU)
drive.driveFieldCentric(strafeSpeed, forwardSpeed, turnSpeed, heading);
\`\`\`

**Note:** Many teams prefer to write their own drive math directly on DcMotorEx
for more control over motor modes, voltage compensation, and caching. The
MecanumDrive helper is convenient for quick prototyping but may limit
advanced optimization.
`,
};
