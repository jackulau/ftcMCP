import { McpServer } from "@modelcontextprotocol/sdk/server/mcp.js";
import { z } from "zod";

export function registerPrompts(server: McpServer): void {
  // ── 1. Setup FTC Project ──────────────────────────────────────────────

  server.prompt(
    "setup-ftc-project",
    "Guided FTC project setup: choose pathing library, dashboard, configure Gradle",
    {
      pathingLibrary: z.string().optional().describe("pedro, roadrunner, or custom"),
      useDashboard: z.string().optional().describe("yes or no"),
    },
    async (args) => {
      const pathing = args.pathingLibrary ?? "not specified";
      const dashboard = args.useDashboard ?? "not specified";

      return {
        messages: [
          {
            role: "user" as const,
            content: {
              type: "text" as const,
              text: `Set up an FTC project with the following preferences:
- Pathing library: ${pathing}
- Use FTC Dashboard: ${dashboard}

Follow these steps IN ORDER:

1. Call the scan_ftc_project tool to understand the current project state (existing files, Gradle config, dependencies already present).

2. Read these resources for Gradle setup guidance:
   - ftc://gradle/project-structure — understand the FTC Gradle layout
   - ftc://gradle/adding-libraries — how to add repositories and dependencies
   - ftc://gradle/all-library-coords — exact Maven/Jitpack repo URLs and dependency coordinates with versions

3. Edit build.dependencies.gradle (NOT build.gradle):
   - Add required maven/jitpack repositories to the repositories { } block
   - Add implementation lines for the chosen libraries to the dependencies { } block
   - Use the EXACT coordinates and versions from ftc://gradle/all-library-coords

4. If adding Pedro Pathing: also edit build.common.gradle and change compileSdkVersion to 34 (Pedro requires API 34).

5. Set up bulk reading for performance:
   - Read ftc://hardware/bulk-reads
   - The base OpMode or hardware init should set all Lynx modules to BulkCachingMode.MANUAL and clear cache each loop iteration.

6. Create the TeamCode folder structure:
   - TeamCode/src/main/java/org/firstinspires/ftc/teamcode/
     - autonomous/
     - teleop/
     - subsystems/
     - util/

7. If using Pedro Pathing, create an initial Constants.java file:
   - Read ftc://pedro/constants-pattern for the correct pattern
   - Place in the util/ folder
   - Include starter poses and tuning values

8. Remind the user to perform a Gradle Sync in Android Studio after all file changes.

Important: Always use the exact library coordinates from the resources. Do not guess versions.`,
            },
          },
        ],
      };
    }
  );

  // ── 2. Create Autonomous ──────────────────────────────────────────────

  server.prompt(
    "create-autonomous",
    "Create a complete FTC autonomous routine",
    {
      pathingLibrary: z.string().optional().describe("pedro or roadrunner"),
      allianceColor: z.string().optional().describe("red or blue"),
      startPosition: z.string().optional().describe("e.g., near-basket, near-observation"),
    },
    async (args) => {
      const pathing = args.pathingLibrary ?? "pedro";
      const color = args.allianceColor ?? "not specified";
      const startPos = args.startPosition ?? "not specified";

      return {
        messages: [
          {
            role: "user" as const,
            content: {
              type: "text" as const,
              text: `Create a complete FTC autonomous routine with:
- Pathing library: ${pathing}
- Alliance color: ${color}
- Start position: ${startPos}

Follow these steps:

1. Call scan_ftc_project to understand the current project — existing OpModes, subsystems, Constants file, hardware names.

2. Read the appropriate pathing resources:
${pathing === "roadrunner" ? `   - ftc://roadrunner/api-reference — Road Runner trajectory API` : `   - ftc://pedro/api-reference — Pedro Pathing follower and path building API
   - ftc://pedro/auto-structure — correct autonomous OpMode structure for Pedro
   - ftc://pedro/coordinate-system — Pedro's coordinate system (important for pose math)`}

3. Read dashboard resources for live tuning:
   - ftc://dashboard/config-pattern — make ALL positions and tuning values @Config public static fields so they can be adjusted live from FTC Dashboard without redeploying
   - ftc://dashboard/telemetry — use MultipleTelemetry to send telemetry to both Driver Station and Dashboard; add field overlay drawing of the robot path

4. Generate the autonomous OpMode:
   - Use a finite state machine (FSM) pattern with an enum of states (e.g., DRIVE_TO_SPECIMEN, SCORE, PARK)
   - All poses and positions must be @Config public static so they are dashboard-tunable
   - Include path callbacks for triggering subsystem actions mid-path (read ftc://pedro/callbacks if using Pedro)
   - Call follower.update() every loop iteration (Pedro) or update drive in loop (Road Runner)
   - Add telemetry for current state, path progress, and loop time
   - Add field overlay drawing of planned paths and robot position

5. Call validate_ftc_code on the generated code to check for common mistakes.

Important: The auto should be competition-ready with proper error handling, state timeouts, and clean structure.`,
            },
          },
        ],
      };
    }
  );

  // ── 3. Create TeleOp ─────────────────────────────────────────────────

  server.prompt(
    "create-teleop",
    "Create a complete FTC TeleOp",
    {
      driveType: z.string().optional().describe("field-centric, robot-centric, or pedro"),
      subsystems: z.string().optional().describe("comma-separated list of subsystems, e.g., arm, claw, lift"),
    },
    async (args) => {
      const drive = args.driveType ?? "field-centric";
      const subsystems = args.subsystems ?? "not specified";

      return {
        messages: [
          {
            role: "user" as const,
            content: {
              type: "text" as const,
              text: `Create a complete FTC TeleOp with:
- Drive type: ${drive}
- Subsystems: ${subsystems}

Follow these steps:

1. Call scan_ftc_project to find existing subsystems, hardware config names, and Constants files.

2. Read the relevant resources:
   - ftc://sdk/gamepad-api — gamepad button/stick API and edge detection
   - ftc://sdk/opmode-patterns — LinearOpMode vs OpMode patterns
   - ftc://hardware/bulk-reads — set up bulk read caching (MANUAL mode, clear each loop)
   - ftc://dashboard/telemetry — MultipleTelemetry for dual DS + Dashboard output
${drive === "pedro" ? `   - ftc://pedro/teleop-structure — Pedro TeleOp drive integration` : `   - ftc://hardware/motors/api — motor control for mecanum math`}
${drive === "field-centric" ? `   - ftc://hardware/sensors/imu — IMU heading for field-centric calculation` : ""}

3. Generate the TeleOp OpMode:
   - Drive control:
     - Mecanum drive with proper strafing math (left stick = translate, right stick X = rotate)
     - ${drive === "field-centric" ? "Field-centric: rotate input vector by negative IMU heading" : drive === "pedro" ? "Use Pedro follower.setTeleOpMovementVectors()" : "Robot-centric: direct stick-to-motor mapping"}
     - Slow mode toggle (e.g., left bumper held = 0.3x speed multiplier)
   - Subsystem control:
     - Map each subsystem to intuitive gamepad controls
     - Use edge detection (gamepad1.a && !previousA) for toggles, continuous for held actions
     - Document the full control mapping in a comment block at the top
   - Performance:
     - Bulk read caching in MANUAL mode with clearBulkCache() at the start of each loop
     - Minimize object allocation in the loop
   - Telemetry:
     - Show drive power, heading, subsystem states, loop time
     - Use MultipleTelemetry for Dashboard + Driver Station

4. Call validate_ftc_code on the generated code.`,
            },
          },
        ],
      };
    }
  );

  // ── 4. Create Subsystem ───────────────────────────────────────────────

  server.prompt(
    "create-subsystem",
    "Create a hardware subsystem with dashboard tuning",
    {
      subsystemType: z.string().optional().describe("e.g., arm, claw, lift, intake"),
      hardware: z.string().optional().describe("e.g., motor + servo, two motors"),
    },
    async (args) => {
      const type = args.subsystemType ?? "generic";
      const hw = args.hardware ?? "not specified";

      return {
        messages: [
          {
            role: "user" as const,
            content: {
              type: "text" as const,
              text: `Create an FTC subsystem class:
- Subsystem type: ${type}
- Hardware: ${hw}

Follow these steps:

1. Call scan_ftc_project to see existing subsystems and hardware configuration names.

2. Read the relevant resources:
   - ftc://hardware/motors/api and ftc://hardware/servos/api — motor and servo control API
   - ftc://hardware/motors/run-modes — RUN_TO_POSITION, RUN_USING_ENCODER, etc.
   - ftc://dashboard/config-pattern — @Config annotation for live tuning

3. Generate the subsystem class:
   - Hardware initialization:
     - Accept HardwareMap in the constructor
     - Get hardware devices by name from hardwareMap
     - Set motor directions, zero power behaviors, and modes
   - Dashboard-tunable constants:
     - Use @Config on the class
     - All position values, speeds, PID coefficients as public static fields
     - Example: public static double ARM_UP_POSITION = 0.8;
   - State methods:
     - Named methods for each position/action (e.g., goToIntake(), goToScore(), open(), close())
     - Return the subsystem instance for method chaining if appropriate
   - Update method:
     - An update() method called each loop that handles state transitions, PID control, etc.
   - Telemetry:
     - A method to add subsystem telemetry (current position, target, state, power)
     - Accept Telemetry object and add data lines

4. Call validate_ftc_code on the generated code.

Important: The class should be self-contained, testable, and follow FTC best practices. Use @Config public static for ALL tunable values so they can be adjusted live via FTC Dashboard.`,
            },
          },
        ],
      };
    }
  );

  // ── 5. Tune PID ───────────────────────────────────────────────────────

  server.prompt(
    "tune-pid",
    "Set up PID tuning with FTC Dashboard",
    {
      controllerType: z.string().optional().describe("e.g., drive, heading, arm, lift"),
    },
    async (args) => {
      const controller = args.controllerType ?? "not specified";

      return {
        messages: [
          {
            role: "user" as const,
            content: {
              type: "text" as const,
              text: `Set up PID tuning for: ${controller}

Follow these steps:

1. Call scan_ftc_project to find existing PID controllers and subsystem code.

2. Read the relevant resources:
   - ftc://dashboard/config-pattern — expose PID coefficients as @Config public static
   - ftc://dashboard/telemetry — graph target vs actual position in real time
   - ftc://dashboard/api — FTC Dashboard API for sending graph packets

3. Set up PID tuning infrastructure:
   - Expose coefficients as @Config public static fields:
     - public static double kP = 0.0;
     - public static double kD = 0.0;
     - public static double kI = 0.0;
     - public static double kF = 0.0; (feedforward for gravity compensation if arm/lift)
   - IMPORTANT: Re-read the static fields every loop iteration since Dashboard modifies them live. If you copy them into a PIDFController object, you must call controller.setPIDF(kP, kI, kD, kF) each loop.

4. Add telemetry graphing:
   - Send target position and actual position as separate telemetry keys every loop
   - These will appear as overlaid graphs on FTC Dashboard
   - Also send error (target - actual) and motor power
   - Use TelemetryPacket for high-frequency graph data

5. Provide tuning methodology instructions:
   - Step 1: Set kI = 0, kD = 0, kF = 0. Increase kP until the mechanism oscillates around the target.
   - Step 2: Increase kD to dampen oscillation. The mechanism should reach the target without overshooting.
   - Step 3: If there is steady-state error (mechanism stops just short of target), add a small kI. Keep kI as small as possible to avoid integral windup.
   - Step 4: For arms and lifts that fight gravity, add kF as a constant feedforward term. For arms, kF should vary with cos(angle). For lifts, kF is constant.
   - Remind: kI is rarely needed and often causes problems. Try increasing kP and kD first.

6. Call validate_ftc_code on the generated tuning code.`,
            },
          },
        ],
      };
    }
  );

  // ── 6. Optimize Performance ───────────────────────────────────────────

  server.prompt(
    "optimize-performance",
    "Optimize FTC robot loop times",
    async () => {
      return {
        messages: [
          {
            role: "user" as const,
            content: {
              type: "text" as const,
              text: `Optimize my FTC robot code for faster loop times.

Follow these steps:

1. Call scan_ftc_project to analyze the full project — all OpModes, subsystems, and hardware usage.

2. Read the performance resources:
   - ftc://hardware/bulk-reads — understand REV Hub bulk read caching modes
   - ftc://hardware/caching-hardware — CachingDcMotorEx, CachingServo to reduce redundant writes
   - ftc://hardware/optimization-summary — overview of all optimization techniques

3. Apply optimizations (check each one):
   a. Bulk reads:
      - Switch ALL Lynx modules to BulkCachingMode.MANUAL
      - Add clearBulkCache() at the very start of each loop iteration
      - This batches all encoder/sensor reads into one USB transfer instead of individual calls
   b. Caching hardware wrappers:
      - Replace DcMotorEx with CachingDcMotorEx — only sends setPower() commands when the value actually changes
      - Replace Servo with CachingServo — same write-deduplication for servos
      - This eliminates redundant USB writes that waste loop time
   c. Minimize I2C reads:
      - I2C sensors (color, distance, IMU) are extremely slow (~7ms each per read)
      - Only read I2C sensors when the value is actually needed
      - Cache I2C values and only re-read every N loops if real-time data isn't critical
      - Consider moving I2C reads to a separate thread if needed
   d. Loop timer:
      - Add a loop timer that measures and displays loop time via telemetry
      - Target: < 20ms per loop (50Hz+), ideal: < 10ms (100Hz+)
      - Track min/max/average loop times

4. Add before/after measurement:
   - Add loop time telemetry BEFORE making changes so the user can see the improvement
   - Typical unoptimized loop: 50-100ms
   - Typical optimized loop: 5-15ms

5. Call validate_ftc_code on modified code.`,
            },
          },
        ],
      };
    }
  );

  // ── 7. Add Dashboard Tuning ───────────────────────────────────────────

  server.prompt(
    "add-dashboard-tuning",
    "Add FTC Dashboard live-tunable variables to existing code",
    {
      className: z.string().optional().describe("class to add tuning to"),
      variables: z.string().optional().describe("comma-separated variable names to make tunable"),
    },
    async (args) => {
      const cls = args.className ?? "not specified";
      const vars = args.variables ?? "not specified";

      return {
        messages: [
          {
            role: "user" as const,
            content: {
              type: "text" as const,
              text: `Add FTC Dashboard live-tunable variables to existing code:
- Class: ${cls}
- Variables to make tunable: ${vars}

Follow these steps:

1. Call scan_ftc_project to find the target class and understand the current code.

2. Read the dashboard resources:
   - ftc://dashboard/config-pattern — the @Config annotation pattern and requirements
   - ftc://dashboard/telemetry — MultipleTelemetry setup for dual output
   - ftc://dashboard/api — Dashboard API details

3. Apply the @Config pattern:
   - Add @Config annotation to the class declaration
   - Convert the target variables to public static fields:
     - They MUST be public and static for Dashboard to detect them
     - Example: private double armSpeed = 0.5 → public static double armSpeed = 0.5
   - For long and double types, consider marking them volatile if they may be read from multiple threads
   - WARNING — Copy semantics: If you copy a @Config static into a local variable or object field, the local copy will NOT update when Dashboard changes the static. Always read from the static field directly each loop.

4. Set up MultipleTelemetry:
   - Replace the standard telemetry object with MultipleTelemetry:
     telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
   - This sends telemetry to BOTH the Driver Station and the Dashboard simultaneously
   - Must be done in init, not in the loop

5. Add telemetry output for the tunable variables:
   - Display current values of all tunable variables each loop
   - This helps verify that Dashboard changes are being applied correctly

6. Call validate_ftc_code on the modified code.

Important: Remind the user that Dashboard serves a web UI at http://192.168.43.1:8080/dash when connected to the robot WiFi. Variables appear under the Config tab organized by class name.`,
            },
          },
        ],
      };
    }
  );

  // ── 8. Command-Based Project Setup ──────────────────────────────────

  server.prompt(
    "setup-command-based",
    "Set up a command-based FTC project using SolversLib with subsystems, commands, and gamepad bindings",
    {
      pathingLibrary: z.string().optional().describe("pedro or roadrunner"),
      subsystems: z.string().optional().describe("Comma-separated subsystem names, e.g., lift, claw, intake, arm"),
    },
    async (args) => {
      const pathing = args.pathingLibrary ?? "pedro";
      const subsystems = args.subsystems ?? "not specified";

      return {
        messages: [
          {
            role: "user" as const,
            content: {
              type: "text" as const,
              text: `Set up a command-based FTC project using SolversLib:
- Pathing library: ${pathing}
- Subsystems: ${subsystems}

Follow these steps:

1. Call scan_ftc_project to analyze the current project state.

2. Read the command-base resources:
   - ftc://command-base/setup — SolversLib vs FTCLib, Gradle installation, import mapping
   - ftc://command-base/api — CommandOpMode, SubsystemBase, CommandBase, built-in command types, composition
   - ftc://command-base/organization — recommended directory structure, Constants pattern, Robot container
   - ftc://command-base/subsystems — PID-controlled subsystems, servo subsystems, voltage compensation, cross-subsystem dependencies
   - ftc://command-base/commands — atomic, timed, instant, persistent, macro, conditional patterns
   - ftc://command-base/triggers — GamepadEx, button bindings, trigger bindings, default commands

3. Set up Gradle for SolversLib:
   - Read ftc://gradle/adding-libraries for the process
   - Add \`maven { url = "https://repo.dairy.foundation/releases" }\` to repositories
   - Add \`implementation "org.solverslib:core:0.3.4"\` to dependencies
   - IMPORTANT: Do NOT also add FTCLib — they cannot coexist
${pathing === "pedro" ? `   - Also add Pedro Pathing dependencies and set compileSdk to 34
   - Read ftc://command-base/pedro-integration for Pedro + command-base integration patterns` : `   - Also add Road Runner dependencies`}

4. Create the project structure:
   - Constants/ — @Config classes for each subsystem's tunable values
   - subsystems/ — SubsystemBase classes with PID in periodic()
   - commands/custom/ — atomic commands (one per file)
   - commands/group/ — composite SequentialCommandGroup/ParallelCommandGroup
   - opmode/teleop/ — CommandOpMode TeleOp with button bindings
   - opmode/auton/ — CommandOpMode autonomous with path following
   - lib/ — Robot container class, utilities

5. Create the Robot container class:
   - Constructs all subsystems
   - Enables bulk reads (or use CommandScheduler.getInstance().setBulkCacheMode())
   - Provides subsystem instances to OpModes

6. Create the subsystem classes:
   - Each extends SubsystemBase
   - PID control in periodic() — commands just set targets
   - All tunable values as @Config public static
   - register() in constructor

7. Create atomic commands:
   - LiftToPositionCommand (set target, wait for convergence)
   - DefaultDriveCommand (persistent, reads gamepad suppliers)
${pathing === "pedro" ? `   - FollowPathCommand (wraps Pedro follower.followPath)` : `   - RoadRunnerActionCommand (wraps Road Runner Action)`}

8. Create the TeleOp OpMode:
   - Thin — just constructs Robot, binds buttons, sets default commands
   - Use GamepadEx for enhanced gamepad with edge detection
   - Bind operator buttons to commands using .whenPressed(), .whileHeld(), etc.
   - Set drive default command

9. Create the Autonomous OpMode:
   - Build paths in initialize()
   - Schedule a single SequentialCommandGroup with all steps
   - Use .alongWith() for parallel actions (drive while lifting)
   - Use WaitCommand for servo settling times

10. Call validate_ftc_code on generated code.

Important: Use SolversLib imports (com.seattlesolvers.solverslib.*), NOT FTCLib imports. All @Config values read at point of use in periodic(), never cached.`,
            },
          },
        ],
      };
    }
  );

  // ── 9. Build and Deploy ─────────────────────────────────────────────

  server.prompt(
    "build-and-deploy",
    "Set up build and deploy workflow for FTC — works with VS Code, Android Studio, IntelliJ, or command line",
    {
      ide: z.string().optional().describe("vscode, android-studio, intellij, or command-line"),
      connection: z.string().optional().describe("usb or wifi"),
    },
    async (args) => {
      const ide = args.ide ?? "not specified";
      const connection = args.connection ?? "not specified";

      return {
        messages: [
          {
            role: "user" as const,
            content: {
              type: "text" as const,
              text: `Set up the build and deploy workflow for my FTC project:
- IDE: ${ide}
- Robot connection: ${connection}

Follow these steps:

1. Call scan_ftc_project to check the current project state and Gradle configuration.

2. Read the development environment resource:
   - ftc://sdk/dev-environment — prerequisites (JDK 17, Android SDK, ANDROID_HOME), IDE-specific setup instructions for VS Code, Android Studio, and IntelliJ

3. Read the build and deploy resource:
   - ftc://gradle/build-and-deploy — Gradle wrapper commands, ADB connection, wireless deploy, logcat debugging, troubleshooting

4. Set up the development environment:
${ide === "vscode" ? `   - Ensure VS Code extensions are installed: Extension Pack for Java (vscjava.vscode-java-pack), Gradle for Java (vscjava.vscode-gradle)
   - Create .vscode/settings.json with JDK 17 path
   - Create .vscode/tasks.json with FTC build/deploy tasks for Ctrl+Shift+B integration` : ide === "android-studio" ? `   - Verify Android Studio is using JDK 17 (NOT the bundled JDK 21 from Ladybug)
   - Configure Gradle JDK: File → Settings → Build → Gradle → Gradle JDK → JDK 17
   - Do NOT upgrade Gradle or AGP when prompted` : ide === "intellij" ? `   - Set Gradle JDK to JDK 17: File → Settings → Build → Gradle → Gradle JDK
   - Import the project as a Gradle project` : `   - Verify JDK 17 is installed and JAVA_HOME is set
   - Verify Android SDK is installed and ANDROID_HOME is set
   - Verify adb is in PATH`}

5. Verify the build works:
   - Run \`./gradlew assembleDebug\` and confirm it succeeds
   - If it fails, check ftc://gradle/common-issues for troubleshooting

6. Set up robot connection:
${connection === "wifi" ? `   - Connect computer to Control Hub WiFi network
   - Run \`adb connect 192.168.43.1:5555\`
   - Verify with \`adb devices\`` : connection === "usb" ? `   - Connect USB cable from computer to Control Hub
   - Verify with \`adb devices\`` : `   - Explain both USB and WiFi connection options
   - USB: plug in and run \`adb devices\`
   - WiFi: connect to robot WiFi, then \`adb connect 192.168.43.1:5555\``}

7. Deploy and test:
   - Run \`./gradlew installDebug\` to build and deploy in one step
   - Confirm the app restarts on the Control Hub
   - Check the Driver Station for your OpModes

8. Set up debugging:
   - Show how to use \`adb logcat\` to view runtime logs
   - Explain how to add Log.d() statements in Java code
   - Show filtered logcat: \`adb logcat -s MyTag:*\`

Important: The Gradle wrapper handles everything. Android Studio is optional — any editor works. JDK 17 is required (not 21).`,
            },
          },
        ],
      };
    }
  );

  // ── 10. Setup Vision ─────────────────────────────────────────────────

  server.prompt(
    "setup-vision",
    "Set up FTC vision — USB webcam with VisionPortal and/or Limelight 3A for AprilTag and color detection",
    {
      visionSystem: z.string().optional().describe("webcam, limelight, or both"),
      detectionType: z.string().optional().describe("apriltag, color, or both"),
    },
    async (args) => {
      const system = args.visionSystem ?? "not specified";
      const detection = args.detectionType ?? "not specified";

      return {
        messages: [
          {
            role: "user" as const,
            content: {
              type: "text" as const,
              text: `Set up FTC vision for my robot:
- Vision system: ${system}
- Detection type: ${detection}

Follow these steps:

1. Call scan_ftc_project to check the current project state, existing vision code, and hardware names.

2. Read the vision overview to understand the options:
   - ftc://vision/overview — USB webcam vs Limelight comparison, when to use each

3. Based on the chosen vision system, read the appropriate setup guides:
${system === "limelight" || system === "both" ? `
   Limelight 3A:
   - ftc://vision/limelight — complete Limelight 3A API, all pipeline types, result reading
   - ftc://vision/megatag — MegaTag1/MegaTag2 robot localization from AprilTags
` : ""}${system === "webcam" || system === "both" || system === "not specified" ? `
   USB Webcam + VisionPortal:
   - ftc://vision/visionportal-setup — VisionPortal builder, lifecycle, streaming
   - ftc://vision/camera-controls — exposure, gain, focus, white balance settings
` : ""}
4. Based on the detection type, read the detection guides:
${detection === "apriltag" || detection === "both" || detection === "not specified" ? `
   AprilTag Detection:
   - ftc://vision/apriltag-detection — AprilTagProcessor setup, decimation, pose reading, tag libraries
` : ""}${detection === "color" || detection === "both" ? `
   Color Detection:
   - ftc://vision/color-detection — custom VisionProcessor with OpenCV, HSV thresholding, contour analysis
` : ""}
5. Read the optimization guide:
   - ftc://vision/optimization — resolution selection, decimation strategy, exposure for motion blur, processor toggling, stream format, ROI cropping, memory management

6. Read common vision patterns:
   - ftc://vision/patterns — init-phase detection, drive-to-AprilTag alignment, field localization, vision + path following integration

7. Generate the vision code:
${system === "limelight" ? `   - Initialize Limelight3A from hardwareMap
   - Set poll rate to 100Hz
   - Select the appropriate pipeline
   - Read results in the loop with staleness checking
   - If using MegaTag2: integrate IMU heading via updateRobotOrientation()` : system === "webcam" || system === "not specified" ? `   - Build VisionPortal with appropriate processors
   - Set camera resolution (640x480 recommended for balance of speed and range)
   - Configure manual exposure (5-6ms) and gain (250) for AprilTag detection
   - Set manual white balance for consistent color detection
   - Implement dynamic decimation switching (1 for init, 3 for driving)
   - Toggle processors when not needed to save CPU` : `   - Set up BOTH webcam VisionPortal AND Limelight
   - Use Limelight for AprilTag localization (offloads CPU)
   - Use webcam for custom color detection
   - Both run simultaneously on separate USB ports`}

8. Add appropriate telemetry:
   - Detection count, tag IDs, ranges, bearings
   - FPS monitoring
   - Pipeline latency (Limelight)
   - Camera state

9. Call validate_ftc_code on the generated code.

Important: Set manual camera controls for competition consistency. Always have a fallback for when vision fails. Disable LiveView and unused processors in competition for maximum performance.`,
            },
          },
        ],
      };
    }
  );

  // ── 11. Setup Gradle ──────────────────────────────────────────────────

  server.prompt(
    "setup-gradle",
    "Configure Gradle dependencies for FTC libraries",
    {
      libraries: z.string().optional().describe("Comma-separated: pedro, dashboard, roadrunner, caching-hardware, ftclib"),
    },
    async (args) => {
      const libs = args.libraries ?? "not specified";

      return {
        messages: [
          {
            role: "user" as const,
            content: {
              type: "text" as const,
              text: `Configure Gradle dependencies for the following FTC libraries: ${libs}

Follow these steps:

1. Call scan_ftc_project to see the current Gradle configuration and what's already installed.

2. Read the Gradle resources:
   - ftc://gradle/project-structure — understand which files to edit
   - ftc://gradle/adding-libraries — step-by-step instructions for adding repos and deps
   - ftc://gradle/all-library-coords — EXACT Maven/Jitpack repository URLs, group IDs, artifact IDs, and versions for ALL supported libraries

3. Edit build.dependencies.gradle:
   - Add any required maven { url '...' } entries to the repositories { } block
   - Add implementation 'group:artifact:version' lines to the dependencies { } block
   - Use the EXACT coordinates from ftc://gradle/all-library-coords — do not guess or use outdated versions
   - Do NOT duplicate repositories or dependencies that are already present

4. Handle special cases:
   - Pedro Pathing requires compileSdkVersion 34: edit build.common.gradle and change compileSdkVersion to 34
   - Pedro Pathing requires Java 8+: ensure sourceCompatibility and targetCompatibility are set (usually already correct in FTC projects)

5. Verify the configuration:
   - Check that no conflicting versions exist
   - Ensure repository URLs are correct (some libraries use jitpack.io, others use maven.google.com or custom Maven repos)

6. Remind the user to:
   - Perform a Gradle Sync in Android Studio (File → Sync Project with Gradle Files or click the elephant icon)
   - Check the Build output for any resolution errors
   - If Gradle sync fails, read ftc://gradle/common-issues for troubleshooting`,
            },
          },
        ],
      };
    }
  );
}
