# ftc-mcp

MCP server that gives AI coding assistants deep knowledge of the FTC Robot Controller ecosystem. Enables teams to "vibe code" their robots through natural language while producing correct, optimized, competition-ready Java code.

**The problem:** AI assistants hallucinate wrong Pedro Pathing APIs (training data is outdated), don't know the `@Config` + `public static` dashboard pattern, use wrong import paths, and can't see your project structure.

**The fix:** This MCP server injects 9,500+ lines of verified FTC documentation, API references, and working code examples directly into your AI assistant's context.

## Quick Install

### Claude Code
```bash
claude mcp add ftc -- npx ftc-mcp
```

### Cursor
Add to `~/.cursor/mcp.json`:
```json
{
  "mcpServers": {
    "ftc": {
      "command": "npx",
      "args": ["ftc-mcp"]
    }
  }
}
```

### VS Code (Copilot)
Add to your MCP configuration:
```json
{
  "mcpServers": {
    "ftc": {
      "command": "npx",
      "args": ["ftc-mcp"]
    }
  }
}
```

### Local Development (from source)
```bash
git clone https://github.com/jackulau/ftcMCP.git
cd ftcMCP
npm install
npm run build

# Then add to your AI client:
claude mcp add ftc -- node /path/to/ftcMCP/build/index.js
```

## What It Provides

### 41 Resources (Documentation)

Contextual docs the AI pulls in when writing FTC code:

| Category | Resources | Coverage |
|----------|-----------|----------|
| **Pedro Pathing 2.0** | 6 | Complete API (Follower, PathBuilder, PathChain, BezierLine/Curve), Constants builder pattern, coordinate system [0,144], auto FSM structure, TeleOp drive, callbacks |
| **FTC Dashboard** | 6 | `@Config` + `public static` pattern, copy semantics pitfall, MultipleTelemetry, TelemetryPacket, Canvas field overlay API, camera streaming, setup |
| **Gradle** | 5 | Project file structure, adding libraries step-by-step, exact Maven coordinates for every library, common issues (compileSdk 34 for Pedro), build process |
| **Hardware** | 17 | DcMotor/DcMotorEx full API, RunModes, motor specs (every goBILDA/REV CPR), servos, IMU, distance/color/touch sensors, encoders (port 0+3 vs 1+2), GoBilda Pinpoint, SparkFun OTOS, REV Hub internals, bulk reads, CachingHardware, custom wrapper patterns, VisionPortal + Limelight |
| **Core SDK** | 5 | OpMode lifecycle (iterative vs linear), hardwareMap patterns, gamepad API, best practices |
| **Road Runner** | 1 | Actions API, TrajectoryActionBuilder |
| **FTCLib** | 1 | Command-based framework, GamepadEx, triggers |

### 6 Tools

| Tool | What It Does |
|------|-------------|
| `scan_ftc_project` | Reads your TeamCode directory -- detects SDK version, installed libraries, existing OpModes, hardware config, Pedro Constants. Call at the start of every session. |
| `scan_hardware_config` | Parses all `hardwareMap.get()` calls to extract every hardware device name and type in your project. |
| `search_ftc_docs` | Full-text search across the entire knowledge base. Filter by category: `pedro`, `dashboard`, `gradle`, `hardware`, `sdk`, etc. |
| `get_ftc_example` | Returns a complete, compilable Java file for any of 10 topics (see below). |
| `get_hardware_reference` | Deep API reference for any device: `DcMotorEx`, `Pinpoint`, `OTOS`, `bulk-reads`, `caching-hardware`, etc. |
| `validate_ftc_code` | Checks code for common FTC mistakes: missing `follower.update()`, `@Config` with `final`, Thread.sleep in iterative OpMode, wrong Pedro v1 imports, copy semantics bugs. |

### 10 Complete Code Examples

Every example is a full, compilable Java file with package declaration, all imports, and working code:

| Topic | Description |
|-------|-------------|
| `pedro-auto` | Pedro Pathing autonomous with FSM state machine, path callbacks, @Config tunable poses, field overlay |
| `pedro-teleop` | Pedro TeleOp with `setTeleOpDrive()`, slow mode, bulk reads, loop timer |
| `pedro-constants` | Complete Constants.java with FollowerConstants, MecanumConstants, PinpointConstants builders |
| `dashboard-config` | @Config demonstration with correct/wrong copy semantics examples |
| `bulk-reads` | Optimized OpMode with LynxModule MANUAL + CachingHardware |
| `subsystem` | Hardware subsystem class with @Config positions, state methods |
| `pid-tuning` | Live PID tuning with dashboard-graphed error/output |
| `vision-pipeline` | VisionPortal + AprilTag processor with dashboard camera stream |
| `custom-pid-drive` | Encoder-based autonomous with IMU heading PID (no pathing library) |
| `field-centric-drive` | Field-centric mecanum TeleOp using IMU |

### 8 Workflow Prompts

Structured instructions that guide the AI through complex FTC tasks:

| Prompt | Description |
|--------|-------------|
| `setup-ftc-project` | Guided project init: choose pathing lib, configure Gradle, add dashboard |
| `create-autonomous` | Full auto creation: poses, paths, FSM, callbacks, dashboard telemetry |
| `create-teleop` | TeleOp: drive type, subsystems, gamepad bindings, slow mode |
| `create-subsystem` | Hardware subsystem with @Config tuning, state methods |
| `tune-pid` | PID tuning with dashboard live graphing |
| `optimize-performance` | Bulk reads, CachingHardware, loop timer, I2C reduction |
| `add-dashboard-tuning` | Add @Config live-tunable variables to existing code |
| `setup-gradle` | Configure Gradle deps for any combination of FTC libraries |

## Supported Libraries

| Library | Version | Knowledge Depth |
|---------|---------|-----------------|
| [FTC SDK](https://github.com/FIRST-Tech-Challenge/FtcRobotController) | 11.1.0 | Full hardware API, OpMode lifecycle, gamepad, telemetry |
| [Pedro Pathing](https://pedropathing.com/) | 2.0.6 | Complete v2.0 API with builder patterns (NOT the outdated v1.x) |
| [FTC Dashboard](https://github.com/acmerobotics/ftc-dashboard) | 0.5.1 | @Config, MultipleTelemetry, Canvas, camera streaming |
| [Road Runner](https://rr.brott.dev/) | 1.0.x | Actions API, TrajectoryActionBuilder |
| [CachingHardware](https://github.com/Dairy-Foundation/CachingHardware) | 1.0.0 | Write caching algorithm, drop-in wrappers |
| [FTCLib](https://docs.ftclib.org/) | 2.1.1 | Command-based framework, GamepadEx |

## Supported Hardware

Full API documentation and initialization patterns for:

- **Motors:** DcMotor, DcMotorEx, all RunModes, PIDF coefficients, every goBILDA/REV/NeveRest motor with exact CPR
- **Servos:** Servo, ServoImplEx (PWM range), CRServo, power pairing rules
- **Sensors:** REV IMU, Color Sensor V3, 2m Distance Sensor, Touch Sensor, Through Bore Encoder
- **Localizers:** goBILDA Pinpoint (full driver API, offsets, status enum), SparkFun OTOS (scalars, calibration)
- **Vision:** VisionPortal, AprilTagProcessor, Limelight 3A
- **REV Hub:** LynxModule bulk reads (OFF/AUTO/MANUAL), I2C timing, encoder port hardware vs software decoding

## Example Vibe Coding Sessions

**"Set up my project with Pedro Pathing and Dashboard"**
> AI calls `scan_ftc_project` -> reads `ftc://gradle/all-library-coords` -> edits `build.dependencies.gradle` with exact repos and versions -> changes `compileSdk` to 34 -> creates Constants.java with builder pattern

**"Create an autonomous that scores 3 samples"**
> AI reads `ftc://pedro/api-reference` + `ftc://pedro/auto-structure` -> generates complete OpMode with @Config tunable poses, FSM state machine, path callbacks, MultipleTelemetry, field overlay

**"My loop times are slow"**
> AI reads `ftc://hardware/bulk-reads` + `ftc://hardware/caching-hardware` -> adds LynxModule MANUAL + CachingDcMotorEx + loop timer telemetry

**"Add a dashboard variable so I can tune arm position"**
> AI reads `ftc://dashboard/config-pattern` -> adds `@Config` class with `public static double ARM_POSITION = 0.5;` -> warns about reading it fresh each loop (copy semantics)

## Project Structure

```
ftc-mcp/
├── src/
│   ├── index.ts                  # Entry point (stdio transport)
│   ├── server.ts                 # McpServer setup
│   ├── knowledge/
│   │   ├── pedro.ts              # Pedro Pathing 2.0 (1,550 lines)
│   │   ├── hardware.ts           # Full hardware stack (1,479 lines)
│   │   ├── examples.ts           # 10 complete code examples (1,396 lines)
│   │   ├── ftc-sdk.ts            # SDK patterns (881 lines)
│   │   ├── dashboard.ts          # FTC Dashboard (845 lines)
│   │   ├── ftclib.ts             # FTCLib framework (636 lines)
│   │   ├── gradle.ts             # Gradle build system (584 lines)
│   │   └── roadrunner.ts         # Road Runner (478 lines)
│   ├── resources/registry.ts     # 41 resource URI registrations
│   ├── tools/registry.ts         # 6 tool implementations
│   └── prompts/registry.ts       # 8 workflow prompts
├── package.json
└── tsconfig.json
```

## Development

```bash
npm install
npm run build          # Compile TypeScript
npm run dev            # Watch mode
npm start              # Run the server
```

## Requirements

- Node.js >= 18
- An MCP-compatible AI client (Claude Code, Cursor, VS Code Copilot, etc.)

## License

MIT
