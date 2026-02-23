import { McpServer } from "@modelcontextprotocol/sdk/server/mcp.js";
import { z } from "zod";
import fs from "node:fs";
import path from "node:path";

import { PEDRO_KNOWLEDGE } from "../knowledge/pedro.js";
import { DASHBOARD_KNOWLEDGE } from "../knowledge/dashboard.js";
import { HARDWARE_KNOWLEDGE } from "../knowledge/hardware.js";
import { FTC_SDK_KNOWLEDGE } from "../knowledge/ftc-sdk.js";
import { GRADLE_KNOWLEDGE } from "../knowledge/gradle.js";
import { ROADRUNNER_KNOWLEDGE } from "../knowledge/roadrunner.js";
import { FTCLIB_KNOWLEDGE } from "../knowledge/ftclib.js";
import { EXAMPLES } from "../knowledge/examples.js";

// ── Helper functions ────────────────────────────────────────────────────────

function readFileSafe(filePath: string): string | null {
  try {
    return fs.readFileSync(filePath, "utf-8");
  } catch {
    return null;
  }
}

function getJavaFiles(dir: string): string[] {
  const results: string[] = [];
  try {
    const entries = fs.readdirSync(dir, { withFileTypes: true });
    for (const entry of entries) {
      const fullPath = path.join(dir, entry.name);
      if (entry.isDirectory()) {
        results.push(...getJavaFiles(fullPath));
      } else if (entry.name.endsWith(".java")) {
        results.push(fullPath);
      }
    }
  } catch {
    // Directory doesn't exist or can't be read
  }
  return results;
}

// ── Tool registration ───────────────────────────────────────────────────────

export function registerTools(server: McpServer): void {

  // ══════════════════════════════════════════════════════════════════════════
  // Tool 1: scan_ftc_project
  // ══════════════════════════════════════════════════════════════════════════

  server.tool(
    "scan_ftc_project",
    "Scan an FTC Robot Controller project to detect SDK version, installed libraries, existing OpModes, hardware configuration, and project structure. Call this at the start of every coding session.",
    {
      projectPath: z.string().describe("Path to the FTC project root directory"),
    },
    async ({ projectPath }) => {
      try {
        const result: Record<string, unknown> = {};

        // 1. Read build.dependencies.gradle
        const depsGradle = readFileSafe(path.join(projectPath, "build.dependencies.gradle"));
        result.buildDependenciesGradle = depsGradle ? "found" : "not found";

        // 2. Read build.common.gradle for compileSdkVersion
        const commonGradle = readFileSafe(path.join(projectPath, "build.common.gradle"));
        let compileSdkVersion: string | null = null;
        if (commonGradle) {
          const sdkMatch = commonGradle.match(/compileSdkVersion\s+(\d+)/);
          if (sdkMatch) compileSdkVersion = sdkMatch[1];
        }
        result.compileSdkVersion = compileSdkVersion;

        // 3. Scan for libraries
        const libraries: Record<string, boolean> = {
          pedropathing: false,
          dashboard: false,
          roadrunner: false,
          cachinghardware: false,
          ftclib: false,
          solverslib: false,
        };
        if (depsGradle) {
          const depsLower = depsGradle.toLowerCase();
          if (depsLower.includes("pedropathing")) libraries.pedropathing = true;
          if (depsLower.includes("dashboard")) libraries.dashboard = true;
          if (depsLower.includes("roadrunner")) libraries.roadrunner = true;
          if (depsLower.includes("cachinghardware")) libraries.cachinghardware = true;
          if (depsLower.includes("ftclib")) libraries.ftclib = true;
          if (depsLower.includes("solverslib")) libraries.solverslib = true;
        }
        result.libraries = libraries;

        // 4. Find TeamCode directory
        const teamCodeDir = path.join(
          projectPath,
          "TeamCode/src/main/java/org/firstinspires/ftc/teamcode"
        );
        const teamCodeExists = fs.existsSync(teamCodeDir);
        result.teamCodeDir = teamCodeExists ? teamCodeDir : "not found";

        // 5-7. Scan Java files
        const opModes: Array<{ name: string; type: string; file: string; extendsClass: string }> = [];
        const configClasses: Array<{ name: string; file: string }> = [];
        const hardwareDevices: Array<{ name: string; type: string; file: string }> = [];
        const javaFiles: string[] = [];
        let usesPedro = false;
        let usesRoadRunner = false;
        let usesSolversLib = false;
        let usesFtcLib = false;
        let usesCommandBase = false;
        let hasFollowerConstants = false;
        let hasMecanumConstants = false;

        if (teamCodeExists) {
          const files = getJavaFiles(teamCodeDir);
          for (const filePath of files) {
            const relativePath = path.relative(projectPath, filePath);
            javaFiles.push(relativePath);

            const content = readFileSafe(filePath);
            if (!content) continue;

            // @Autonomous annotation
            const autoMatch = content.match(/@Autonomous\s*\(\s*name\s*=\s*"([^"]+)"/);
            if (autoMatch) {
              let extendsClass = "Unknown";
              const extendsMatch = content.match(/extends\s+(OpMode|LinearOpMode|CommandOpMode)/);
              if (extendsMatch) extendsClass = extendsMatch[1];
              opModes.push({ name: autoMatch[1], type: "Autonomous", file: relativePath, extendsClass });
            }

            // @TeleOp annotation
            const teleopMatch = content.match(/@TeleOp\s*\(\s*name\s*=\s*"([^"]+)"/);
            if (teleopMatch) {
              let extendsClass = "Unknown";
              const extendsMatch = content.match(/extends\s+(OpMode|LinearOpMode|CommandOpMode)/);
              if (extendsMatch) extendsClass = extendsMatch[1];
              opModes.push({ name: teleopMatch[1], type: "TeleOp", file: relativePath, extendsClass });
            }

            // @Config annotation
            if (content.includes("@Config")) {
              const classMatch = content.match(/class\s+(\w+)/);
              if (classMatch) {
                configClasses.push({ name: classMatch[1], file: relativePath });
              }
            }

            // hardwareMap.get() calls
            const hwRegex = /hardwareMap\.get\(\s*(\w+)\.class\s*,\s*"([^"]+)"\s*\)/g;
            let hwMatch;
            while ((hwMatch = hwRegex.exec(content)) !== null) {
              hardwareDevices.push({ name: hwMatch[2], type: hwMatch[1], file: relativePath });
            }

            // Old-style hardwareMap access
            const oldHwRegex = /hardwareMap\.\w+\.get\("([^"]+)"\)/g;
            let oldHwMatch;
            while ((oldHwMatch = oldHwRegex.exec(content)) !== null) {
              hardwareDevices.push({ name: oldHwMatch[1], type: "unknown", file: relativePath });
            }

            // Pedro imports
            if (content.includes("com.pedropathing")) usesPedro = true;

            // Road Runner imports
            if (content.includes("com.acmerobotics.roadrunner")) usesRoadRunner = true;

            // SolversLib imports
            if (content.includes("com.seattlesolvers.solverslib")) usesSolversLib = true;

            // FTCLib imports (legacy)
            if (content.includes("com.arcrobotics.ftclib")) usesFtcLib = true;

            // Command-based pattern detection
            if (content.includes("CommandOpMode") || content.includes("SubsystemBase") || content.includes("CommandBase")) {
              usesCommandBase = true;
            }

            // Constants detection
            if (content.includes("FollowerConstants")) hasFollowerConstants = true;
            if (content.includes("MecanumConstants")) hasMecanumConstants = true;
          }
        }

        // Deduplicate hardware devices
        const deviceMap = new Map<string, { name: string; type: string; files: string[] }>();
        for (const dev of hardwareDevices) {
          const key = `${dev.name}::${dev.type}`;
          if (deviceMap.has(key)) {
            const existing = deviceMap.get(key)!;
            if (!existing.files.includes(dev.file)) {
              existing.files.push(dev.file);
            }
          } else {
            deviceMap.set(key, { name: dev.name, type: dev.type, files: [dev.file] });
          }
        }

        result.javaFiles = javaFiles;
        result.opModes = opModes;
        result.configClasses = configClasses;
        result.hardwareDevices = Array.from(deviceMap.values());
        result.usesPedro = usesPedro;
        result.usesRoadRunner = usesRoadRunner;
        result.usesSolversLib = usesSolversLib;
        result.usesFtcLib = usesFtcLib;
        result.usesCommandBase = usesCommandBase;
        result.hasFollowerConstants = hasFollowerConstants;
        result.hasMecanumConstants = hasMecanumConstants;

        return {
          content: [{ type: "text" as const, text: JSON.stringify(result, null, 2) }],
        };
      } catch (error) {
        const message = error instanceof Error ? error.message : String(error);
        return {
          content: [{ type: "text" as const, text: `Error scanning project: ${message}\n\nMake sure the projectPath points to the FTC Robot Controller project root (the directory containing build.dependencies.gradle and the TeamCode/ folder).` }],
        };
      }
    }
  );

  // ══════════════════════════════════════════════════════════════════════════
  // Tool 2: scan_hardware_config
  // ══════════════════════════════════════════════════════════════════════════

  server.tool(
    "scan_hardware_config",
    "Parse all Java files to extract hardware device names, types, and usage from hardwareMap.get() calls and Constants files.",
    {
      projectPath: z.string().describe("Path to the FTC project root directory"),
    },
    async ({ projectPath }) => {
      try {
        const teamCodeDir = path.join(
          projectPath,
          "TeamCode/src/main/java/org/firstinspires/ftc/teamcode"
        );

        if (!fs.existsSync(teamCodeDir)) {
          return {
            content: [{ type: "text" as const, text: `TeamCode directory not found at: ${teamCodeDir}\n\nMake sure projectPath points to the FTC project root.` }],
          };
        }

        const files = getJavaFiles(teamCodeDir);
        const deviceMap = new Map<string, { name: string; type: string; files: string[] }>();

        for (const filePath of files) {
          const content = readFileSafe(filePath);
          if (!content) continue;
          const relativePath = path.relative(projectPath, filePath);

          // hardwareMap.get(Type.class, "name")
          const hwRegex = /hardwareMap\.get\(\s*(\w+)\.class\s*,\s*"([^"]+)"\s*\)/g;
          let match;
          while ((match = hwRegex.exec(content)) !== null) {
            const key = `${match[2]}::${match[1]}`;
            if (deviceMap.has(key)) {
              const existing = deviceMap.get(key)!;
              if (!existing.files.includes(relativePath)) {
                existing.files.push(relativePath);
              }
            } else {
              deviceMap.set(key, { name: match[2], type: match[1], files: [relativePath] });
            }
          }

          // Old-style: hardwareMap.dcMotor.get("name")
          const oldHwRegex = /hardwareMap\.(\w+)\.get\("([^"]+)"\)/g;
          let oldMatch;
          while ((oldMatch = oldHwRegex.exec(content)) !== null) {
            const key = `${oldMatch[2]}::${oldMatch[1]}`;
            if (deviceMap.has(key)) {
              const existing = deviceMap.get(key)!;
              if (!existing.files.includes(relativePath)) {
                existing.files.push(relativePath);
              }
            } else {
              deviceMap.set(key, { name: oldMatch[2], type: oldMatch[1], files: [relativePath] });
            }
          }

          // Pedro Constants motor name patterns
          const pedroMotorRegex = /\.(\w+MotorName)\("([^"]+)"\)/g;
          let pedroMatch;
          while ((pedroMatch = pedroMotorRegex.exec(content)) !== null) {
            const key = `${pedroMatch[2]}::DcMotorEx (Pedro ${pedroMatch[1]})`;
            if (!deviceMap.has(key)) {
              deviceMap.set(key, { name: pedroMatch[2], type: `DcMotorEx (Pedro ${pedroMatch[1]})`, files: [relativePath] });
            }
          }

          // Pedro Constants device name patterns (e.g., .setDeviceName("pinpoint"))
          const pedroDeviceRegex = /\.(?:setDeviceName|setHardwareMapName|hardwareMapName)\("([^"]+)"\)/g;
          let pedroDevMatch;
          while ((pedroDevMatch = pedroDeviceRegex.exec(content)) !== null) {
            const key = `${pedroDevMatch[1]}::PedroDevice`;
            if (!deviceMap.has(key)) {
              deviceMap.set(key, { name: pedroDevMatch[1], type: "Pedro Localizer Device", files: [relativePath] });
            }
          }

          // Pedro v1 static field style: FollowerConstants.leftFrontMotorName = "name"
          const pedroStaticRegex = /(\w+MotorName)\s*=\s*"([^"]+)"/g;
          let pedroStaticMatch;
          while ((pedroStaticMatch = pedroStaticRegex.exec(content)) !== null) {
            const key = `${pedroStaticMatch[2]}::DcMotorEx (Pedro ${pedroStaticMatch[1]})`;
            if (!deviceMap.has(key)) {
              deviceMap.set(key, { name: pedroStaticMatch[2], type: `DcMotorEx (Pedro ${pedroStaticMatch[1]})`, files: [relativePath] });
            }
          }
        }

        const devices = Array.from(deviceMap.values());

        return {
          content: [{
            type: "text" as const,
            text: JSON.stringify({ devices, totalFiles: files.length }, null, 2),
          }],
        };
      } catch (error) {
        const message = error instanceof Error ? error.message : String(error);
        return {
          content: [{ type: "text" as const, text: `Error scanning hardware config: ${message}` }],
        };
      }
    }
  );

  // ══════════════════════════════════════════════════════════════════════════
  // Tool 3: search_ftc_docs
  // ══════════════════════════════════════════════════════════════════════════

  server.tool(
    "search_ftc_docs",
    "Search the FTC knowledge base for documentation matching a query. Returns the most relevant sections.",
    {
      query: z.string().describe("Search query"),
      category: z.string().optional().describe("Category filter: sdk, pedro, roadrunner, dashboard, gradle, hardware, performance, command-base, ftclib, all. Covers build/deploy, IDE setup, ADB, and dev environment topics under sdk and gradle categories."),
    },
    async ({ query, category }) => {
      // Build search index from all knowledge modules
      const index: Array<{ category: string; key: string; content: string }> = [];

      for (const [key, content] of Object.entries(FTC_SDK_KNOWLEDGE)) {
        index.push({ category: "sdk", key, content });
      }
      for (const [key, content] of Object.entries(PEDRO_KNOWLEDGE)) {
        index.push({ category: "pedro", key, content });
      }
      for (const [key, content] of Object.entries(DASHBOARD_KNOWLEDGE)) {
        index.push({ category: "dashboard", key, content });
      }
      for (const [key, content] of Object.entries(HARDWARE_KNOWLEDGE)) {
        index.push({ category: "hardware", key, content });
      }
      for (const [key, content] of Object.entries(GRADLE_KNOWLEDGE)) {
        index.push({ category: "gradle", key, content });
      }
      for (const [key, content] of Object.entries(ROADRUNNER_KNOWLEDGE)) {
        index.push({ category: "roadrunner", key, content });
      }
      for (const [key, content] of Object.entries(FTCLIB_KNOWLEDGE)) {
        index.push({ category: "command-base", key, content });
      }

      // Split query into lowercase words
      const queryWords = query.toLowerCase().split(/\s+/).filter(w => w.length > 0);

      // Score each entry
      let scored = index.map(entry => {
        const contentLower = entry.content.toLowerCase();
        let score = 0;
        for (const word of queryWords) {
          if (contentLower.includes(word)) {
            score++;
          }
        }
        return { ...entry, score };
      });

      // Filter by category if specified
      if (category && category !== "all") {
        scored = scored.filter(entry => entry.category === category);
      }

      // Sort by score descending
      scored.sort((a, b) => b.score - a.score);

      // Return top 3
      const top = scored.slice(0, 3).filter(entry => entry.score > 0);

      if (top.length === 0) {
        return {
          content: [{
            type: "text" as const,
            text: `No matches found for query: "${query}"${category ? ` in category: ${category}` : ""}.\n\nAvailable categories: sdk, pedro, roadrunner, dashboard, gradle, hardware, ftclib, all`,
          }],
        };
      }

      const results = top.map((entry, i) =>
        `--- Result ${i + 1} [${entry.category}/${entry.key}] (score: ${entry.score}/${queryWords.length}) ---\n\n${entry.content}`
      ).join("\n\n");

      return {
        content: [{ type: "text" as const, text: results }],
      };
    }
  );

  // ══════════════════════════════════════════════════════════════════════════
  // Tool 4: get_ftc_example
  // ══════════════════════════════════════════════════════════════════════════

  server.tool(
    "get_ftc_example",
    "Get a complete, compilable Java code example for an FTC topic. Returns a full working file.",
    {
      topic: z.string().describe("Example topic: pedro-auto, pedro-teleop, pedro-constants, dashboard-config, bulk-reads, subsystem, pid-tuning, vision-pipeline, custom-pid-drive, field-centric-drive, command-teleop, command-auto, command-subsystem"),
    },
    async ({ topic }) => {
      const example = EXAMPLES[topic];

      if (example) {
        return {
          content: [{ type: "text" as const, text: example }],
        };
      }

      const available = Object.keys(EXAMPLES).join(", ");
      return {
        content: [{
          type: "text" as const,
          text: `Example not found for topic: "${topic}".\n\nAvailable topics: ${available}`,
        }],
      };
    }
  );

  // ══════════════════════════════════════════════════════════════════════════
  // Tool 5: get_hardware_reference
  // ══════════════════════════════════════════════════════════════════════════

  server.tool(
    "get_hardware_reference",
    "Get the complete API reference for a specific FTC hardware device or system.",
    {
      device: z.string().describe("Hardware device or system: DcMotorEx, Servo, CRServo, IMU, Pinpoint, OTOS, Limelight, ColorSensor, DistanceSensor, TouchSensor, bulk-reads, caching-hardware, custom-wrappers, encoders, rev-hub, vision"),
    },
    async ({ device }) => {
      const deviceLower = device.toLowerCase();

      const deviceMapping: Record<string, string[]> = {
        "dcmotor":          ["motorsApi", "motorRunModes", "motorSpecs"],
        "dcmotorex":        ["motorsApi", "motorRunModes", "motorSpecs"],
        "motor":            ["motorsApi", "motorRunModes", "motorSpecs"],
        "motors":           ["motorsApi", "motorRunModes", "motorSpecs"],
        "servo":            ["servosApi"],
        "crservo":          ["servosApi"],
        "imu":              ["sensorsImu"],
        "pinpoint":         ["pinpoint"],
        "otos":             ["otos"],
        "limelight":        ["vision"],
        "colorsensor":      ["sensorsColor"],
        "color":            ["sensorsColor"],
        "distancesensor":   ["sensorsDistance"],
        "distance":         ["sensorsDistance"],
        "touchsensor":      ["sensorsDigital"],
        "touch":            ["sensorsDigital"],
        "digital":          ["sensorsDigital"],
        "bulk-reads":       ["bulkReads"],
        "bulkread":         ["bulkReads"],
        "bulkreads":        ["bulkReads"],
        "bulk-read":        ["bulkReads"],
        "caching":          ["cachingHardware"],
        "caching-hardware": ["cachingHardware"],
        "cachinghardware":  ["cachingHardware"],
        "custom-wrappers":  ["customWrappers"],
        "wrappers":         ["customWrappers"],
        "customwrappers":   ["customWrappers"],
        "encoder":          ["sensorsEncoder"],
        "encoders":         ["sensorsEncoder"],
        "rev-hub":          ["revHub"],
        "revhub":           ["revHub"],
        "lynx":             ["revHub"],
        "vision":           ["vision"],
        "apriltag":         ["vision"],
        "camera":           ["vision"],
        "optimization":     ["optimizationSummary"],
        "pipeline":         ["commandPipeline"],
        "command-pipeline": ["commandPipeline"],
        "commandpipeline":  ["commandPipeline"],
        "lynxcommand":      ["commandPipeline"],
        "lynx-command":     ["commandPipeline"],
        "usb":              ["commandPipeline"],
        "write-optimization": ["writeOptimization"],
        "writeoptimization":  ["writeOptimization"],
        "write-caching":    ["writeOptimization", "cachingHardware"],
        "queueing":         ["commandPipeline", "writeOptimization"],
        "queuing":          ["commandPipeline", "writeOptimization"],
        "loop-time":        ["loopTimeBudget"],
        "looptime":         ["loopTimeBudget"],
        "loop-budget":      ["loopTimeBudget"],
        "performance":      ["optimizationSummary", "commandPipeline", "writeOptimization", "loopTimeBudget"],
      };

      const keys = deviceMapping[deviceLower];

      if (keys) {
        const hw = HARDWARE_KNOWLEDGE as Record<string, string>;
        const sections = keys.map(k => hw[k]).filter(Boolean);
        return {
          content: [{ type: "text" as const, text: sections.join("\n\n---\n\n") }],
        };
      }

      const availableDevices = [
        "DcMotorEx / motor", "Servo / CRServo", "IMU",
        "Pinpoint", "OTOS", "Limelight",
        "ColorSensor", "DistanceSensor", "TouchSensor",
        "bulk-reads", "caching-hardware", "custom-wrappers",
        "encoders", "rev-hub", "vision", "optimization",
        "command-pipeline / queueing", "write-optimization", "loop-time / performance",
      ];
      return {
        content: [{
          type: "text" as const,
          text: `No reference found for device: "${device}".\n\nAvailable devices:\n${availableDevices.map(d => `  - ${d}`).join("\n")}`,
        }],
      };
    }
  );

  // ══════════════════════════════════════════════════════════════════════════
  // Tool 6: validate_ftc_code
  // ══════════════════════════════════════════════════════════════════════════

  server.tool(
    "validate_ftc_code",
    "Check FTC Java code for common mistakes: wrong APIs, missing update() calls, caching @Config values, wrong patterns.",
    {
      code: z.string().describe("Java code to validate"),
      context: z.string().optional().describe("Additional context about the code"),
    },
    async ({ code }) => {
      const issues: string[] = [];

      // 1. Pedro imports but missing follower.update()
      if (
        code.includes("com.pedropathing") &&
        !code.includes("follower.update()")
      ) {
        issues.push(
          "[CRITICAL] Missing follower.update() in loop — Pedro Pathing requires follower.update() to be called every loop iteration for path following and localization to work."
        );
      }

      // 2. @Config vars cached in instance field (assigned in constructor or init)
      if (code.includes("@Config")) {
        // Look for patterns like: this.something = STATIC_FIELD or instanceField = ClassName.STATIC
        // in init() or constructor contexts
        const cachePattern = /(?:this\.\w+\s*=\s*\w+\.\w+|(?:private|protected)\s+\w+\s+\w+\s*=\s*\w+\.\w+)\s*;/;
        // More specific: look for instance field assignment from a static in init() or constructor
        const initBlock = code.match(/(?:public\s+void\s+init\s*\(\)|public\s+\w+\s*\([^)]*\)\s*\{)([\s\S]*?)(?:\n\s*\}|\n\s*public)/);
        if (initBlock) {
          const initContent = initBlock[1];
          // Check if init assigns static-looking values to instance fields
          if (/\w+\s*=\s*[A-Z][A-Z_]+/.test(initContent)) {
            issues.push(
              "[WARNING] Possible copy semantics issue: @Config values appear to be cached in init(). Read @Config static fields at point of use in loop() so dashboard edits take effect immediately."
            );
          }
        }
      }

      // 3. public static final with @Config
      if (code.includes("@Config") && /public\s+static\s+final\s+/.test(code)) {
        issues.push(
          "[ERROR] @Config fields must NOT be final — the dashboard modifies them at runtime via reflection. Remove the 'final' keyword."
        );
      }

      // 4. Thread.sleep in iterative OpMode (not LinearOpMode)
      if (
        code.includes("Thread.sleep") &&
        /extends\s+OpMode\b/.test(code) &&
        !/extends\s+LinearOpMode/.test(code)
      ) {
        issues.push(
          "[CRITICAL] Thread.sleep() in an iterative OpMode will freeze the entire robot (including the watchdog timer). Use a timer/state machine pattern instead. Thread.sleep() is only safe inside LinearOpMode.runOpMode()."
        );
      }

      // 5. setMode(RUN_TO_POSITION) without setTargetPosition before it
      const rtpIndex = code.indexOf("RUN_TO_POSITION");
      if (rtpIndex !== -1) {
        const beforeRtp = code.substring(0, rtpIndex);
        const lastSetTarget = beforeRtp.lastIndexOf("setTargetPosition");
        const lastSetMode = beforeRtp.lastIndexOf("setMode");
        // If setTargetPosition doesn't appear before RUN_TO_POSITION, or the last setMode is more
        // recent than the last setTargetPosition, flag it
        if (lastSetTarget === -1) {
          issues.push(
            "[ERROR] Must call setTargetPosition() BEFORE setMode(RUN_TO_POSITION). The motor needs a target before entering RUN_TO_POSITION mode."
          );
        }
      }

      // 6. Pedro v1 imports
      if (
        code.includes("org.firstinspires.ftc.teamcode.pedroPathing") ||
        code.includes("org.firstinspires.ftc.teamcode.pedropathing")
      ) {
        issues.push(
          "[ERROR] Using old Pedro Pathing v1.x imports (org.firstinspires.ftc.teamcode.pedroPathing). Update to v2.0 imports: com.pedropathing.*"
        );
      }

      // 7. MANUAL bulk caching without clearBulkCache()
      if (
        code.includes("BulkCachingMode.MANUAL") &&
        !code.includes("clearBulkCache()")
      ) {
        issues.push(
          "[CRITICAL] MANUAL bulk caching mode is set but clearBulkCache() is never called. This will return stale (cached) data from the first read forever. Add clearBulkCache() at the start of each loop iteration."
        );
      }

      // 8. Dashboard imported but no MultipleTelemetry
      if (
        (code.includes("com.acmerobotics.dashboard") || code.includes("FtcDashboard")) &&
        !code.includes("MultipleTelemetry")
      ) {
        issues.push(
          "[SUGGESTION] FTC Dashboard is imported but MultipleTelemetry is not used. Consider wrapping telemetry with MultipleTelemetry to send data to both the Driver Station and Dashboard simultaneously."
        );
      }

      // 9. Gamepad Y axis not negated
      // Look for direct use of gamepad*.left_stick_y for forward/drive power without negation
      const gamepadYDirectRegex = /(?:drive|forward|y|power)\s*=\s*gamepad[12]\.left_stick_y\b/;
      if (gamepadYDirectRegex.test(code)) {
        issues.push(
          "[WARNING] Gamepad Y axis is inverted in FTC — pushing the stick forward returns a negative value. Use -gamepad1.left_stick_y for intuitive forward control."
        );
      }
      // Also check for setPower(gamepad1.left_stick_y) pattern directly on motors
      if (/\.setPower\(\s*gamepad[12]\.left_stick_y\s*\)/.test(code)) {
        issues.push(
          "[WARNING] Gamepad Y axis is inverted — use -gamepad1.left_stick_y for forward. Currently passing raw (inverted) value to setPower()."
        );
      }

      // 10. SolversLib + FTCLib coexistence
      if (
        code.includes("com.seattlesolvers.solverslib") &&
        code.includes("com.arcrobotics.ftclib")
      ) {
        issues.push(
          "[ERROR] SolversLib and FTCLib imports found in the same file. These libraries CANNOT coexist — they share the same class names in different packages. Use one or the other. SolversLib (com.seattlesolvers.solverslib) is recommended as the actively maintained fork."
        );
      }

      // 11. CommandOpMode without super.run()
      if (
        code.includes("CommandOpMode") &&
        /public\s+void\s+run\s*\(\)/.test(code) &&
        !code.includes("super.run()")
      ) {
        issues.push(
          "[CRITICAL] CommandOpMode.run() is overridden but super.run() is not called. The CommandScheduler will not execute — no commands or subsystem periodic() methods will run. Add super.run() at the start of your run() method."
        );
      }

      // 12. SubsystemBase without register() or addRequirements()
      if (
        code.includes("extends SubsystemBase") &&
        !code.includes("register(") &&
        !code.includes("register()")
      ) {
        issues.push(
          "[WARNING] SubsystemBase is extended but register() is never called. The subsystem won't be registered with the CommandScheduler, so periodic() won't run. Call register() in the constructor or register the subsystem in your OpMode's initialize()."
        );
      }

      // 13. CommandBase without addRequirements()
      if (
        code.includes("extends CommandBase") &&
        !code.includes("addRequirements(")
      ) {
        issues.push(
          "[WARNING] CommandBase is extended but addRequirements() is never called. Without declaring subsystem requirements, the scheduler cannot prevent conflicting commands from running simultaneously on the same subsystem."
        );
      }

      if (issues.length === 0) {
        return {
          content: [{
            type: "text" as const,
            text: "No issues found. The code passes all common FTC validation checks.",
          }],
        };
      }

      const report = `Found ${issues.length} issue(s):\n\n${issues.map((issue, i) => `${i + 1}. ${issue}`).join("\n\n")}`;
      return {
        content: [{ type: "text" as const, text: report }],
      };
    }
  );
}
