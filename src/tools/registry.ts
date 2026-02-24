/**
 * MCP Tool Registration — Context-Optimized
 *
 * Inspired by Cloudflare's Code Mode approach to reducing context window usage:
 * - Consolidated from 6 tools → 3 tools (fewer tool definitions in context)
 * - Compressed descriptions (no enumerated value lists in schemas)
 * - Progressive discovery (available options returned at runtime, not upfront)
 *
 * Tools:
 *   1. scan_project     — merged scan_ftc_project + scan_hardware_config
 *   2. search_knowledge — merged search_ftc_docs + get_ftc_example + get_hardware_reference
 *   3. validate_ftc_code — same logic, shorter description
 */

import { McpServer } from "@modelcontextprotocol/sdk/server/mcp.js";
import { z } from "zod";
import fs from "node:fs";
import path from "node:path";

import {
  buildSearchIndex,
  getExample,
  listExamples,
  lookupDeviceReference,
} from "../knowledge/index.js";

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
      } else if (entry.name.endsWith(".java") || entry.name.endsWith(".kt")) {
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
  // Tool 1: scan_project (merged scan_ftc_project + scan_hardware_config)
  // ══════════════════════════════════════════════════════════════════════════

  server.tool(
    "scan_project",
    "Scan FTC project for SDK version, libraries, OpModes, hardware devices, and structure. Use at session start.",
    {
      projectPath: z.string().describe("FTC project root path"),
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
          panels: false,
          roadrunner: false,
          cachinghardware: false,
          ftclib: false,
          solverslib: false,
        };
        if (depsGradle) {
          const depsLower = depsGradle.toLowerCase();
          if (depsLower.includes("pedropathing")) libraries.pedropathing = true;
          if (depsLower.includes("dashboard")) libraries.dashboard = true;
          if (depsLower.includes("fullpanels") || depsLower.includes("com.bylazar")) libraries.panels = true;
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

        // 5. Scan Java/Kotlin files (combined from both old scan tools)
        const opModes: Array<{ name: string; type: string; file: string; extendsClass: string }> = [];
        const configClasses: Array<{ name: string; file: string }> = [];
        const javaFiles: string[] = [];
        let usesPedro = false;
        let usesRoadRunner = false;
        let usesSolversLib = false;
        let usesFtcLib = false;
        let usesCommandBase = false;
        let usesPanels = false;
        let hasFollowerConstants = false;
        let hasMecanumConstants = false;

        // Combined device map (includes patterns from old scan_hardware_config)
        const deviceMap = new Map<string, { name: string; type: string; files: string[] }>();

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

            // hardwareMap.get(Type.class, "name")
            const hwRegex = /hardwareMap\.get\(\s*(\w+)\.class\s*,\s*"([^"]+)"\s*\)/g;
            let hwMatch;
            while ((hwMatch = hwRegex.exec(content)) !== null) {
              const key = `${hwMatch[2]}::${hwMatch[1]}`;
              if (deviceMap.has(key)) {
                const existing = deviceMap.get(key)!;
                if (!existing.files.includes(relativePath)) {
                  existing.files.push(relativePath);
                }
              } else {
                deviceMap.set(key, { name: hwMatch[2], type: hwMatch[1], files: [relativePath] });
              }
            }

            // Old-style: hardwareMap.dcMotor.get("name")
            const oldHwRegex = /hardwareMap\.(\w+)\.get\("([^"]+)"\)/g;
            let oldHwMatch;
            while ((oldHwMatch = oldHwRegex.exec(content)) !== null) {
              const key = `${oldHwMatch[2]}::${oldHwMatch[1]}`;
              if (deviceMap.has(key)) {
                const existing = deviceMap.get(key)!;
                if (!existing.files.includes(relativePath)) {
                  existing.files.push(relativePath);
                }
              } else {
                deviceMap.set(key, { name: oldHwMatch[2], type: oldHwMatch[1], files: [relativePath] });
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

            // Library imports detection
            if (content.includes("com.pedropathing")) usesPedro = true;
            if (content.includes("com.acmerobotics.roadrunner")) usesRoadRunner = true;
            if (content.includes("com.bylazar.panels") || content.includes("com.bylazar.telemetry") || content.includes("com.bylazar.configurables") || content.includes("com.bylazar.field")) usesPanels = true;
            if (content.includes("com.seattlesolvers.solverslib")) usesSolversLib = true;
            if (content.includes("com.arcrobotics.ftclib")) usesFtcLib = true;
            if (content.includes("CommandOpMode") || content.includes("SubsystemBase") || content.includes("CommandBase")) usesCommandBase = true;
            if (content.includes("FollowerConstants")) hasFollowerConstants = true;
            if (content.includes("MecanumConstants")) hasMecanumConstants = true;
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
        result.usesPanels = usesPanels;
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
  // Tool 2: search_knowledge
  //   Merged: search_ftc_docs + get_ftc_example + get_hardware_reference
  //   Progressive discovery: tries exact match → device lookup → full-text search
  // ══════════════════════════════════════════════════════════════════════════

  server.tool(
    "search_knowledge",
    "Search FTC knowledge base. Finds documentation, code examples, and hardware API references.",
    {
      query: z.string().describe("Search query, example topic, or device name"),
    },
    async ({ query }) => {
      // 1. Try exact match as a code example topic
      const example = getExample(query);
      if (example) {
        return {
          content: [{ type: "text" as const, text: `[Example: ${query}]\n\n${example}` }],
        };
      }

      // 2. Try hardware/vision device reference lookup
      const reference = lookupDeviceReference(query);
      if (reference) {
        return {
          content: [{ type: "text" as const, text: reference }],
        };
      }

      // 3. Full-text search across all knowledge
      const index = buildSearchIndex();
      const queryWords = query.toLowerCase().split(/\s+/).filter(w => w.length > 0);

      const scored = index.map(entry => {
        const contentLower = entry.content.toLowerCase();
        let score = 0;
        for (const word of queryWords) {
          if (contentLower.includes(word)) score++;
        }
        return { ...entry, score };
      });

      scored.sort((a, b) => b.score - a.score);
      const top = scored.slice(0, 3).filter(entry => entry.score > 0);

      if (top.length > 0) {
        const results = top.map((entry, i) =>
          `--- Result ${i + 1} [${entry.category}/${entry.key}] (score: ${entry.score}/${queryWords.length}) ---\n\n${entry.content}`
        ).join("\n\n");
        return {
          content: [{ type: "text" as const, text: results }],
        };
      }

      // 4. Nothing found — return available options (progressive discovery)
      const examples = listExamples();
      return {
        content: [{
          type: "text" as const,
          text: `No matches for: "${query}"\n\nTry:\n- Example topics: ${examples.join(", ")}\n- Device names: DcMotorEx, Servo, IMU, Pinpoint, OTOS, Limelight, bulk-reads, vision, etc.\n- Search terms: "pedro pathing", "dashboard setup", "vision pipeline", etc.`,
        }],
      };
    }
  );

  // ══════════════════════════════════════════════════════════════════════════
  // Tool 3: validate_ftc_code (same validation logic, compressed description)
  // ══════════════════════════════════════════════════════════════════════════

  server.tool(
    "validate_ftc_code",
    "Validate FTC Java code for common mistakes and anti-patterns.",
    {
      code: z.string().describe("Java code to validate"),
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
        const initBlock = code.match(/(?:public\s+void\s+init\s*\(\)|public\s+\w+\s*\([^)]*\)\s*\{)([\s\S]*?)(?:\n\s*\}|\n\s*public)/);
        if (initBlock) {
          const initContent = initBlock[1];
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
      const gamepadYDirectRegex = /(?:drive|forward|y|power)\s*=\s*gamepad[12]\.left_stick_y\b/;
      if (gamepadYDirectRegex.test(code)) {
        issues.push(
          "[WARNING] Gamepad Y axis is inverted in FTC — pushing the stick forward returns a negative value. Use -gamepad1.left_stick_y for intuitive forward control."
        );
      }
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

      // 12. SubsystemBase without register()
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
