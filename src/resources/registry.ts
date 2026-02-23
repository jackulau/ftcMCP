import { McpServer } from "@modelcontextprotocol/sdk/server/mcp.js";
import { PEDRO_KNOWLEDGE } from "../knowledge/pedro.js";
import { DASHBOARD_KNOWLEDGE } from "../knowledge/dashboard.js";
import { HARDWARE_KNOWLEDGE } from "../knowledge/hardware.js";
import { FTC_SDK_KNOWLEDGE } from "../knowledge/ftc-sdk.js";
import { GRADLE_KNOWLEDGE } from "../knowledge/gradle.js";
import { ROADRUNNER_KNOWLEDGE } from "../knowledge/roadrunner.js";
import { FTCLIB_KNOWLEDGE } from "../knowledge/ftclib.js";

export function registerResources(server: McpServer): void {
  // ── Core SDK (FTC_SDK_KNOWLEDGE) ──────────────────────────────────────

  server.resource(
    "FTC Overview",
    "ftc://sdk/overview",
    async (uri) => ({
      contents: [{
        uri: uri.href,
        mimeType: "text/plain",
        text: FTC_SDK_KNOWLEDGE.overview
      }]
    })
  );

  server.resource(
    "OpMode Patterns",
    "ftc://sdk/opmode-patterns",
    async (uri) => ({
      contents: [{
        uri: uri.href,
        mimeType: "text/plain",
        text: FTC_SDK_KNOWLEDGE.opmodePatterns
      }]
    })
  );

  server.resource(
    "Hardware Map",
    "ftc://sdk/hardware-map",
    async (uri) => ({
      contents: [{
        uri: uri.href,
        mimeType: "text/plain",
        text: FTC_SDK_KNOWLEDGE.hardwareMap
      }]
    })
  );

  server.resource(
    "Gamepad API",
    "ftc://sdk/gamepad-api",
    async (uri) => ({
      contents: [{
        uri: uri.href,
        mimeType: "text/plain",
        text: FTC_SDK_KNOWLEDGE.gamepadApi
      }]
    })
  );

  server.resource(
    "Best Practices",
    "ftc://sdk/best-practices",
    async (uri) => ({
      contents: [{
        uri: uri.href,
        mimeType: "text/plain",
        text: FTC_SDK_KNOWLEDGE.bestPractices
      }]
    })
  );

  // ── Pedro Pathing (PEDRO_KNOWLEDGE) ───────────────────────────────────

  server.resource(
    "Pedro API Reference",
    "ftc://pedro/api-reference",
    async (uri) => ({
      contents: [{
        uri: uri.href,
        mimeType: "text/plain",
        text: PEDRO_KNOWLEDGE.apiReference
      }]
    })
  );

  server.resource(
    "Pedro Constants Pattern",
    "ftc://pedro/constants-pattern",
    async (uri) => ({
      contents: [{
        uri: uri.href,
        mimeType: "text/plain",
        text: PEDRO_KNOWLEDGE.constantsPattern
      }]
    })
  );

  server.resource(
    "Pedro Coordinate System",
    "ftc://pedro/coordinate-system",
    async (uri) => ({
      contents: [{
        uri: uri.href,
        mimeType: "text/plain",
        text: PEDRO_KNOWLEDGE.coordinateSystem
      }]
    })
  );

  server.resource(
    "Pedro Auto Structure",
    "ftc://pedro/auto-structure",
    async (uri) => ({
      contents: [{
        uri: uri.href,
        mimeType: "text/plain",
        text: PEDRO_KNOWLEDGE.autoStructure
      }]
    })
  );

  server.resource(
    "Pedro TeleOp Structure",
    "ftc://pedro/teleop-structure",
    async (uri) => ({
      contents: [{
        uri: uri.href,
        mimeType: "text/plain",
        text: PEDRO_KNOWLEDGE.teleopStructure
      }]
    })
  );

  server.resource(
    "Pedro Callbacks",
    "ftc://pedro/callbacks",
    async (uri) => ({
      contents: [{
        uri: uri.href,
        mimeType: "text/plain",
        text: PEDRO_KNOWLEDGE.callbacks
      }]
    })
  );

  // ── Dashboard (DASHBOARD_KNOWLEDGE) ───────────────────────────────────

  server.resource(
    "Dashboard Config Pattern",
    "ftc://dashboard/config-pattern",
    async (uri) => ({
      contents: [{
        uri: uri.href,
        mimeType: "text/plain",
        text: DASHBOARD_KNOWLEDGE.configPattern
      }]
    })
  );

  server.resource(
    "Dashboard Telemetry",
    "ftc://dashboard/telemetry",
    async (uri) => ({
      contents: [{
        uri: uri.href,
        mimeType: "text/plain",
        text: DASHBOARD_KNOWLEDGE.telemetry
      }]
    })
  );

  server.resource(
    "Dashboard Canvas",
    "ftc://dashboard/canvas",
    async (uri) => ({
      contents: [{
        uri: uri.href,
        mimeType: "text/plain",
        text: DASHBOARD_KNOWLEDGE.canvas
      }]
    })
  );

  server.resource(
    "Dashboard Camera",
    "ftc://dashboard/camera",
    async (uri) => ({
      contents: [{
        uri: uri.href,
        mimeType: "text/plain",
        text: DASHBOARD_KNOWLEDGE.camera
      }]
    })
  );

  server.resource(
    "Dashboard Setup",
    "ftc://dashboard/setup",
    async (uri) => ({
      contents: [{
        uri: uri.href,
        mimeType: "text/plain",
        text: DASHBOARD_KNOWLEDGE.setup
      }]
    })
  );

  server.resource(
    "Dashboard API",
    "ftc://dashboard/api",
    async (uri) => ({
      contents: [{
        uri: uri.href,
        mimeType: "text/plain",
        text: DASHBOARD_KNOWLEDGE.api
      }]
    })
  );

  // ── Gradle (GRADLE_KNOWLEDGE) ─────────────────────────────────────────

  server.resource(
    "Gradle Project Structure",
    "ftc://gradle/project-structure",
    async (uri) => ({
      contents: [{
        uri: uri.href,
        mimeType: "text/plain",
        text: GRADLE_KNOWLEDGE.projectStructure
      }]
    })
  );

  server.resource(
    "Adding Libraries",
    "ftc://gradle/adding-libraries",
    async (uri) => ({
      contents: [{
        uri: uri.href,
        mimeType: "text/plain",
        text: GRADLE_KNOWLEDGE.addingLibraries
      }]
    })
  );

  server.resource(
    "All Library Coordinates",
    "ftc://gradle/all-library-coords",
    async (uri) => ({
      contents: [{
        uri: uri.href,
        mimeType: "text/plain",
        text: GRADLE_KNOWLEDGE.allLibraryCoords
      }]
    })
  );

  server.resource(
    "Common Gradle Issues",
    "ftc://gradle/common-issues",
    async (uri) => ({
      contents: [{
        uri: uri.href,
        mimeType: "text/plain",
        text: GRADLE_KNOWLEDGE.commonIssues
      }]
    })
  );

  server.resource(
    "Build Process",
    "ftc://gradle/build-process",
    async (uri) => ({
      contents: [{
        uri: uri.href,
        mimeType: "text/plain",
        text: GRADLE_KNOWLEDGE.buildProcess
      }]
    })
  );

  // ── Hardware (HARDWARE_KNOWLEDGE) ─────────────────────────────────────

  server.resource(
    "Motors API",
    "ftc://hardware/motors/api",
    async (uri) => ({
      contents: [{
        uri: uri.href,
        mimeType: "text/plain",
        text: HARDWARE_KNOWLEDGE.motorsApi
      }]
    })
  );

  server.resource(
    "Motor Run Modes",
    "ftc://hardware/motors/run-modes",
    async (uri) => ({
      contents: [{
        uri: uri.href,
        mimeType: "text/plain",
        text: HARDWARE_KNOWLEDGE.motorRunModes
      }]
    })
  );

  server.resource(
    "Motor Specs",
    "ftc://hardware/motors/specs",
    async (uri) => ({
      contents: [{
        uri: uri.href,
        mimeType: "text/plain",
        text: HARDWARE_KNOWLEDGE.motorSpecs
      }]
    })
  );

  server.resource(
    "Servos API",
    "ftc://hardware/servos/api",
    async (uri) => ({
      contents: [{
        uri: uri.href,
        mimeType: "text/plain",
        text: HARDWARE_KNOWLEDGE.servosApi
      }]
    })
  );

  server.resource(
    "IMU Sensor",
    "ftc://hardware/sensors/imu",
    async (uri) => ({
      contents: [{
        uri: uri.href,
        mimeType: "text/plain",
        text: HARDWARE_KNOWLEDGE.sensorsImu
      }]
    })
  );

  server.resource(
    "Distance Sensor",
    "ftc://hardware/sensors/distance",
    async (uri) => ({
      contents: [{
        uri: uri.href,
        mimeType: "text/plain",
        text: HARDWARE_KNOWLEDGE.sensorsDistance
      }]
    })
  );

  server.resource(
    "Color Sensor",
    "ftc://hardware/sensors/color",
    async (uri) => ({
      contents: [{
        uri: uri.href,
        mimeType: "text/plain",
        text: HARDWARE_KNOWLEDGE.sensorsColor
      }]
    })
  );

  server.resource(
    "Digital Sensors",
    "ftc://hardware/sensors/digital",
    async (uri) => ({
      contents: [{
        uri: uri.href,
        mimeType: "text/plain",
        text: HARDWARE_KNOWLEDGE.sensorsDigital
      }]
    })
  );

  server.resource(
    "Encoders",
    "ftc://hardware/sensors/encoder",
    async (uri) => ({
      contents: [{
        uri: uri.href,
        mimeType: "text/plain",
        text: HARDWARE_KNOWLEDGE.sensorsEncoder
      }]
    })
  );

  server.resource(
    "GoBilda Pinpoint",
    "ftc://hardware/pinpoint",
    async (uri) => ({
      contents: [{
        uri: uri.href,
        mimeType: "text/plain",
        text: HARDWARE_KNOWLEDGE.pinpoint
      }]
    })
  );

  server.resource(
    "SparkFun OTOS",
    "ftc://hardware/otos",
    async (uri) => ({
      contents: [{
        uri: uri.href,
        mimeType: "text/plain",
        text: HARDWARE_KNOWLEDGE.otos
      }]
    })
  );

  server.resource(
    "REV Hub",
    "ftc://hardware/rev-hub",
    async (uri) => ({
      contents: [{
        uri: uri.href,
        mimeType: "text/plain",
        text: HARDWARE_KNOWLEDGE.revHub
      }]
    })
  );

  server.resource(
    "Bulk Reads",
    "ftc://hardware/bulk-reads",
    async (uri) => ({
      contents: [{
        uri: uri.href,
        mimeType: "text/plain",
        text: HARDWARE_KNOWLEDGE.bulkReads
      }]
    })
  );

  server.resource(
    "Caching Hardware",
    "ftc://hardware/caching-hardware",
    async (uri) => ({
      contents: [{
        uri: uri.href,
        mimeType: "text/plain",
        text: HARDWARE_KNOWLEDGE.cachingHardware
      }]
    })
  );

  server.resource(
    "Optimization Summary",
    "ftc://hardware/optimization-summary",
    async (uri) => ({
      contents: [{
        uri: uri.href,
        mimeType: "text/plain",
        text: HARDWARE_KNOWLEDGE.optimizationSummary
      }]
    })
  );

  server.resource(
    "Custom Wrappers",
    "ftc://hardware/custom-wrappers",
    async (uri) => ({
      contents: [{
        uri: uri.href,
        mimeType: "text/plain",
        text: HARDWARE_KNOWLEDGE.customWrappers
      }]
    })
  );

  server.resource(
    "Command Pipeline",
    "ftc://hardware/command-pipeline",
    async (uri) => ({
      contents: [{
        uri: uri.href,
        mimeType: "text/plain",
        text: HARDWARE_KNOWLEDGE.commandPipeline
      }]
    })
  );

  server.resource(
    "Write Optimization",
    "ftc://hardware/write-optimization",
    async (uri) => ({
      contents: [{
        uri: uri.href,
        mimeType: "text/plain",
        text: HARDWARE_KNOWLEDGE.writeOptimization
      }]
    })
  );

  server.resource(
    "Loop Time Budget",
    "ftc://hardware/loop-time-budget",
    async (uri) => ({
      contents: [{
        uri: uri.href,
        mimeType: "text/plain",
        text: HARDWARE_KNOWLEDGE.loopTimeBudget
      }]
    })
  );

  server.resource(
    "Vision",
    "ftc://hardware/vision",
    async (uri) => ({
      contents: [{
        uri: uri.href,
        mimeType: "text/plain",
        text: HARDWARE_KNOWLEDGE.vision
      }]
    })
  );

  // ── Road Runner (ROADRUNNER_KNOWLEDGE) ────────────────────────────────

  server.resource(
    "Road Runner API",
    "ftc://roadrunner/api-reference",
    async (uri) => ({
      contents: [{
        uri: uri.href,
        mimeType: "text/plain",
        text: ROADRUNNER_KNOWLEDGE.apiReference
      }]
    })
  );

  // ── Command-Based / SolversLib (FTCLIB_KNOWLEDGE) ──────────────────────

  server.resource(
    "Command-Base Setup",
    "ftc://command-base/setup",
    async (uri) => ({
      contents: [{
        uri: uri.href,
        mimeType: "text/plain",
        text: FTCLIB_KNOWLEDGE.setup
      }]
    })
  );

  server.resource(
    "Command-Base API",
    "ftc://command-base/api",
    async (uri) => ({
      contents: [{
        uri: uri.href,
        mimeType: "text/plain",
        text: FTCLIB_KNOWLEDGE.apiReference
      }]
    })
  );

  server.resource(
    "Command-Base Subsystem Patterns",
    "ftc://command-base/subsystems",
    async (uri) => ({
      contents: [{
        uri: uri.href,
        mimeType: "text/plain",
        text: FTCLIB_KNOWLEDGE.subsystemPatterns
      }]
    })
  );

  server.resource(
    "Command-Base Command Patterns",
    "ftc://command-base/commands",
    async (uri) => ({
      contents: [{
        uri: uri.href,
        mimeType: "text/plain",
        text: FTCLIB_KNOWLEDGE.commandPatterns
      }]
    })
  );

  server.resource(
    "Command-Base Triggers & GamepadEx",
    "ftc://command-base/triggers",
    async (uri) => ({
      contents: [{
        uri: uri.href,
        mimeType: "text/plain",
        text: FTCLIB_KNOWLEDGE.triggers
      }]
    })
  );

  server.resource(
    "Command-Base Project Organization",
    "ftc://command-base/organization",
    async (uri) => ({
      contents: [{
        uri: uri.href,
        mimeType: "text/plain",
        text: FTCLIB_KNOWLEDGE.organization
      }]
    })
  );

  server.resource(
    "Command-Base Pedro Integration",
    "ftc://command-base/pedro-integration",
    async (uri) => ({
      contents: [{
        uri: uri.href,
        mimeType: "text/plain",
        text: FTCLIB_KNOWLEDGE.pedroIntegration
      }]
    })
  );

  server.resource(
    "Command-Base SolversLib Extras",
    "ftc://command-base/extras",
    async (uri) => ({
      contents: [{
        uri: uri.href,
        mimeType: "text/plain",
        text: FTCLIB_KNOWLEDGE.extras
      }]
    })
  );
}
