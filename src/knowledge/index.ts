/**
 * Centralized knowledge index for progressive discovery.
 *
 * Inspired by Cloudflare's Code Mode pattern: instead of enumerating every
 * possible value in tool descriptions (which wastes context tokens), this
 * module provides runtime lookup functions. Tool descriptions stay compact
 * and options are returned dynamically when needed.
 */

import { PEDRO_KNOWLEDGE } from "./pedro.js";
import { DASHBOARD_KNOWLEDGE } from "./dashboard.js";
import { HARDWARE_KNOWLEDGE } from "./hardware.js";
import { FTC_SDK_KNOWLEDGE } from "./ftc-sdk.js";
import { GRADLE_KNOWLEDGE } from "./gradle.js";
import { ROADRUNNER_KNOWLEDGE } from "./roadrunner.js";
import { FTCLIB_KNOWLEDGE } from "./ftclib.js";
import { VISION_KNOWLEDGE } from "./vision.js";
import { PANELS_KNOWLEDGE } from "./panels.js";
import { EXAMPLES } from "./examples.js";

// ── Types ───────────────────────────────────────────────────────────────

interface ResourceEntry {
  name: string;
  key: string;
}

interface CategoryDef {
  source: Record<string, string>;
  label: string;
  entries: Record<string, ResourceEntry>;
}

// ── Category Index ──────────────────────────────────────────────────────
// Maps: category → slug → { name, knowledgeKey }
// All hardware URIs are flattened to ftc://{category}/{slug} (no nested paths)

export const CATEGORIES: Record<string, CategoryDef> = {
  sdk: {
    source: FTC_SDK_KNOWLEDGE as unknown as Record<string, string>,
    label: "FTC SDK",
    entries: {
      "overview":         { name: "FTC Overview",           key: "overview" },
      "opmode-patterns":  { name: "OpMode Patterns",        key: "opmodePatterns" },
      "hardware-map":     { name: "Hardware Map",            key: "hardwareMap" },
      "gamepad-api":      { name: "Gamepad API",             key: "gamepadApi" },
      "best-practices":   { name: "Best Practices",          key: "bestPractices" },
      "dev-environment":  { name: "Dev Environment Setup",   key: "devEnvironment" },
    },
  },
  pedro: {
    source: PEDRO_KNOWLEDGE as unknown as Record<string, string>,
    label: "Pedro Pathing",
    entries: {
      "api-reference":      { name: "Pedro API Reference",      key: "apiReference" },
      "constants-pattern":  { name: "Pedro Constants Pattern",   key: "constantsPattern" },
      "coordinate-system":  { name: "Pedro Coordinate System",   key: "coordinateSystem" },
      "auto-structure":     { name: "Pedro Auto Structure",      key: "autoStructure" },
      "teleop-structure":   { name: "Pedro TeleOp Structure",    key: "teleopStructure" },
      "callbacks":          { name: "Pedro Callbacks",            key: "callbacks" },
    },
  },
  dashboard: {
    source: DASHBOARD_KNOWLEDGE as unknown as Record<string, string>,
    label: "FTC Dashboard",
    entries: {
      "config-pattern": { name: "Dashboard Config Pattern", key: "configPattern" },
      "telemetry":      { name: "Dashboard Telemetry",      key: "telemetry" },
      "canvas":         { name: "Dashboard Canvas",          key: "canvas" },
      "camera":         { name: "Dashboard Camera",          key: "camera" },
      "setup":          { name: "Dashboard Setup",           key: "setup" },
      "api":            { name: "Dashboard API",             key: "api" },
    },
  },
  gradle: {
    source: GRADLE_KNOWLEDGE as unknown as Record<string, string>,
    label: "Gradle",
    entries: {
      "project-structure":  { name: "Gradle Project Structure",  key: "projectStructure" },
      "adding-libraries":   { name: "Adding Libraries",          key: "addingLibraries" },
      "all-library-coords": { name: "All Library Coordinates",   key: "allLibraryCoords" },
      "common-issues":      { name: "Common Gradle Issues",      key: "commonIssues" },
      "build-process":      { name: "Build Process",             key: "buildProcess" },
      "build-and-deploy":   { name: "Build and Deploy",          key: "buildAndDeploy" },
    },
  },
  hardware: {
    source: HARDWARE_KNOWLEDGE as unknown as Record<string, string>,
    label: "Hardware",
    entries: {
      "motors-api":            { name: "Motors API",              key: "motorsApi" },
      "motor-run-modes":       { name: "Motor Run Modes",         key: "motorRunModes" },
      "motor-specs":           { name: "Motor Specs",             key: "motorSpecs" },
      "servos-api":            { name: "Servos API",              key: "servosApi" },
      "sensors-imu":           { name: "IMU Sensor",              key: "sensorsImu" },
      "sensors-distance":      { name: "Distance Sensor",         key: "sensorsDistance" },
      "sensors-color":         { name: "Color Sensor",            key: "sensorsColor" },
      "sensors-digital":       { name: "Digital Sensors",          key: "sensorsDigital" },
      "sensors-encoder":       { name: "Encoders",                 key: "sensorsEncoder" },
      "pinpoint":              { name: "GoBilda Pinpoint",         key: "pinpoint" },
      "otos":                  { name: "SparkFun OTOS",            key: "otos" },
      "rev-hub":               { name: "REV Hub",                  key: "revHub" },
      "bulk-reads":            { name: "Bulk Reads",               key: "bulkReads" },
      "caching-hardware":      { name: "Caching Hardware",         key: "cachingHardware" },
      "optimization-summary":  { name: "Optimization Summary",     key: "optimizationSummary" },
      "custom-wrappers":       { name: "Custom Wrappers",          key: "customWrappers" },
      "command-pipeline":      { name: "Command Pipeline",         key: "commandPipeline" },
      "write-optimization":    { name: "Write Optimization",       key: "writeOptimization" },
      "loop-time-budget":      { name: "Loop Time Budget",         key: "loopTimeBudget" },
      "vision":                { name: "Vision",                    key: "vision" },
    },
  },
  roadrunner: {
    source: ROADRUNNER_KNOWLEDGE as unknown as Record<string, string>,
    label: "Road Runner",
    entries: {
      "api-reference": { name: "Road Runner API", key: "apiReference" },
    },
  },
  "command-base": {
    source: FTCLIB_KNOWLEDGE as unknown as Record<string, string>,
    label: "Command-Base (SolversLib)",
    entries: {
      "setup":              { name: "Command-Base Setup",               key: "setup" },
      "api":                { name: "Command-Base API",                 key: "apiReference" },
      "subsystems":         { name: "Command-Base Subsystem Patterns",  key: "subsystemPatterns" },
      "commands":           { name: "Command-Base Command Patterns",    key: "commandPatterns" },
      "triggers":           { name: "Command-Base Triggers & GamepadEx", key: "triggers" },
      "organization":       { name: "Command-Base Project Organization", key: "organization" },
      "pedro-integration":  { name: "Command-Base Pedro Integration",   key: "pedroIntegration" },
      "extras":             { name: "Command-Base SolversLib Extras",   key: "extras" },
    },
  },
  vision: {
    source: VISION_KNOWLEDGE as unknown as Record<string, string>,
    label: "Vision",
    entries: {
      "overview":            { name: "Vision Overview",          key: "overview" },
      "visionportal-setup":  { name: "VisionPortal Setup",      key: "visionPortalSetup" },
      "apriltag-detection":  { name: "AprilTag Detection",       key: "aprilTagDetection" },
      "camera-controls":     { name: "Camera Controls",          key: "cameraControls" },
      "limelight":           { name: "Limelight 3A Guide",       key: "limelight" },
      "megatag":             { name: "MegaTag Localization",     key: "megaTag" },
      "color-detection":     { name: "Color Detection",          key: "colorDetection" },
      "optimization":        { name: "Vision Optimization",      key: "visionOptimization" },
      "multi-camera":        { name: "Multi-Camera Setup",       key: "multiCamera" },
      "patterns":            { name: "Vision Patterns",          key: "visionPatterns" },
    },
  },
  panels: {
    source: PANELS_KNOWLEDGE as unknown as Record<string, string>,
    label: "Panels",
    entries: {
      "overview":       { name: "Panels Overview",       key: "overview" },
      "setup":          { name: "Panels Setup",          key: "setup" },
      "configurables":  { name: "Panels Configurables",  key: "configurables" },
      "telemetry":      { name: "Panels Telemetry",      key: "telemetry" },
      "field":          { name: "Panels Field Drawing",  key: "field" },
      "limelight":      { name: "Panels Limelight",      key: "limelight" },
      "plugins":        { name: "Panels Plugins",        key: "plugins" },
      "gamepads":       { name: "Panels Gamepads",       key: "gamepads" },
    },
  },
};

// ── Resource Lookup ─────────────────────────────────────────────────────

export function getResource(category: string, slug: string): string | null {
  const cat = CATEGORIES[category];
  if (!cat) return null;
  const entry = cat.entries[slug];
  if (!entry) return null;
  return cat.source[entry.key] ?? null;
}

export function listResources(category: string): Array<{ uri: string; name: string }> {
  const cat = CATEGORIES[category];
  if (!cat) return [];
  return Object.entries(cat.entries).map(([slug, entry]) => ({
    uri: `ftc://${category}/${slug}`,
    name: entry.name,
  }));
}

// ── Full-Text Search ────────────────────────────────────────────────────

export function buildSearchIndex(): Array<{ category: string; key: string; content: string }> {
  const index: Array<{ category: string; key: string; content: string }> = [];
  for (const [category, catDef] of Object.entries(CATEGORIES)) {
    for (const [slug, entry] of Object.entries(catDef.entries)) {
      const content = catDef.source[entry.key];
      if (content) {
        index.push({ category, key: slug, content });
      }
    }
  }
  return index;
}

// ── Example Lookup ──────────────────────────────────────────────────────

export function getExample(topic: string): string | null {
  return EXAMPLES[topic] ?? null;
}

export function listExamples(): string[] {
  return Object.keys(EXAMPLES);
}

// ── Hardware/Vision Device Alias Lookup ─────────────────────────────────
// Allows flexible device name queries (e.g., "DcMotorEx", "motor", "imu")

const HW_ALIASES: Record<string, string[]> = {
  "dcmotor":          ["motorsApi", "motorRunModes", "motorSpecs"],
  "dcmotorex":        ["motorsApi", "motorRunModes", "motorSpecs"],
  "motor":            ["motorsApi", "motorRunModes", "motorSpecs"],
  "motors":           ["motorsApi", "motorRunModes", "motorSpecs"],
  "servo":            ["servosApi"],
  "crservo":          ["servosApi"],
  "imu":              ["sensorsImu"],
  "pinpoint":         ["pinpoint"],
  "otos":             ["otos"],
  "colorsensor":      ["sensorsColor"],
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

const VISION_ALIASES: Record<string, string[]> = {
  "vision":               ["overview", "visionPortalSetup", "aprilTagDetection"],
  "visionportal":         ["visionPortalSetup"],
  "vision-portal":        ["visionPortalSetup"],
  "apriltag":             ["aprilTagDetection"],
  "april-tag":            ["aprilTagDetection"],
  "camera":               ["cameraControls"],
  "camera-controls":      ["cameraControls"],
  "cameracontrols":       ["cameraControls"],
  "exposure":             ["cameraControls"],
  "gain":                 ["cameraControls"],
  "focus":                ["cameraControls"],
  "white-balance":        ["cameraControls"],
  "whitebalance":         ["cameraControls"],
  "limelight":            ["limelight", "megaTag"],
  "limelight3a":          ["limelight", "megaTag"],
  "limelight-3a":         ["limelight", "megaTag"],
  "megatag":              ["megaTag"],
  "megatag2":             ["megaTag"],
  "mega-tag":             ["megaTag"],
  "localization":         ["megaTag"],
  "color-detection":      ["colorDetection"],
  "colordetection":       ["colorDetection"],
  "color-detect":         ["colorDetection"],
  "hsv":                  ["colorDetection"],
  "opencv":               ["colorDetection"],
  "contour":              ["colorDetection"],
  "visionprocessor":      ["colorDetection"],
  "vision-processor":     ["colorDetection"],
  "vision-optimization":  ["visionOptimization"],
  "visionoptimization":   ["visionOptimization"],
  "decimation":           ["visionOptimization", "aprilTagDetection"],
  "fps":                  ["visionOptimization"],
  "resolution":           ["visionOptimization"],
  "motion-blur":          ["visionOptimization", "cameraControls"],
  "multi-camera":         ["multiCamera"],
  "multicamera":          ["multiCamera"],
  "dual-camera":          ["multiCamera"],
  "dualcamera":           ["multiCamera"],
  "multiportal":          ["multiCamera"],
  "vision-patterns":      ["visionPatterns"],
  "visionpatterns":       ["visionPatterns"],
  "drive-to-tag":         ["visionPatterns"],
  "init-detection":       ["visionPatterns"],
  "snapscript":           ["megaTag"],
  "python-pipeline":      ["megaTag"],
};

export function lookupDeviceReference(device: string): string | null {
  const deviceLower = device.toLowerCase();

  // Check hardware aliases
  const hwKeys = HW_ALIASES[deviceLower];
  if (hwKeys) {
    const hw = HARDWARE_KNOWLEDGE as Record<string, string>;
    const sections = hwKeys.map(k => hw[k]).filter(Boolean);
    if (sections.length > 0) return sections.join("\n\n---\n\n");
  }

  // Check vision aliases
  const visionKeys = VISION_ALIASES[deviceLower];
  if (visionKeys) {
    const vis = VISION_KNOWLEDGE as Record<string, string>;
    const sections = visionKeys.map(k => vis[k]).filter(Boolean);
    if (sections.length > 0) return sections.join("\n\n---\n\n");
  }

  return null;
}
