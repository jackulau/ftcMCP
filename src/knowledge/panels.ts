export const PANELS_KNOWLEDGE = {
  overview: `
## Panels — All-in-One FTC Dashboard (by Lazar)

### What is Panels?
Panels (by Lazar, team 19234 ByteForce) is a modern, plugin-driven dashboard for FTC robots. It is an alternative to (and replacement for) FTC Dashboard (acmerobotics). Panels provides a richer feature set with a plugin architecture, wireless Limelight control, gamepad support, capture/replay, real-time configurables, field drawing, graphing, and more.

**Website:** https://panels.bylazar.com
**GitHub:** https://github.com/ftcontrol/ftcontrol-panels
**Current version:** 1.0.12 (FullPanels), 1.0.5 (Core)

### Key Differences from FTC Dashboard
| Feature | FTC Dashboard | Panels |
|---------|--------------|--------|
| Live variable tuning | \`@Config\` + \`public static\` | \`@Configurable\` annotation (supports more types) |
| Telemetry | MultipleTelemetry / TelemetryPacket | \`PanelsTelemetry.telemetry\` (drop-in) |
| Field overlay | Canvas API via TelemetryPacket | \`PanelsField.field\` with cursor-based drawing |
| Camera streaming | CameraStreamSource interface | Built-in CameraStream plugin |
| Limelight control | Not supported | Full wireless Limelight 3A dashboard |
| Gamepad | Single gamepad via browser | Up to 2 gamepads (Driver 1 + Driver 2) |
| Graphing | Auto-graph numeric telemetry | Dedicated Graph plugin |
| Capture/replay | Not supported | Record and replay debug data |
| Theming | Fixed UI | Full theme customization (colors, layout, widgets) |
| Architecture | Monolithic | Plugin-driven (extensible with custom Svelte + Kotlin plugins) |
| Coordinate presets | Fixed (field center origin) | Presets for Panels, Pedro Pathing, Road Runner |
| Battery monitoring | Not built-in | Battery plugin with real-time monitoring |
| Latency monitoring | Not built-in | Pinger plugin for input delay tracking |
| OpMode control | Basic start/stop | Full Driver Station-style with Autos/TeleOps sections + timers |

### When to Use Panels vs FTC Dashboard
- **Use Panels** if you want an all-in-one solution with Limelight support, capture/replay, richer configurables, gamepad, and plugin extensibility.
- **Use FTC Dashboard** if your team already has FTC Dashboard integrated and prefers the simpler \`@Config\` + \`MultipleTelemetry\` pattern.
- **Both are valid choices.** Panels is newer (v1.0) and actively developed. FTC Dashboard is more established and widely documented.
- **Do NOT use both simultaneously.** They both serve as web dashboards on the Control Hub and will conflict.

### Plugin Architecture
Panels is entirely plugin-driven. Built-in plugins include:
- **OpModeControl** — Driver Station-style OpMode management with Autos/TeleOps sections and timers
- **Telemetry** — Real-time telemetry display that mirrors Driver Hub telemetry
- **Configurables** — Live-tunable variables that update in real time (any data type)
- **Field** — Drawable canvas of the game field with mapped coordinates and pathing library presets
- **Graph** — Visualize live data over time for PID tuning and validation
- **Capture** — Record debug data and replay later for diagnosing intermittent issues
- **Lights** — goBILDA-style RGB indicator feedback
- **Battery** — Real-time battery level monitoring
- **Pinger** — Input delay monitoring
- **Gamepad** — Up to 2 FTC gamepads for wireless robot control
- **LimelightProxy** — Full Limelight 3A dashboard access with streaming, stats, and telemetry
- **CameraStream** — Optimized FTC webcam streaming
- **Themes** — Full UI customization (colors, layout, grid widgets, navbar navlets, tabbed groups)
- **Docs** — Integrated offline documentation
- **Utils** — Shared utilities

Custom plugins can be built with a Svelte frontend and Kotlin backend.
`,

  setup: `
## Panels Installation & Setup

### Gradle Configuration

**Step 1:** Add the Panels Maven repository to \`build.dependencies.gradle\`:
\`\`\`groovy
repositories {
    maven { url = 'https://mymaven.bylazar.com/releases' }
}
\`\`\`

**Step 2:** Add the FullPanels dependency (includes ALL plugins):
\`\`\`groovy
dependencies {
    implementation 'com.bylazar:fullpanels:1.0.12'
}
\`\`\`

Or for core Panels only (no built-in plugins):
\`\`\`groovy
dependencies {
    implementation 'com.bylazar:panels:1.0.5'
}
\`\`\`

**Step 3:** Ensure your \`build.common.gradle\` or TeamCode \`build.gradle\` has:
\`\`\`groovy
android {
    compileSdk 34   // Panels requires API 34+
    // ...
    compileOptions {
        sourceCompatibility JavaVersion.VERSION_11
        targetCompatibility JavaVersion.VERSION_11
    }
}
\`\`\`

**Step 4:** If using Kotlin (recommended), add the Kotlin plugin:
\`\`\`groovy
apply plugin: 'org.jetbrains.kotlin.android'

kotlinOptions {
    jvmTarget = '11'
}
\`\`\`

**Step 5:** Create a Config class that extends PanelsConfig:
\`\`\`kotlin
import com.bylazar.panels.PanelsConfig

class Config : PanelsConfig() {
    @Transient
    override var isDisabled = false
}
\`\`\`

### Accessing Panels
| Connection Method | URL |
|-------------------|-----|
| **Control Hub** (Wi-Fi Direct) | \`http://192.168.43.1:8080/panels\` |
| **Phone** (Wi-Fi Direct) | \`http://192.168.49.1:8080/panels\` |

Open the URL in any modern web browser on a device connected to the Robot Controller's Wi-Fi network.

### Java Support
Panels is developed in Kotlin but works in Java too. For Java users, the APIs are the same but you access singleton objects differently:
\`\`\`java
// Kotlin
val telemetry = PanelsTelemetry.telemetry;

// Java
Telemetry telemetry = PanelsTelemetry.INSTANCE.getTelemetry();
\`\`\`

### IMPORTANT: Panels vs FTC Dashboard
Do NOT install both Panels and FTC Dashboard. They are alternative dashboards — using both will create conflicts. If migrating from FTC Dashboard:
1. Remove the FTC Dashboard dependency (\`com.acmerobotics.dashboard:dashboard\`)
2. Remove the brott.dev Maven repository (if only used for dashboard)
3. Replace \`@Config\` with \`@Configurable\`
4. Replace \`MultipleTelemetry\` with \`PanelsTelemetry.telemetry\`
5. Replace \`TelemetryPacket.fieldOverlay()\` with \`PanelsField.field\`
`,

  configurables: `
## Panels Configurables — Live Variable Tuning

### @Configurable Annotation
Panels uses the \`@Configurable\` annotation (from \`com.bylazar.configurables.annotations\`) to expose variables for live tuning. This is the Panels equivalent of FTC Dashboard's \`@Config\`.

### Import
\`\`\`kotlin
import com.bylazar.configurables.annotations.Configurable
import com.bylazar.configurables.PanelsConfigurables
\`\`\`

### Basic Usage (Kotlin)
\`\`\`kotlin
@Configurable
object RobotConstants {
    @JvmField var DRIVE_SPEED = 0.8
    @JvmField var ENCODER_TICKS = 1120
    @JvmField var USE_GYRO = true
}
\`\`\`

### Usage on OpMode Classes (Kotlin)
\`\`\`kotlin
@Configurable
@TeleOp(name = "My TeleOp", group = "Dev")
class MyTeleOp : OpMode() {
    companion object {
        @JvmField var armPower = 0.5
        @JvmField var slowMode = false
    }

    override fun init() {
        PanelsConfigurables.refreshClass(this)
    }

    override fun loop() {
        // armPower and slowMode update live from the Panels dashboard
    }
}
\`\`\`

### Refreshing Configurables
Call \`PanelsConfigurables.refreshClass()\` to register a class's configurable values with the dashboard:
\`\`\`kotlin
// In init()
PanelsConfigurables.refreshClass(MyConstants)
PanelsConfigurables.refreshClass(this)  // for the OpMode itself if @Configurable
\`\`\`

### Supported Types
Panels Configurables supports a broader set of types than FTC Dashboard's @Config:
- Primitives: \`Int\`, \`Long\`, \`Double\`, \`Float\`, \`Boolean\`
- \`String\`
- \`enum\` types
- Arrays (including mixed-type arrays)
- Lists and Maps
- Custom objects (recursively expands fields)
- Nested types (deeply nested custom objects)
- Generic types (with \`@GenericValue\` annotation)
- Nullable types

### Custom Object Example
\`\`\`kotlin
@Configurable
object PIDConstants {
    class PIDCoefficients(
        val kP: Double,
        val kI: Double,
        val kD: Double
    )

    @JvmField var DRIVE_PID = PIDCoefficients(0.05, 0.0, 0.01)
    @JvmField var ARM_PID = PIDCoefficients(0.1, 0.0, 0.02)
}
\`\`\`

### Generic Type Support
For generic/parameterized types, use the \`@GenericValue\` annotation:
\`\`\`kotlin
import com.bylazar.configurables.annotations.GenericValue

class TParamClass<T>(val test: T)

@Configurable
object MyConfig {
    @JvmField
    @field:GenericValue(Int::class)
    var testTParamClass = TParamClass(1)
}
\`\`\`

### Java Usage
\`\`\`java
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.PanelsConfigurables;

@Configurable
public class RobotConstants {
    public static double DRIVE_SPEED = 0.8;
    public static int ENCODER_TICKS = 1120;
    public static boolean USE_GYRO = true;
}

// In your OpMode init:
PanelsConfigurables.INSTANCE.refreshClass(RobotConstants.class);
\`\`\`

### Key Differences from FTC Dashboard @Config
| FTC Dashboard | Panels |
|--------------|--------|
| \`@Config\` | \`@Configurable\` |
| \`public static\` fields only | \`@JvmField var\` (Kotlin) or \`public static\` (Java) |
| No arrays/maps/generics | Full support for arrays, lists, maps, generics |
| Reflection-based | Kotlin reflection with broader type support |
| \`FtcDashboard.getInstance()\` | \`PanelsConfigurables.refreshClass()\` |
`,

  telemetry: `
## Panels Telemetry

### Getting the Telemetry Instance
\`\`\`kotlin
import com.bylazar.telemetry.PanelsTelemetry

val telemetry = PanelsTelemetry.telemetry
\`\`\`

The \`PanelsTelemetry.telemetry\` object provides a telemetry interface that sends data to the Panels dashboard. It mirrors Driver Hub telemetry so data stays consistent.

### Basic Usage (Kotlin)
\`\`\`kotlin
import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp

@TeleOp(name = "Panels Telemetry Demo")
class TelemetryDemo : OpMode() {
    val telemetry = PanelsTelemetry.telemetry

    override fun init() {
        telemetry.debug("Init was called")
        telemetry.update()
    }

    override fun loop() {
        telemetry.debug("Loop at \${System.currentTimeMillis()}")
        telemetry.debug("Heading: \${getHeading()}")
        telemetry.update()  // Must call update() to send data
    }
}
\`\`\`

### Telemetry Methods
| Method | Description |
|--------|-------------|
| \`debug(message: String)\` | Add a debug/info line to telemetry |
| \`update()\` | Send all queued telemetry data to the dashboard |

### Configuring Telemetry Plugin
Create a config class extending TelemetryPluginConfig:
\`\`\`kotlin
import com.bylazar.telemetry.TelemetryPluginConfig

class TelemetryConfig : TelemetryPluginConfig() {
    @Transient
    override var telemetryUpdateInterval = 100L  // ms between updates
}
\`\`\`

### Java Usage
\`\`\`java
import com.bylazar.telemetry.PanelsTelemetry;

@TeleOp(name = "Panels Telemetry Demo")
public class TelemetryDemo extends OpMode {
    @Override
    public void init() {
        PanelsTelemetry.INSTANCE.getTelemetry().debug("Init");
        PanelsTelemetry.INSTANCE.getTelemetry().update();
    }

    @Override
    public void loop() {
        PanelsTelemetry.INSTANCE.getTelemetry().debug("Running");
        PanelsTelemetry.INSTANCE.getTelemetry().update();
    }
}
\`\`\`

### Migration from FTC Dashboard
If migrating from FTC Dashboard's MultipleTelemetry:
\`\`\`
// BEFORE (FTC Dashboard):
telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
telemetry.addData("Speed", speed);
telemetry.update();

// AFTER (Panels):
val telemetry = PanelsTelemetry.telemetry
telemetry.debug("Speed: $speed")
telemetry.update()
\`\`\`
`,

  field: `
## Panels Field — Canvas Drawing API

### Getting the Field Instance
\`\`\`kotlin
import com.bylazar.field.PanelsField

val field = PanelsField.field
\`\`\`

### Coordinate System Presets
Panels provides coordinate system presets for popular FTC pathing libraries:
\`\`\`kotlin
// Set coordinate offsets to match your pathing library
field.setOffsets(PanelsField.presets.PANELS)          // Panels default
field.setOffsets(PanelsField.presets.PEDRO_PATHING)   // Pedro Pathing coordinates
field.setOffsets(PanelsField.presets.ROAD_RUNNER)     // Road Runner coordinates
\`\`\`
This is a major advantage — no manual coordinate translation needed when using Pedro Pathing or Road Runner.

### Drawing API
The Panels Field API uses a cursor-based drawing model:

\`\`\`kotlin
// Move the drawing cursor to a position
field.moveCursor(x, y)

// Draw shapes at the cursor position
field.circle(radius)           // Draw a circle
field.rect(width, height)     // Draw a rectangle
field.line(endX, endY)        // Draw a line from cursor to (endX, endY)
field.img(width, height, id)  // Draw a registered image

// Style the drawings
field.setStyle(fillColor, strokeColor, strokeWidth)

// Send the field data to the dashboard
field.update()
\`\`\`

### Color Constants
\`\`\`kotlin
PanelsField.RED
PanelsField.BLUE
// And other CSS-compatible color constants
\`\`\`

### Registering and Drawing Images
\`\`\`kotlin
// Register an image (do this once, e.g., at class level)
val imgID = field.registerImage(PanelsField.images.DECODE.DARK)

// Draw the image at the cursor position
field.moveCursor(40.0, 40.0)
field.img(10.0, 10.0, imgID)
\`\`\`

### Setting the Field Background
\`\`\`kotlin
field.setBackground(PanelsField.images.DECODE.LIGHT)
\`\`\`

### Complete Example — Drawing Robot on Field
\`\`\`kotlin
import com.bylazar.configurables.annotations.Configurable
import com.bylazar.field.PanelsField
import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp

@Configurable
@TeleOp(name = "Field Drawing Demo")
class FieldDemo : OpMode() {
    companion object {
        @JvmField var config = Config.PEDRO
        enum class Config { PANELS, PEDRO, RR }
    }

    val telemetry = PanelsTelemetry.telemetry
    val field = PanelsField.field

    override fun init() {
        // Set coordinate system to match your pathing library
        field.setOffsets(when (config) {
            Config.PANELS -> PanelsField.presets.PANELS
            Config.PEDRO -> PanelsField.presets.PEDRO_PATHING
            Config.RR -> PanelsField.presets.ROAD_RUNNER
        })

        field.setBackground(PanelsField.images.DECODE.LIGHT)
    }

    override fun loop() {
        // Draw robot position
        field.moveCursor(robotX, robotY)
        field.setStyle(PanelsField.RED, PanelsField.BLUE, 0.1)
        field.circle(6.5)  // Robot radius in inches

        // Draw a target position
        field.moveCursor(targetX, targetY)
        field.rect(10.0, 5.0)

        // Draw a path line
        field.moveCursor(robotX, robotY)
        field.line(targetX, targetY)

        field.update()

        telemetry.debug("Robot: (\$robotX, \$robotY)")
        telemetry.update()
    }
}
\`\`\`

### Migration from FTC Dashboard Canvas
If migrating from FTC Dashboard's Canvas/TelemetryPacket field overlay:
\`\`\`
// BEFORE (FTC Dashboard):
TelemetryPacket packet = new TelemetryPacket();
Canvas canvas = packet.fieldOverlay();
canvas.setStroke("blue").strokeCircle(robotX, robotY, 9);
dashboard.sendTelemetryPacket(packet);

// AFTER (Panels):
val field = PanelsField.field
field.moveCursor(robotX, robotY)
field.setStyle(PanelsField.BLUE, PanelsField.BLUE, 0.1)
field.circle(9.0)
field.update()
\`\`\`
`,

  limelight: `
## Panels Limelight Integration

### Wireless Limelight Control
Panels includes a built-in LimelightProxy plugin that gives you full access to the Limelight 3A dashboard without a USB connection. This is a major advantage over FTC Dashboard which has no native Limelight support.

### Features
- Wirelessly stream Limelight video feed
- Track camera stats and telemetry
- Configure pipelines remotely
- Integrated with the Panels dashboard UI

### Configuration
\`\`\`kotlin
import com.bylazar.limelightproxy.LimelightProxyConfig

class LimelightProxyConfig : LimelightProxyConfig() {
    @Transient
    override var isDev = false
}
\`\`\`

### Usage
The Limelight proxy is a plugin that runs alongside Panels. Once configured, access the Limelight dashboard directly from the Panels web interface. The built-in widgets provide:
- Video streaming
- Camera stats tracking
- Telemetry logging
- Pipeline switching

This keeps your vision system seamlessly integrated with the rest of your dashboard without needing a separate USB connection or browser tab.
`,

  plugins: `
## Panels Plugin Development

### Architecture
Panels is entirely plugin-driven. Each plugin has:
- A **Kotlin backend** that runs on the Control Hub
- A **Svelte frontend** that renders in the browser dashboard
- Access to the full Panels UI, JS utilities, and FTC SDK

### Built-in Plugins
Each plugin is a separate Gradle module:
| Plugin | Package | Description |
|--------|---------|-------------|
| Panels (Core) | \`com.bylazar.panels\` | Core server, plugin loader, web UI |
| OpModeControl | \`com.bylazar.opmodecontrol\` | Driver Station-style OpMode management |
| Telemetry | \`com.bylazar.telemetry\` | Real-time telemetry |
| Configurables | \`com.bylazar.configurables\` | Live variable tuning |
| Field | \`com.bylazar.field\` | Field canvas drawing |
| Graph | \`com.bylazar.graph\` | Data visualization over time |
| Capture | \`com.bylazar.capture\` | Record/replay debug data |
| Gamepad | \`com.bylazar.gamepad\` | Wireless gamepad support |
| LimelightProxy | \`com.bylazar.limelightproxy\` | Limelight 3A integration |
| CameraStream | \`com.bylazar.camerastream\` | FTC webcam streaming |
| Battery | \`com.bylazar.battery\` | Battery level monitoring |
| Pinger | \`com.bylazar.pinger\` | Input delay monitoring |
| Lights | \`com.bylazar.lights\` | RGB indicator feedback |
| Themes | \`com.bylazar.themes\` | UI customization |
| Docs | \`com.bylazar.docs\` | Integrated documentation |
| Utils | \`com.bylazar.utils\` | Shared utilities |

### Creating Custom Plugins
1. Create a new Gradle module in your project
2. Build a Svelte frontend for the browser UI
3. Build a Kotlin backend that hooks into the FTC SDK
4. Register your plugin with Panels
5. Use the Panels JS utilities, components, and API

### Auto-Update
Each plugin can check for updates when online, keeping your dashboard current.
`,

  gamepads: `
## Panels Gamepad Support

### Overview
Panels supports up to two FTC gamepads wirelessly, giving Driver 1 and Driver 2 full control of the robot from the dashboard. Both controllers are mapped and ready to use out of the box.

### How It Works
- Connect gamepads to the computer running the Panels dashboard in the browser
- Panels forwards gamepad input to the robot as \`gamepad1\` and \`gamepad2\`
- Both controllers are mapped using standard FTC gamepad bindings
- Works alongside (or as a replacement for) the physical Driver Station

### Use Cases
- Testing robot code without a Driver Station
- Remote development and debugging
- Practice driving from a laptop
- Demonstrating robot behavior

### Comparison with FTC Dashboard Gamepad
| Feature | FTC Dashboard | Panels |
|---------|--------------|--------|
| Gamepad count | 1 | 2 (Driver 1 + Driver 2) |
| Setup | Automatic via browser | Automatic via browser |
| Mapping | Standard FTC | Standard FTC |
`,
};
