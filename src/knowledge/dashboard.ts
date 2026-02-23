export const DASHBOARD_KNOWLEDGE = {
  configPattern: `
## @Config Annotation Pattern (FTC Dashboard)

### Import
\`\`\`java
import com.acmerobotics.dashboard.config.Config;
\`\`\`

### Requirements
- The class MUST be annotated with \`@Config\`
- Fields MUST be \`public static\`
- Fields must NOT be \`final\`

### Basic Usage
\`\`\`java
@Config
public class RobotConstants {
    public static double DRIVE_SPEED = 0.8;
    public static int ENCODER_TICKS = 1120;
    public static boolean USE_GYRO = true;
}
\`\`\`

### Custom Dashboard Group Name
\`\`\`java
@Config("Arm Settings")
public class ArmConstants {
    public static double ARM_POWER = 0.5;
    public static int ARM_TARGET = 300;
}
\`\`\`
By default the group name is the class simple name. \`@Config("CustomName")\` overrides this.

### Disabling a Config Class
Add \`@Disabled\` to prevent it from appearing on the dashboard:
\`\`\`java
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Config
@Disabled
public class DeprecatedConstants {
    public static double OLD_VALUE = 1.0;
}
\`\`\`

### Supported Types
- Primitives: \`boolean\`, \`int\`, \`long\`, \`float\`, \`double\`
- \`String\`
- \`enum\` types
- Custom objects (recursively expands all \`public\` non-\`static\` non-\`final\` instance fields)
- Arrays of supported types

### Custom Object Example
\`\`\`java
@Config
public class PIDConstants {
    public static class PIDCoefficients {
        public double kP;
        public double kI;
        public double kD;

        public PIDCoefficients(double kP, double kI, double kD) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
        }
    }

    public static PIDCoefficients DRIVE_PID = new PIDCoefficients(0.05, 0.0, 0.01);
    public static PIDCoefficients ARM_PID = new PIDCoefficients(0.1, 0.0, 0.02);
}
\`\`\`
On the dashboard, \`DRIVE_PID\` expands into editable fields for \`kP\`, \`kI\`, and \`kD\`.

### Kotlin Usage
In Kotlin, fields must use \`@JvmField var\` because the dashboard reads/writes fields directly via reflection — Kotlin properties with getters/setters are not supported.
\`\`\`kotlin
@Config
object RobotConstants {
    @JvmField var DRIVE_SPEED = 0.8
    @JvmField var USE_GYRO = true
    @JvmField var ENCODER_TICKS = 1120
}
\`\`\`

### Thread Safety
For \`long\` and \`double\` fields that are read in a loop thread and written from the dashboard, mark them \`volatile\` to guarantee visibility across threads:
\`\`\`java
@Config
public class TuningConstants {
    public static volatile double HEADING_KP = 1.0;
    public static volatile long DELAY_MS = 250;
    public static int TICKS = 100; // int/boolean are already atomic, volatile optional
}
\`\`\`

### CRITICAL: Copy Semantics Pitfall
Dashboard modifies the static field at runtime. If you copy the value into a local variable or constructor parameter, you will never see updates.

**WRONG** — caching the value into a constructor parameter:
\`\`\`java
@Config
public class DriveConstants {
    public static double MAX_SPEED = 0.8;
}

public class MecanumDrive {
    private final double maxSpeed;

    public MecanumDrive(HardwareMap hardwareMap) {
        // BUG: maxSpeed is copied once; dashboard changes to MAX_SPEED are never seen
        this.maxSpeed = DriveConstants.MAX_SPEED;
    }

    public void drive(double forward, double strafe, double turn) {
        // This always uses the stale value captured in the constructor
        double speed = forward * maxSpeed;
        // ...
    }
}
\`\`\`

**RIGHT** — reading the static field at point of use:
\`\`\`java
public class MecanumDrive {
    public MecanumDrive(HardwareMap hardwareMap) {
        // No cached copy
    }

    public void drive(double forward, double strafe, double turn) {
        // Reads the live static field every loop iteration — dashboard changes take effect immediately
        double speed = forward * DriveConstants.MAX_SPEED;
        // ...
    }
}
\`\`\`

### ValueProvider<T> — Programmatic Config
For variables that cannot be simple static fields, use the ValueProvider interface:
\`\`\`java
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.ValueProvider;

FtcDashboard dashboard = FtcDashboard.getInstance();

dashboard.addConfigVariable("Motors", "leftPower", new ValueProvider<Double>() {
    @Override
    public Double get() {
        return leftMotor.getPower();
    }

    @Override
    public void set(Double value) {
        leftMotor.setPower(value);
    }
});

// Remove when no longer needed
dashboard.removeConfigVariable("Motors", "leftPower");
\`\`\`
`,

  telemetry: `
## FTC Dashboard Telemetry

### Getting the Dashboard Instance
\`\`\`java
import com.acmerobotics.dashboard.FtcDashboard;

FtcDashboard dashboard = FtcDashboard.getInstance();
\`\`\`
\`getInstance()\` returns the singleton. It is available during \`init()\`, \`start()\`, and \`loop()\` (or their LinearOpMode equivalents).

### Approach 1: MultipleTelemetry (Recommended Standard Pattern)
Sends telemetry to BOTH the Driver Station AND the dashboard simultaneously. This is THE standard pattern used by most FTC teams.
\`\`\`java
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

@TeleOp(name = "TeleOp Example")
public class MyTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() {
        // Wrap both telemetry objects — all calls go to DS + dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Loop Time", "%.1f ms", getRuntime() * 1000);
            telemetry.addData("Heading", "%.2f deg", getHeading());
            telemetry.update(); // Sends to both DS and dashboard
        }
    }
}
\`\`\`

### dashboard.getTelemetry()
\`\`\`java
Telemetry dashTelemetry = dashboard.getTelemetry();
dashTelemetry.addData("key", value);
dashTelemetry.update(); // Sends only to dashboard
\`\`\`
Returns a standard \`Telemetry\` adapter that speaks the dashboard protocol. Use this standalone if you only want dashboard output, or wrap it with \`MultipleTelemetry\`.

### Approach 2: TelemetryPacket (Full Control)
For field overlay drawing and advanced telemetry, use packets directly.
\`\`\`java
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

@TeleOp(name = "Packet Example")
public class PacketExample extends LinearOpMode {
    @Override
    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();

        waitForStart();

        while (opModeIsActive()) {
            TelemetryPacket packet = new TelemetryPacket();

            // Key-value data (appears in telemetry panel)
            packet.put("x", robotX);
            packet.put("y", robotY);
            packet.put("heading", Math.toDegrees(robotHeading));

            // Free-form log lines
            packet.addLine("Status: Running");
            packet.addLine("Loop #" + loopCount);

            // Field overlay drawing (see Canvas section)
            packet.fieldOverlay()
                .setFill("blue")
                .fillCircle(robotX, robotY, 9);

            dashboard.sendTelemetryPacket(packet);
        }
    }
}
\`\`\`

### TelemetryPacket API
| Method | Description |
|--------|-------------|
| \`put(String key, Object value)\` | Add a key-value pair to the telemetry data |
| \`addLine(String text)\` | Add a free-form log line |
| \`fieldOverlay()\` | Returns the \`Canvas\` for drawing on the field overlay |
| \`new TelemetryPacket(false)\` | Create a packet that suppresses the default field image background |

### Sending and Timing
\`\`\`java
dashboard.sendTelemetryPacket(packet);              // Send one packet
dashboard.setTelemetryTransmissionInterval(50);     // Set interval in ms (default: 100ms)
dashboard.clearTelemetry();                          // Clear all telemetry on the dashboard
\`\`\`
- Default transmission interval: **100 ms**
- Buffer capacity: **100 packets** — if you produce packets faster than they are consumed, the oldest are dropped.

### GOTCHA: Double-Packet Problem
If you use MultipleTelemetry AND manually call \`sendTelemetryPacket()\`, each loop iteration sends TWO packets — one from \`telemetry.update()\` (via MultipleTelemetry) and one from \`sendTelemetryPacket()\`. This wastes bandwidth and can cause confusing graph behavior.

**Pick ONE approach:**
- **MultipleTelemetry** for simple key-value telemetry to both DS and dashboard.
- **TelemetryPacket + sendTelemetryPacket()** when you need field overlay drawing or full packet control. If you also need DS telemetry, call \`telemetry.addData()\`/\`telemetry.update()\` on the original (non-wrapped) telemetry separately.

### Complete Working Example — Both Approaches Combined Correctly
\`\`\`java
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Dashboard Demo")
public class DashboardDemo extends LinearOpMode {
    @Override
    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();

        double robotX = 0, robotY = 0, robotHeading = 0;

        waitForStart();

        while (opModeIsActive()) {
            // Update robot state (placeholder logic)
            robotX += gamepad1.left_stick_x * 0.5;
            robotY -= gamepad1.left_stick_y * 0.5;
            robotHeading += gamepad1.right_stick_x * 0.05;

            // --- Dashboard telemetry via TelemetryPacket (with field overlay) ---
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("x", robotX);
            packet.put("y", robotY);
            packet.put("heading (deg)", Math.toDegrees(robotHeading));

            packet.fieldOverlay()
                .setStroke("blue")
                .strokeCircle(robotX, robotY, 9)
                .setStroke("green")
                .strokeLine(
                    robotX, robotY,
                    robotX + 12 * Math.cos(robotHeading),
                    robotY + 12 * Math.sin(robotHeading)
                );

            dashboard.sendTelemetryPacket(packet);

            // --- Driver Station telemetry (separate, no MultipleTelemetry wrapper) ---
            telemetry.addData("X", "%.1f", robotX);
            telemetry.addData("Y", "%.1f", robotY);
            telemetry.addData("Heading", "%.1f deg", Math.toDegrees(robotHeading));
            telemetry.update();
        }
    }
}
\`\`\`
`,

  canvas: `
## Canvas API — FTC Dashboard Field Overlay

The Canvas is accessed via \`packet.fieldOverlay()\` on a \`TelemetryPacket\`. All Canvas methods are **fluent/chainable** (they return \`this\`).

### Coordinate System
- Units: **inches**
- Origin: **center of the field**
- **+X**: toward the right when facing the red alliance wall
- **+Y**: away from the red alliance wall (toward the blue alliance wall)
- The default field image is 144 x 144 inches (12 ft x 12 ft)

### Shape Methods (Exact Signatures)

| Method | Description |
|--------|-------------|
| \`fillCircle(double x, double y, double r)\` | Filled circle at (x, y) with radius r |
| \`strokeCircle(double x, double y, double r)\` | Outlined circle at (x, y) with radius r |
| \`fillRect(double x, double y, double w, double h)\` | Filled rectangle — (x, y) is the CENTER, w and h are full width/height |
| \`strokeRect(double x, double y, double w, double h)\` | Outlined rectangle — (x, y) is the CENTER |
| \`fillPolygon(double[] xPoints, double[] yPoints)\` | Filled polygon from coordinate arrays |
| \`strokePolygon(double[] xPoints, double[] yPoints)\` | Outlined polygon from coordinate arrays |
| \`strokePolyline(double[] xPoints, double[] yPoints)\` | Open polyline (not closed) from coordinate arrays |
| \`strokeLine(double x1, double y1, double x2, double y2)\` | Line segment from (x1,y1) to (x2,y2) |

### Text Methods

| Method | Description |
|--------|-------------|
| \`fillText(String text, double x, double y, String font, double theta)\` | Filled text at (x,y), rotated by theta radians |
| \`strokeText(String text, double x, double y, String font, double theta)\` | Outlined text at (x,y), rotated by theta radians |
| \`fillText(String text, double x, double y, String font, double theta, boolean usePageFrame)\` | With explicit page frame flag |
| \`strokeText(String text, double x, double y, String font, double theta, boolean usePageFrame)\` | With explicit page frame flag |

- \`font\`: CSS font string, e.g. \`"16px monospace"\`, \`"bold 12px sans-serif"\`
- \`theta\`: rotation in radians
- \`usePageFrame\`: when \`true\`, coordinates are in page/pixel frame (ignores field transforms); when \`false\` (default), coordinates are in the field/transform frame

### Image Drawing
\`\`\`java
canvas.drawImage(String path, double x, double y, double w, double h)
canvas.drawImage(String path, double x, double y, double w, double h, boolean usePageFrame)
\`\`\`
- Custom images go in: \`TeamCode/src/main/assets/images/\`
- \`path\` is relative to the assets directory, e.g. \`"/images/myRobot.png"\`
- (x, y) is the center; w and h are full dimensions in inches

### Style Methods

| Method | Description |
|--------|-------------|
| \`setFill(String cssColor)\` | Set fill color — any CSS color: \`"red"\`, \`"#FF0000"\`, \`"rgba(255,0,0,0.5)"\` |
| \`setStroke(String cssColor)\` | Set stroke/outline color |
| \`setStrokeWidth(int px)\` | Set stroke width in pixels |
| \`setAlpha(double alpha)\` | Set global transparency: 0.0 (invisible) to 1.0 (opaque) |

### Transform Methods
Each transform method **OVERRIDES** the previous transform of that type (they do NOT compose/accumulate).

| Method | Description |
|--------|-------------|
| \`setTranslation(double x, double y)\` | Translate subsequent drawings by (x, y) |
| \`setRotation(double theta)\` | Rotate subsequent drawings by theta radians |
| \`setScale(double sx, double sy)\` | Scale subsequent drawings |

To combine transforms, apply them before drawing. They affect all subsequent draw calls until changed.

### Common Pattern: Drawing a Robot on the Field
\`\`\`java
TelemetryPacket packet = new TelemetryPacket();
Canvas canvas = packet.fieldOverlay();

double robotX = 24.0;   // inches from center
double robotY = -36.0;  // inches from center
double heading = Math.toRadians(45); // radians
double robotRadius = 9.0; // inches (typical 18" robot)

// Draw robot body
canvas.setStroke("blue")
      .setStrokeWidth(2)
      .strokeCircle(robotX, robotY, robotRadius);

// Draw heading indicator line
canvas.setStroke("green")
      .strokeLine(
          robotX, robotY,
          robotX + robotRadius * Math.cos(heading),
          robotY + robotRadius * Math.sin(heading)
      );

dashboard.sendTelemetryPacket(packet);
\`\`\`

### Common Pattern: Drawing a Path/Trajectory
\`\`\`java
TelemetryPacket packet = new TelemetryPacket();
Canvas canvas = packet.fieldOverlay();

// Define waypoints
double[] xPoints = {0, 12, 24, 36, 48};
double[] yPoints = {0, 12, 12, 0,  -12};

// Draw the path as a polyline
canvas.setStroke("red")
      .setStrokeWidth(1)
      .strokePolyline(xPoints, yPoints);

// Highlight waypoints
canvas.setFill("yellow")
      .setStroke("black")
      .setStrokeWidth(1);
for (int i = 0; i < xPoints.length; i++) {
    canvas.fillCircle(xPoints[i], yPoints[i], 2);
    canvas.strokeCircle(xPoints[i], yPoints[i], 2);
}

dashboard.sendTelemetryPacket(packet);
\`\`\`

### Complete Example with Transforms
\`\`\`java
TelemetryPacket packet = new TelemetryPacket();
Canvas canvas = packet.fieldOverlay();

// Draw a robot-shaped rectangle rotated to the robot's heading
double robotX = 10, robotY = 20;
double heading = Math.toRadians(30);

canvas.setTranslation(robotX, robotY)
      .setRotation(heading)
      .setStroke("blue")
      .setStrokeWidth(2)
      .strokeRect(0, 0, 18, 18)  // 18x18 inch robot centered at transform origin
      .setStroke("red")
      .strokeLine(0, 0, 12, 0);  // heading arrow

// Reset transforms for other drawings
canvas.setTranslation(0, 0)
      .setRotation(0);

// Draw field elements in absolute coordinates
canvas.setFill("yellow")
      .fillCircle(0, 0, 3); // mark field center

dashboard.sendTelemetryPacket(packet);
\`\`\`
`,

  camera: `
## Camera Streaming to FTC Dashboard

### Method 1: VisionPortal Processor (Recommended for modern FTC SDK)

Create a class that implements both \`VisionProcessor\` (for VisionPortal) and \`CameraStreamSource\` (for dashboard):

\`\`\`java
import android.graphics.Bitmap;
import android.graphics.Canvas;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Mat;

import java.util.concurrent.atomic.AtomicReference;

public class CameraStreamProcessor implements VisionProcessor, CameraStreamSource {
    private final AtomicReference<Bitmap> lastFrame = new AtomicReference<>(
        Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565)
    );

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        // Called when the processor is initialized
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        // Convert the OpenCV Mat to an Android Bitmap
        Bitmap bitmap = Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.RGB_565);
        Utils.matToBitmap(frame, bitmap);
        lastFrame.set(bitmap);
        return null; // No user context needed
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                            float scaleBmpPxToCanvasPx, float scaleCanvasDensity,
                            Object userContext) {
        // Optional: draw annotations on the DS preview
    }

    @Override
    public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
        continuation.dispatch(new Consumer<Bitmap>() {
            @Override
            public void accept(Bitmap bitmapConsumer) {
                bitmapConsumer = lastFrame.get();
            }
        });
    }
}
\`\`\`

**Using it in your OpMode:**
\`\`\`java
import com.acmerobotics.dashboard.FtcDashboard;
import org.firstinspires.ftc.vision.VisionPortal;

@TeleOp(name = "Camera Stream Demo")
public class CameraStreamDemo extends LinearOpMode {
    @Override
    public void runOpMode() {
        CameraStreamProcessor cameraProcessor = new CameraStreamProcessor();

        VisionPortal portal = new VisionPortal.Builder()
            .addProcessor(cameraProcessor)
            .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
            .build();

        FtcDashboard.getInstance().startCameraStream(cameraProcessor, 30);

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addLine("Streaming to dashboard...");
            telemetry.update();
        }

        // Camera stream auto-stops when OpMode ends
        portal.close();
    }
}
\`\`\`

### Method 2: EasyOpenCV (OpenCvWebcam)
\`\`\`java
import com.acmerobotics.dashboard.FtcDashboard;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@TeleOp(name = "EasyOpenCV Stream")
public class EasyOpenCVStream extends LinearOpMode {
    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources()
            .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        OpenCvWebcam webcam = OpenCvCameraFactory.getInstance()
            .createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        webcam.setPipeline(new MyPipeline());

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Error", errorCode);
                telemetry.update();
            }
        });

        // OpenCvWebcam implements CameraStreamSource
        FtcDashboard.getInstance().startCameraStream(webcam, 0); // 0 = unlimited FPS

        waitForStart();

        while (opModeIsActive()) {
            sleep(20);
        }

        webcam.stopStreaming();
    }
}
\`\`\`

### Method 3: Limelight 3A
\`\`\`java
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.limelightvision.Limelight3A;

@TeleOp(name = "Limelight Stream")
public class LimelightStream extends LinearOpMode {
    @Override
    public void runOpMode() {
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();

        // Limelight3A implements CameraStreamSource
        FtcDashboard.getInstance().startCameraStream(limelight, 0);

        waitForStart();

        while (opModeIsActive()) {
            sleep(20);
        }

        limelight.stop();
    }
}
\`\`\`

### Camera Stream API
\`\`\`java
FtcDashboard dashboard = FtcDashboard.getInstance();

// Start streaming from any CameraStreamSource
dashboard.startCameraStream(source, maxFps);  // maxFps: 0 = unlimited

// Stop the camera stream
dashboard.stopCameraStream();

// Send a single Bitmap image (useful for processed frames)
dashboard.sendImage(bitmap);

// Set JPEG compression quality (0-100, default 50)
dashboard.setImageQuality(75);
\`\`\`

### Important Notes
- The camera stream **auto-stops** when the OpMode ends.
- \`maxFps\` of \`0\` means unlimited — frames are sent as fast as they are produced.
- Lower \`setImageQuality()\` values reduce bandwidth but degrade image quality.
- Only ONE camera stream can be active at a time. Starting a new stream replaces the previous one.
`,

  setup: `
## FTC Dashboard Installation & Setup

### Gradle Configuration

**Step 1:** Add the Maven repository to \`build.dependencies.gradle\` (or your project-level \`build.gradle\`):
\`\`\`groovy
repositories {
    maven { url = 'https://maven.brott.dev/' }
}
\`\`\`

**Step 2:** Add the dependency (in your TeamCode \`build.gradle\`):
\`\`\`groovy
dependencies {
    implementation 'com.acmerobotics.dashboard:dashboard:0.5.1'
}
\`\`\`

### Accessing the Dashboard

| Connection Method | URL |
|-------------------|-----|
| **Control Hub** (Wi-Fi Direct) | \`http://192.168.43.1:8080/dash\` |
| **Phone** (Wi-Fi Direct) | \`http://192.168.49.1:8080/dash\` |

Open the URL in any modern web browser (Chrome recommended) on a device connected to the Robot Controller's Wi-Fi network.

### Dashboard Features
- **Browser OpMode Controls**: Start, stop, and init OpModes directly from the dashboard web interface without needing a Driver Station.
- **Gamepad Support**: Connect a gamepad to the computer running the dashboard browser; the dashboard forwards gamepad input to the robot as gamepad1/gamepad2.
- **Live Telemetry Graphs**: Numeric telemetry values are automatically graphed over time.
- **Configuration Panel**: Live-edit \`@Config\` variables without redeploying code.
- **Camera Stream**: View camera feed in the browser.
- **Field Overlay**: Draw on a top-down field diagram in real time.

### Disabling for Competition (RS09)
FTC rule RS09 prohibits wireless communication to non-approved devices during competition. The dashboard must be disabled before competing:

\`\`\`java
// In your OpMode or robot initialization
FtcDashboard dashboard = FtcDashboard.getInstance();
// Dashboard is enabled by default; no special action needed for practice
// For competition, disable the dashboard to comply with RS09
\`\`\`

The simplest approach: do NOT connect to the dashboard network during matches. The dashboard only activates when a browser client connects. Alternatively, you can avoid including the dashboard dependency in your competition build, or call \`FtcDashboard.getInstance().setEnabled(false)\` at init.

### Complete Minimal OpMode with Dashboard
\`\`\`java
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp(name = "Dashboard Quickstart")
public class DashboardQuickstart extends LinearOpMode {
    public static double MOTOR_POWER = 0.5;
    public static int TARGET_POSITION = 1000;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Motor Power", MOTOR_POWER);
            telemetry.addData("Target", TARGET_POSITION);
            telemetry.addData("Time", "%.1f s", getRuntime());
            telemetry.update();
        }
    }
}
\`\`\`
`,

  api: `
## FtcDashboard Public API Summary

\`\`\`java
import com.acmerobotics.dashboard.FtcDashboard;

FtcDashboard dashboard = FtcDashboard.getInstance();
\`\`\`

### Singleton Access

| Method | Description |
|--------|-------------|
| \`static FtcDashboard getInstance()\` | Returns the singleton dashboard instance. Available after SDK initialization, during init/start/loop. |

### Telemetry

| Method | Description |
|--------|-------------|
| \`void sendTelemetryPacket(TelemetryPacket packet)\` | Send a telemetry packet with key-value data and optional field overlay |
| \`void clearTelemetry()\` | Clear all telemetry data displayed on the dashboard |
| \`Telemetry getTelemetry()\` | Get a standard Telemetry adapter that forwards to the dashboard |
| \`void setTelemetryTransmissionInterval(int ms)\` | Set how often telemetry is sent to the browser (default: 100ms) |

### Camera Streaming

| Method | Description |
|--------|-------------|
| \`void startCameraStream(CameraStreamSource source, double maxFps)\` | Start streaming from a CameraStreamSource; maxFps 0 = unlimited |
| \`void stopCameraStream()\` | Stop the active camera stream |
| \`void sendImage(Bitmap bitmap)\` | Send a single Bitmap image to the dashboard |
| \`void setImageQuality(int quality)\` | Set JPEG quality 0-100 (default: 50) |

### Configuration Variables

| Method | Description |
|--------|-------------|
| \`void addConfigVariable(String group, String name, ValueProvider<?> provider)\` | Register a programmatic config variable under the given group |
| \`void removeConfigVariable(String group, String name)\` | Remove a previously registered config variable |
| \`void updateConfig()\` | Force a config update — pushes current static field values to connected clients |

### OpMode & State Control

| Method | Description |
|--------|-------------|
| \`boolean isEnabled()\` | Returns whether the dashboard is currently enabled |
| \`void suppressOpMode(boolean suppress)\` | When true, suppresses the current OpMode from appearing in the dashboard OpMode list |

### Typical Usage Pattern
\`\`\`java
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.config.ValueProvider;

@Config
@TeleOp(name = "Full API Demo")
public class FullApiDemo extends LinearOpMode {
    public static double GAIN = 1.0;
    public static int THRESHOLD = 100;

    @Override
    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();

        // Option A: MultipleTelemetry for simple key-value data
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        // Register a dynamic config variable
        dashboard.addConfigVariable("Sensors", "brightness", new ValueProvider<Integer>() {
            @Override
            public Integer get() {
                return 50; // read from sensor
            }

            @Override
            public void set(Integer value) {
                // apply brightness
            }
        });

        waitForStart();

        while (opModeIsActive()) {
            // Option B: Full packet with field overlay
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("gain", GAIN);
            packet.put("threshold", THRESHOLD);

            packet.fieldOverlay()
                .setFill("green")
                .fillCircle(0, 0, 9);

            dashboard.sendTelemetryPacket(packet);

            // Driver Station telemetry (using original, not MultipleTelemetry to avoid double-send)
            // If you used MultipleTelemetry above AND sendTelemetryPacket, comment out one approach.
            telemetry.addData("Gain", GAIN);
            telemetry.update();
        }

        // Cleanup
        dashboard.removeConfigVariable("Sensors", "brightness");
    }
}
\`\`\`
`,
};
