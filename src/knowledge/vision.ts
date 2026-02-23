export const VISION_KNOWLEDGE = {
  overview: `
## FTC Vision Systems Overview

FTC teams have two primary vision options: **USB webcams** processed via the SDK's VisionPortal,
and the **Limelight 3A** smart camera. Both can detect AprilTags and game elements; they differ
in processing location, setup complexity, and capability.

### Option 1: USB Webcam + VisionPortal (SDK Built-In)

The FTC SDK (v9.0+) includes VisionPortal — a built-in vision framework that runs on the
Control Hub's CPU. It processes frames from any UVC-compatible USB webcam.

**Capabilities:**
- AprilTag detection and 6DOF pose estimation (built-in AprilTagProcessor)
- Custom color/object detection via VisionProcessor interface + OpenCV
- Multiple processors running simultaneously on the same camera
- Camera controls: exposure, gain, focus, white balance
- Dual camera support (MultiPortal)
- Dashboard camera streaming

**When to use:**
- Budget-constrained teams (webcams cost $10-30)
- Need custom OpenCV pipelines (color detection, contour analysis)
- Running multiple detection processors simultaneously
- Need full control over the processing pipeline

**Recommended webcams:**
- Logitech C270 — cheapest, fixed focus, 640x480 max, good for AprilTags
- Logitech C920/C922 — adjustable focus, up to 1920x1080, better image quality
- Logitech C615 — compact, adjustable focus, good all-rounder
- Any UVC-compatible USB webcam works

### Option 2: Limelight 3A (Smart Camera)

The Limelight 3A is a dedicated vision coprocessor with an integrated camera. It runs its own
processing pipeline independently from the Control Hub CPU. The FTC SDK treats it as a hardwareMap device.

**Capabilities:**
- AprilTag detection with MegaTag1/MegaTag2 robot localization
- Built-in color blob tracking (90 FPS at 640x480)
- Neural network object detection and classification
- Barcode/QR code reading
- Custom Python (SnapScript) pipelines with OpenCV + numpy
- 10 hot-swappable pipelines configurable via web UI
- Built-in FTC field map for the current season
- Zero Control Hub CPU load for vision processing

**When to use:**
- Want fast, reliable AprilTag localization (MegaTag2 is best-in-class)
- Need high FPS color tracking without impacting loop time
- Don't want to write custom OpenCV code
- Budget allows ($275 for the device)

**Specs:**
- Sensor: OV5647 (640x480 @ 90 FPS)
- FOV: 54.5° horizontal, 42° vertical
- Connection: USB-C to Control Hub USB 3.0 port
- Power: 4.1V-5.75V via USB, max 4W
- Boot time: ~15-20 seconds
- No LED illumination, no Google Coral neural accelerator

### Can You Use Both?
Yes. A USB webcam with VisionPortal and a Limelight 3A can run simultaneously.
They use separate USB ports and separate processing (VisionPortal on Control Hub CPU,
Limelight on its own processor). Common pattern: Limelight for AprilTag localization,
webcam for custom color detection.
`,

  visionPortalSetup: `
## VisionPortal Setup — Complete Guide

VisionPortal is the SDK's built-in vision framework. It manages the camera lifecycle, routes
frames to processors, and handles the preview display.

### Imports
\`\`\`java
import android.util.Size;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.vision.VisionPortal;
\`\`\`

### Quick Setup (Defaults)
\`\`\`java
// One-liner with a single processor
VisionPortal portal = VisionPortal.easyCreateWithDefaults(
    hardwareMap.get(WebcamName.class, "Webcam 1"),
    aprilTagProcessor
);

// Multiple processors
VisionPortal portal = VisionPortal.easyCreateWithDefaults(
    hardwareMap.get(WebcamName.class, "Webcam 1"),
    aprilTagProcessor, myColorProcessor
);
\`\`\`

### Builder Pattern (Full Control)
\`\`\`java
VisionPortal portal = new VisionPortal.Builder()
    // Camera source — USB webcam or phone camera
    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
    // OR: .setCamera(BuiltinCameraDirection.BACK)  // phone camera

    // Add one or more processors
    .addProcessor(aprilTagProcessor)
    .addProcessor(myColorProcessor)   // optional second processor

    // Resolution — must match a resolution the webcam supports
    // Common choices: 320x240, 640x480, 800x600, 1280x720, 1920x1080
    .setCameraResolution(new Size(640, 480))

    // Stream format
    // YUY2 (default): higher quality, higher USB bandwidth
    // MJPEG: compressed, saves USB bandwidth (use for dual cameras)
    .setStreamFormat(VisionPortal.StreamFormat.YUY2)

    // LiveView — the camera preview on the Robot Controller screen
    .enableLiveView(true)           // true = show preview (default)
    .setAutoStopLiveView(false)     // if true, goes orange when no processors enabled

    .build();
\`\`\`

### Camera State Machine
\`\`\`
OPENING_CAMERA_DEVICE → CAMERA_DEVICE_READY → STARTING_STREAM → STREAMING
                                                                     ↓
                        CAMERA_DEVICE_CLOSED ← CLOSING ← STOPPING_STREAM
\`\`\`

Camera controls (exposure, gain) require the camera to be in STREAMING state.
Always wait for streaming before setting controls:
\`\`\`java
// Wait for camera to start streaming
while (!isStopRequested() &&
       portal.getCameraState() != VisionPortal.CameraState.STREAMING) {
    sleep(20);
}
// NOW safe to set camera controls
\`\`\`

### Lifecycle Management — 4 Levels of CPU Control
\`\`\`java
// Level 1: Toggle LiveView only (fastest resume, minimal CPU savings)
portal.stopLiveView();
portal.resumeLiveView();

// Level 2: Toggle processors (fast resume, good CPU savings)
portal.setProcessorEnabled(aprilTagProcessor, false);  // disable
portal.setProcessorEnabled(aprilTagProcessor, true);   // re-enable

// Level 3: Stop/resume streaming (~1 second resume time)
portal.stopStreaming();     // stops all background processing
portal.resumeStreaming();   // restarts camera stream

// Level 4: Close portal (permanent — cannot reopen)
portal.close();  // call only when completely done with vision
\`\`\`

**Best practice:** Use Level 2 (processor toggling) during a match. Disable AprilTag
during TeleOp driving, enable it when you need alignment. Avoid Level 3 unless you
have a long period without vision (the ~1s resume time can cost you in auto).

### Monitoring
\`\`\`java
// Frame rate
float fps = portal.getFps();

// Camera state
VisionPortal.CameraState state = portal.getCameraState();

// Processor status
boolean enabled = portal.getProcessorEnabled(aprilTagProcessor);
\`\`\`

### Stream Format Guidance
| Format | Bandwidth | Quality | Best For |
|---|---|---|---|
| YUY2 (default) | High | Best | Single camera, plenty of USB bandwidth |
| MJPEG | Low | Good (compressed) | Dual cameras, shared USB hub, Dashboard streaming |

**Note:** Very low resolutions (<432x240) with MJPEG may have poor image quality.
Very high resolutions (>640x480) with YUY2 may exceed USB 2.0 bandwidth on shared hub.

### Discovering Supported Resolutions
If you don't know what resolutions your webcam supports, intentionally request an
invalid resolution (like 1x1). The error message will list all supported resolutions:
\`\`\`java
.setCameraResolution(new Size(1, 1))  // Will fail with list of supported sizes
\`\`\`
`,

  aprilTagDetection: `
## AprilTag Detection — Complete Guide

The FTC SDK includes a built-in AprilTagProcessor that detects AprilTag fiducial markers
and computes 6DOF (six degrees of freedom) pose estimation. The current FTC season uses
the TAG_36h11 family.

### Imports
\`\`\`java
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
\`\`\`

### Quick Setup
\`\`\`java
AprilTagProcessor aprilTag = AprilTagProcessor.easyCreateWithDefaults();
\`\`\`

### Builder Pattern (Full Customization)
\`\`\`java
AprilTagProcessor aprilTag = new AprilTagProcessor.Builder()
    // Tag family — TAG_36h11 for FTC (default)
    .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)

    // Tag library — includes current season field tags + sample tags
    .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())

    // Output units (default: INCH, DEGREES)
    .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

    // Lens intrinsics (fx, fy, cx, cy) — override if SDK doesn't have your webcam's calibration
    // Omit this to use the SDK's built-in calibration for recognized cameras
    // .setLensIntrinsics(578.272, 578.272, 402.145, 221.506)

    // Annotation drawing options (what to draw on the preview)
    .setDrawTagID(true)           // Show tag ID number
    .setDrawTagOutline(true)      // Draw outline around tag
    .setDrawAxes(false)           // Draw XYZ axes (costly, disable in competition)
    .setDrawCubeProjection(false) // Draw 3D cube projection (costly, disable in competition)

    .build();
\`\`\`

### Decimation — THE Key Performance Setting
Decimation downsamples the image before tag detection, trading range for speed.
This is the single most impactful performance setting for AprilTag.

\`\`\`java
// Can be changed at ANY time during runtime (not just at build)
aprilTag.setDecimation(3);  // default is 3
\`\`\`

**Decimation trade-offs (Logitech C920 at 640x480):**
| Decimation | FPS | Detect 2" tag from | Detect 5" tag from |
|---|---|---|---|
| 1 | ~10 | 10 feet | 25+ feet |
| 2 | ~22 | 6 feet | 15 feet |
| 3 (default) | ~30 | 4 feet | 10 feet |

**Strategy:**
- Use decimation=1 during init (static robot, max range, FPS doesn't matter)
- Switch to decimation=3 during driving (need high FPS, closer range OK)
- Switch to decimation=2 for final alignment (balance of range and speed)

### Reading Detections
\`\`\`java
List<AprilTagDetection> detections = aprilTag.getDetections();

for (AprilTagDetection detection : detections) {
    int tagId = detection.id;

    if (detection.metadata != null) {
        // Tag IS in the library — full 6DOF pose available
        String tagName = detection.metadata.name;

        // Camera-relative pose (AprilTagPoseFtc):
        double x     = detection.ftcPose.x;         // lateral offset (inches, right = positive)
        double y     = detection.ftcPose.y;         // forward distance (inches)
        double z     = detection.ftcPose.z;         // vertical offset (inches, up = positive)
        double yaw   = detection.ftcPose.yaw;       // heading rotation (degrees)
        double pitch = detection.ftcPose.pitch;     // pitch rotation (degrees)
        double roll  = detection.ftcPose.roll;      // roll rotation (degrees)

        // Spherical coordinates (often more useful for driving):
        double range     = detection.ftcPose.range;     // direct distance to tag (inches)
        double bearing   = detection.ftcPose.bearing;   // horizontal angle to tag (degrees)
        double elevation = detection.ftcPose.elevation;  // vertical angle to tag (degrees)
    } else {
        // Tag NOT in library — only pixel center available (no pose)
        double centerX = detection.center.x;  // pixels
        double centerY = detection.center.y;  // pixels
    }
}
\`\`\`

### AprilTagPoseFtc Coordinate System
\`\`\`
        +Z (up)
        |
        |
        +--- +X (right)
       /
      /
    +Y (forward, out of camera)
\`\`\`
- **Y axis** = straight out from the camera lens (distance)
- **X axis** = to the right of the camera
- **Z axis** = upward from the camera
- **range** = sqrt(x² + y²) = direct line-of-sight distance
- **bearing** = horizontal angle from camera centerline to tag
- **elevation** = vertical angle from camera centerline to tag

### Custom Tag Libraries
Add your own tags alongside the season's field tags:
\`\`\`java
AprilTagLibrary myLibrary = new AprilTagLibrary.Builder()
    .addTags(AprilTagGameDatabase.getCurrentGameTagLibrary())  // include season tags
    .addTag(55, "Team Marker", 3.5, DistanceUnit.INCH)         // add custom tag
    .build();

AprilTagProcessor aprilTag = new AprilTagProcessor.Builder()
    .setTagLibrary(myLibrary)
    .build();
\`\`\`

**Tag metadata includes:**
- \`id\` — integer ID code
- \`name\` — human-readable name
- \`tagsize\` — size across the inner black border (needed for pose estimation)
- \`fieldPosition\` / \`fieldOrientation\` — optional field-relative position for localization

### Complete AprilTag Detection Example
\`\`\`java
@TeleOp(name = "AprilTag Detection", group = "Vision")
public class AprilTagExample extends LinearOpMode {
    @Override
    public void runOpMode() {
        AprilTagProcessor aprilTag = new AprilTagProcessor.Builder()
            .setDrawAxes(true)
            .setDrawTagOutline(true)
            .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
            .build();

        VisionPortal portal = new VisionPortal.Builder()
            .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
            .addProcessor(aprilTag)
            .setCameraResolution(new Size(640, 480))
            .build();

        // Wait for camera to start streaming
        while (!isStopRequested() &&
               portal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            sleep(20);
        }

        // Set low exposure for reduced motion blur
        ExposureControl exposure = portal.getCameraControl(ExposureControl.class);
        exposure.setMode(ExposureControl.Mode.Manual);
        exposure.setExposure(6, TimeUnit.MILLISECONDS);
        GainControl gain = portal.getCameraControl(GainControl.class);
        gain.setGain(250);

        // High decimation during init for max range
        aprilTag.setDecimation(1);

        waitForStart();

        // Switch to faster decimation during driving
        aprilTag.setDecimation(3);

        while (opModeIsActive()) {
            List<AprilTagDetection> detections = aprilTag.getDetections();
            telemetry.addData("Tags Detected", detections.size());

            for (AprilTagDetection det : detections) {
                if (det.metadata != null) {
                    telemetry.addLine(String.format("\\nTag %d \\"%s\\"", det.id, det.metadata.name));
                    telemetry.addData("  Range",   "%.1f inches", det.ftcPose.range);
                    telemetry.addData("  Bearing", "%.1f degrees", det.ftcPose.bearing);
                    telemetry.addData("  Yaw",     "%.1f degrees", det.ftcPose.yaw);
                }
            }
            telemetry.addData("FPS", "%.1f", portal.getFps());
            telemetry.update();
            sleep(20);
        }
        portal.close();
    }
}
\`\`\`
`,

  cameraControls: `
## Camera Controls — Exposure, Gain, Focus, White Balance

USB webcam controls are critical for reliable vision. Low exposure reduces motion blur
(critical for AprilTags during driving), and manual white balance ensures consistent color detection.

**IMPORTANT:** Camera controls are only available for USB webcams, NOT phone cameras or Limelight.
Controls require the camera to be in STREAMING state — always wait for streaming first.

### Imports
\`\`\`java
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.FocusControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.PtzcControl;
import java.util.concurrent.TimeUnit;
\`\`\`

### Wait for Camera Ready
\`\`\`java
// MUST wait for STREAMING state before accessing any camera control
while (!isStopRequested() &&
       portal.getCameraState() != VisionPortal.CameraState.STREAMING) {
    sleep(20);
}
\`\`\`

### Exposure Control (Most Important for AprilTags)
\`\`\`java
ExposureControl exposure = portal.getCameraControl(ExposureControl.class);

// Switch to manual mode (required before setting exposure)
exposure.setMode(ExposureControl.Mode.Manual);
sleep(50);  // allow mode switch to take effect

// Set exposure time in milliseconds
exposure.setExposure(6, TimeUnit.MILLISECONDS);
sleep(20);

// Get supported range
long minExposure = exposure.getMinExposure(TimeUnit.MILLISECONDS);  // typically 0-1
long maxExposure = exposure.getMaxExposure(TimeUnit.MILLISECONDS);  // typically 200-250

// Check if manual mode is supported
boolean canManual = exposure.isModeSupported(ExposureControl.Mode.Manual);
\`\`\`

**Exposure guidelines:**
- **5-6 ms** — Best for AprilTag detection while driving. Very little motion blur.
  Image will be dark, compensate with gain.
- **10-15 ms** — Good balance for color detection. Some motion blur at speed.
- **30+ ms** — Bright image, significant motion blur. Only use when robot is stationary.
- **Auto mode** — Camera adjusts itself. Inconsistent. Avoid for competition.

### Gain Control (Brightness Compensation)
\`\`\`java
GainControl gain = portal.getCameraControl(GainControl.class);
gain.setGain(250);  // 0 to 255 typically
sleep(20);

// Get supported range
int minGain = gain.getMinGain();  // typically 0
int maxGain = gain.getMaxGain();  // typically 255
\`\`\`

**Gain guidelines:**
- Higher gain = brighter image, more noise
- Lower gain = darker image, less noise
- For low exposure (5-6ms), set gain to 200-250 to compensate for darkness
- High gain + low exposure is better than high exposure for moving robots

### Focus Control
\`\`\`java
FocusControl focus = portal.getCameraControl(FocusControl.class);

// Fixed focus at infinity (best for AprilTags)
focus.setMode(FocusControl.Mode.Fixed);
focus.setFocusLength(0);  // 0 = infinity on most webcams
sleep(20);

// OR auto-focus (inconsistent, not recommended for competition)
focus.setMode(FocusControl.Mode.Auto);

// Get supported range
double minFocus = focus.getMinFocusLength();  // 0
double maxFocus = focus.getMaxFocusLength();  // 250 typically
\`\`\`

**Focus guidelines:**
- **Fixed at 0 (infinity)** — Best for AprilTags and distant objects. Always consistent.
- **Auto-focus** — Can hunt and cause missed detections. Avoid in competition.
- Logitech C270 has fixed focus (no adjustment needed — it's always at infinity).
- Logitech C920/C922 have adjustable focus — MUST set to fixed in code.

### White Balance Control (Critical for Color Detection)
\`\`\`java
WhiteBalanceControl wb = portal.getCameraControl(WhiteBalanceControl.class);

// Manual white balance (consistent color detection)
wb.setMode(WhiteBalanceControl.Mode.MANUAL);
wb.setWhiteBalanceTemperature(4600);  // Kelvin (neutral white)
sleep(20);

// Get supported range
int minTemp = wb.getMinWhiteBalanceTemperature();  // ~2000
int maxTemp = wb.getMaxWhiteBalanceTemperature();  // ~6500
\`\`\`

**White balance guidelines:**
- **Manual ~4000-5000K** — Best for consistent color detection under venue lighting
- **Auto** — Camera adjusts to ambient light. Colors shift between venues. Avoid for competition.
- Tune white balance at the actual competition venue if possible
- Indoor fluorescent: ~4000K. Indoor LED: ~5000K. Outdoor/daylight: ~6500K.

### Recommended Competition Settings
\`\`\`java
// For AprilTag detection during driving (low blur, moderate brightness)
private void setAprilTagExposure(VisionPortal portal) {
    ExposureControl exposure = portal.getCameraControl(ExposureControl.class);
    exposure.setMode(ExposureControl.Mode.Manual);
    sleep(50);
    exposure.setExposure(6, TimeUnit.MILLISECONDS);
    sleep(20);
    GainControl gain = portal.getCameraControl(GainControl.class);
    gain.setGain(250);
    sleep(20);
}

// For color detection (brighter, consistent white balance)
private void setColorDetectionSettings(VisionPortal portal) {
    ExposureControl exposure = portal.getCameraControl(ExposureControl.class);
    exposure.setMode(ExposureControl.Mode.Manual);
    sleep(50);
    exposure.setExposure(15, TimeUnit.MILLISECONDS);
    sleep(20);
    GainControl gain = portal.getCameraControl(GainControl.class);
    gain.setGain(150);
    sleep(20);
    WhiteBalanceControl wb = portal.getCameraControl(WhiteBalanceControl.class);
    wb.setMode(WhiteBalanceControl.Mode.MANUAL);
    wb.setWhiteBalanceTemperature(4600);
    sleep(20);
}
\`\`\`

### Logitech C920 Reference Ranges
| Control | Min | Max | Default |
|---|---|---|---|
| Exposure | 0 ms | 204 ms | Auto |
| Gain | 0 | 255 | Auto |
| White Balance | 2000K | 6500K | Auto |
| Focus | 0 | 250 | 0 (infinity) |
| Zoom | 100 | 500 | 100 |
`,

  limelight: `
## Limelight 3A — Complete FTC Guide

The Limelight 3A is a dedicated vision coprocessor with an integrated camera. It connects via
USB-C and appears as a network device in the FTC hardwareMap. All image processing runs on the
Limelight's processor — zero CPU load on the Control Hub.

### Hardware Setup
1. Mount the Limelight 3A on the robot (4x #10 THRU, 4x M3, or 6x M4 mounting holes)
2. Connect USB-C cable from Limelight to the Control Hub's **USB 3.0 port** (blue port)
3. Power comes through USB (4.1V-5.75V, max 4W)
4. Wait for green status light (~15-20 seconds boot)
5. Connect to the Control Hub's WiFi, then access the web UI at \`http://limelight.local:5801\`

### Hardware Map Configuration
On the Driver Station:
1. Click "Configure Robot"
2. Click "Scan" — an "Ethernet Device" will appear
3. Rename it to \`"limelight"\`
4. Save the configuration

### Imports
\`\`\`java
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
\`\`\`

### Basic Setup
\`\`\`java
Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");

// Set how frequently to poll for data (100 = maximum freshness)
limelight.setPollRateHz(100);

// Select which pipeline to use (0-9, configured in web UI)
limelight.pipelineSwitch(0);

// Start polling — REQUIRED. getLatestResult() returns null without this.
limelight.start();
\`\`\`

### Available Pipeline Types (Configured in Web UI)
| Pipeline Type | Description | FPS |
|---|---|---|
| **AprilTag / Fiducial** | Detects AprilTags, computes 6DOF pose, MegaTag localization | 20-50 |
| **Color Blob** | Tracks colored objects by HSV range | 90 |
| **Neural Network Detector** | Object detection using ML model (CPU inference) | 15-30 |
| **Neural Network Classifier** | Image classification using ML model | 15-30 |
| **Barcode / QR Code** | Reads barcodes and QR codes | 30-60 |
| **Python SnapScript** | Custom OpenCV + numpy pipeline | Varies |

### Reading Results — Universal Pattern
\`\`\`java
LLResult result = limelight.getLatestResult();
if (result != null && result.isValid()) {
    // Basic targeting (works for ALL pipeline types)
    double tx = result.getTx();     // horizontal offset to target (degrees)
    double ty = result.getTy();     // vertical offset to target (degrees)
    double ta = result.getTa();     // target area (0-100% of image)

    // Latency
    double captureLatency   = result.getCaptureLatency();    // ms from exposure to tracking
    double targetingLatency = result.getTargetingLatency();  // ms for tracking computation
    double totalLatency     = captureLatency + targetingLatency;

    // Data freshness
    double staleness = result.getStaleness();  // age in ms (< 100ms is fresh)
}
\`\`\`

### AprilTag / Fiducial Results
\`\`\`java
List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
for (LLResultTypes.FiducialResult fr : fiducials) {
    int tagId   = fr.getFiducialId();
    String family = fr.getFamily();            // "36H11C" for FTC tags
    double tx   = fr.getTargetXDegrees();      // horizontal offset
    double ty   = fr.getTargetYDegrees();      // vertical offset
    double area = fr.getTargetArea();           // target area (0-100)

    // 3D pose data (requires "Full 3D" enabled in pipeline Advanced tab)
    Pose3D robotPoseTag    = fr.getRobotPoseTargetSpace();    // robot relative to this tag
    Pose3D cameraPoseTag   = fr.getCameraPoseTargetSpace();   // camera relative to this tag
    Pose3D robotPoseField  = fr.getRobotPoseFieldSpace();     // robot in field space (from this tag)
    Pose3D tagPoseCamera   = fr.getTargetPoseCameraSpace();   // tag in camera coordinate system
    Pose3D tagPoseRobot    = fr.getTargetPoseRobotSpace();    // tag in robot coordinate system
}
\`\`\`

### Color Blob Results
\`\`\`java
List<LLResultTypes.ColorResult> colors = result.getColorResults();
for (LLResultTypes.ColorResult cr : colors) {
    double tx   = cr.getTargetXDegrees();  // horizontal offset
    double ty   = cr.getTargetYDegrees();  // vertical offset
    double area = cr.getTargetArea();       // blob area (0-100)
}
\`\`\`

### Neural Network Detector Results
\`\`\`java
List<LLResultTypes.DetectorResult> detections = result.getDetectorResults();
for (LLResultTypes.DetectorResult dr : detections) {
    String className = dr.getClassName();      // detected class name
    double tx   = dr.getTargetXDegrees();
    double ty   = dr.getTargetYDegrees();
    double area = dr.getTargetArea();
}
\`\`\`

### Neural Network Classifier Results
\`\`\`java
List<LLResultTypes.ClassifierResult> classResults = result.getClassifierResults();
for (LLResultTypes.ClassifierResult cr : classResults) {
    String className  = cr.getClassName();    // classified class name
    double confidence = cr.getConfidence();   // confidence score (0-1)
}
\`\`\`

### Barcode / QR Code Results
\`\`\`java
List<LLResultTypes.BarcodeResult> barcodes = result.getBarcodeResults();
for (LLResultTypes.BarcodeResult br : barcodes) {
    String data   = br.getData();    // barcode content string
    String family = br.getFamily();  // barcode type (e.g., "QR")
}
\`\`\`

### Status Monitoring
\`\`\`java
LLStatus status = limelight.getStatus();
String name     = status.getName();           // device hostname
double temp     = status.getTemp();           // CPU temperature (Celsius)
double cpu      = status.getCpu();            // CPU usage (percentage)
double fps      = status.getFps();            // current processing FPS
int pipelineIdx = status.getPipelineIndex();  // current pipeline index
String pipeType = status.getPipelineType();   // current pipeline type string
\`\`\`

### Lifecycle
\`\`\`java
limelight.start();   // begin polling (REQUIRED)
limelight.stop();    // stop polling (call in stop() or end of OpMode)
\`\`\`

### Complete Example
\`\`\`java
@TeleOp(name = "Limelight Demo", group = "Vision")
public class LimelightDemo extends LinearOpMode {
    @Override
    public void runOpMode() {
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);  // fast telemetry for Limelight
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(0);
        limelight.start();

        waitForStart();

        while (opModeIsActive()) {
            LLStatus status = limelight.getStatus();
            telemetry.addData("LL", "Temp:%.1fC CPU:%.0f%% FPS:%d",
                status.getTemp(), status.getCpu(), (int) status.getFps());

            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                telemetry.addData("tx/ty", "%.1f / %.1f", result.getTx(), result.getTy());
                telemetry.addData("Latency", "%.0fms",
                    result.getCaptureLatency() + result.getTargetingLatency());
                telemetry.addData("Staleness", "%.0fms", result.getStaleness());

                for (LLResultTypes.FiducialResult fr : result.getFiducialResults()) {
                    telemetry.addData("Tag " + fr.getFiducialId(),
                        "tx:%.1f ty:%.1f area:%.1f",
                        fr.getTargetXDegrees(), fr.getTargetYDegrees(), fr.getTargetArea());
                }
            } else {
                telemetry.addData("Result", "No valid targets");
            }
            telemetry.update();
        }
        limelight.stop();
    }
}
\`\`\`
`,

  megaTag: `
## MegaTag — Robot Localization from AprilTags

The Limelight 3A offers two robot localization modes that compute your robot's position on the
field from visible AprilTags. These are the most powerful features of the Limelight for FTC.

### Prerequisites (Both MegaTag Modes)
1. Configure the **camera's position relative to robot center** in the Limelight web UI
   (Input tab → Camera Pose). Measure X/Y/Z offset and rotation accurately.
2. Upload or select the **field map** (the Limelight ships with the current FTC season's map)
3. Enable **"Full 3D"** in the AprilTag pipeline's Advanced tab

### MegaTag1 — Multi-Tag Fusion (No IMU Required)

MegaTag1 uses multiple simultaneously-visible AprilTags to compute a stable robot pose.
It does NOT require an external IMU.

**How it works:**
- When 2+ tags are visible, the pose is very stable and accurate
- With only 1 tag, there is risk of pose ambiguity (flip between two possible poses)
  especially when the camera is head-on to the tag
- No external sensor fusion — purely vision-based

**Code:**
\`\`\`java
LLResult result = limelight.getLatestResult();
if (result != null && result.isValid()) {
    Pose3D botpose = result.getBotpose();  // MegaTag1 result
    if (botpose != null) {
        double x       = botpose.getPosition().x;  // field X (meters)
        double y       = botpose.getPosition().y;  // field Y (meters)
        double z       = botpose.getPosition().z;  // field Z (meters)
        double yaw     = botpose.getOrientation().getYaw();    // heading (degrees)
        double pitch   = botpose.getOrientation().getPitch();
        double roll    = botpose.getOrientation().getRoll();
    }
}
\`\`\`

**When to use MegaTag1:**
- Robot doesn't have a reliable IMU
- Multiple tags are usually visible (MT1 shines with 2+ tags)
- Backup when IMU is unreliable

### MegaTag2 — IMU-Fused Localization (Recommended)

MegaTag2 fuses the robot's IMU heading with AprilTag detections for dramatically improved
accuracy. It eliminates the single-tag ambiguity problem entirely.

**How it works:**
- You provide the robot's current heading (yaw) from the IMU each frame
- The Limelight uses this heading to constrain the pose solution
- Even a single tag gives excellent localization (no ambiguity)
- Works reliably at any distance where the tag is detectable
- More accurate than MegaTag1 in almost all scenarios

**Code:**
\`\`\`java
IMU imu = hardwareMap.get(IMU.class, "imu");
// ... initialize IMU ...

// In loop — MUST call updateRobotOrientation() EVERY iteration BEFORE getting results:
double yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
limelight.updateRobotOrientation(yaw);

LLResult result = limelight.getLatestResult();
if (result != null && result.isValid()) {
    Pose3D botpose = result.getBotpose_MT2();  // MegaTag2 result
    if (botpose != null) {
        double x       = botpose.getPosition().x;  // field X (meters)
        double y       = botpose.getPosition().y;  // field Y (meters)
        double heading = botpose.getOrientation().getYaw();  // field heading (degrees)

        telemetry.addData("MT2 Pose", "X:%.2f Y:%.2f Heading:%.1f",
            x, y, heading);
    }
}
\`\`\`

**CRITICAL:** \`updateRobotOrientation()\` must be called every loop iteration BEFORE
\`getLatestResult()\`. If you skip it, MegaTag2 results will be inaccurate or unavailable.

**When to use MegaTag2:**
- Always, if you have a working IMU (which every FTC robot does via the Control Hub)
- Best-in-class FTC robot localization
- Works even with a single visible tag

### MegaTag1 vs MegaTag2 Comparison

| Feature | MegaTag1 | MegaTag2 |
|---|---|---|
| IMU required | No | Yes (provide yaw each frame) |
| Single tag accuracy | Poor (ambiguity risk) | Excellent |
| Multi-tag accuracy | Excellent | Excellent |
| Heading accuracy | Vision-only | IMU-fused (more stable) |
| Setup complexity | Simpler | Slightly more code |
| Recommended | Backup option | Primary choice |

### FTC Field Coordinate System (Limelight)
- Origin (0, 0, 0) = center of the field, on the floor
- Units: meters (NOT inches — convert if needed)
- 0° yaw: varies by field map configuration
- Configure in the Limelight web UI to match your desired convention

### Integrating MegaTag with Pedro Pathing / Odometry
A common pattern is to use MegaTag2 for periodic position corrections on top of
continuous odometry tracking:

\`\`\`java
// In loop:
limelight.updateRobotOrientation(getCurrentHeading());
LLResult result = limelight.getLatestResult();

if (result != null && result.isValid()) {
    Pose3D mt2 = result.getBotpose_MT2();
    if (mt2 != null && result.getStaleness() < 100) {
        double confidence = 1.0 / (1.0 + result.getStaleness() / 50.0);
        // Only correct if confidence is high and data is fresh
        if (confidence > 0.7) {
            // Convert meters to inches (Pedro uses inches)
            double xInches = mt2.getPosition().x * 39.3701;
            double yInches = mt2.getPosition().y * 39.3701;
            // Apply correction to your odometry / path follower
            // follower.setPose(new Pose(xInches, yInches, heading));
        }
    }
}
\`\`\`

### Python SnapScript Pipelines (Custom Limelight Vision)
The Limelight can run custom Python pipelines using OpenCV 4.10, numpy, and tensorflow:

\`\`\`python
# Written in the Limelight web UI's SnapScript editor
def runPipeline(image, llrobot):
    # image: BGR numpy array from camera
    # llrobot: double[8] from robot code (via updatePythonInputs())

    import cv2
    import numpy as np

    # Example: detect yellow objects
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower = np.array([20, 100, 100])
    upper = np.array([30, 255, 255])
    mask = cv2.inRange(hsv, lower, upper)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    largestContour = np.array([[]])
    llpython = [0, 0, 0, 0, 0, 0, 0, 0]

    if len(contours) > 0:
        largestContour = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(largestContour)
        llpython[0] = x + w / 2    # center x
        llpython[1] = y + h / 2    # center y
        llpython[2] = w * h         # area
        cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)

    # Returns: (contour for crosshair, annotated image, output array)
    return largestContour, image, llpython
\`\`\`

**Robot-side code to communicate with SnapScript:**
\`\`\`java
// Send data TO Python pipeline
double[] inputs = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0};
limelight.updatePythonInputs(inputs);

// Read data FROM Python pipeline
double[] output = result.getPythonOutput();
double centerX = output[0];
double centerY = output[1];
double area    = output[2];
\`\`\`

### Snapshots (Offline Debugging)
\`\`\`java
limelight.captureSnapshot("auto_start");  // save current frame for later analysis
limelight.deleteSnapshots();               // clear all saved snapshots
\`\`\`
View snapshots in the Limelight web UI for debugging pipeline issues without the robot running.
`,

  colorDetection: `
## Color Detection — Custom VisionProcessor with OpenCV

For detecting game elements by color (samples, specimens, etc.), you write a custom
VisionProcessor that uses OpenCV for HSV thresholding and contour analysis. This runs
on the Control Hub CPU via VisionPortal.

### How Custom VisionProcessors Work
\`\`\`
Camera Frame (RGB/YUV) → VisionPortal → Your VisionProcessor.processFrame() → Results
                                        └→ Annotated frame for preview/Dashboard
\`\`\`

The \`VisionProcessor\` interface has three methods:
1. \`init()\` — called once when the portal starts
2. \`processFrame()\` — called for each camera frame, does the actual detection
3. \`onDrawFrame()\` — called to draw annotations on the preview

### Imports
\`\`\`java
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import android.graphics.Canvas;
\`\`\`

### Complete Color Detection Processor
\`\`\`java
public class SampleColorProcessor implements VisionProcessor {

    // Detection result — volatile for thread safety (processFrame runs on a camera thread)
    public volatile DetectionResult latestResult = new DetectionResult();

    // HSV ranges for different sample colors
    // Tune these using FTC Dashboard or the Limelight web UI as reference
    // H: 0-179, S: 0-255, V: 0-255 in OpenCV
    private Scalar lowerBound;
    private Scalar upperBound;

    // Reusable Mats (allocate once, reuse to avoid GC pressure)
    private Mat hsv = new Mat();
    private Mat mask = new Mat();
    private Mat hierarchy = new Mat();

    public enum SampleColor { RED, BLUE, YELLOW }

    public static class DetectionResult {
        public boolean detected = false;
        public double centerX = 0;     // pixels
        public double centerY = 0;     // pixels
        public double width = 0;       // pixels
        public double height = 0;      // pixels
        public double area = 0;        // pixels^2
        public double aspectRatio = 0;
    }

    public SampleColorProcessor(SampleColor color) {
        switch (color) {
            case RED:
                // Red wraps around the hue circle — handle both ends
                // We'll handle this specially in processFrame
                lowerBound = new Scalar(0, 120, 70);
                upperBound = new Scalar(10, 255, 255);
                break;
            case BLUE:
                lowerBound = new Scalar(100, 120, 70);
                upperBound = new Scalar(130, 255, 255);
                break;
            case YELLOW:
                lowerBound = new Scalar(18, 120, 100);
                upperBound = new Scalar(32, 255, 255);
                break;
        }
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        // Called once when the portal starts streaming
        // Can use width/height to set up ROI or scaling
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        // Convert BGR to HSV color space
        Imgproc.cvtColor(frame, hsv, Imgproc.COLOR_RGB2HSV);

        // Threshold to create binary mask
        Core.inRange(hsv, lowerBound, upperBound, mask);

        // For RED: also capture the high-hue range (170-179)
        if (lowerBound.val[0] < 15) {
            Mat mask2 = new Mat();
            Core.inRange(hsv, new Scalar(170, 120, 70), new Scalar(179, 255, 255), mask2);
            Core.bitwise_or(mask, mask2, mask);
            mask2.release();
        }

        // Morphological operations to clean up noise
        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
        Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_OPEN, kernel);   // remove small noise
        Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_CLOSE, kernel);  // fill small holes
        kernel.release();

        // Find contours
        java.util.List<MatOfPoint> contours = new java.util.ArrayList<>();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL,
                             Imgproc.CHAIN_APPROX_SIMPLE);

        // Find the largest contour
        DetectionResult result = new DetectionResult();
        double maxArea = 0;
        Rect bestRect = null;

        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area > 500 && area > maxArea) {  // minimum area filter
                maxArea = area;
                bestRect = Imgproc.boundingRect(contour);
                result.detected = true;
                result.area = area;
            }
        }

        if (bestRect != null) {
            result.centerX = bestRect.x + bestRect.width / 2.0;
            result.centerY = bestRect.y + bestRect.height / 2.0;
            result.width = bestRect.width;
            result.height = bestRect.height;
            result.aspectRatio = (double) bestRect.width / bestRect.height;

            // Draw bounding box on frame (visible in LiveView and Dashboard)
            Imgproc.rectangle(frame,
                new Point(bestRect.x, bestRect.y),
                new Point(bestRect.x + bestRect.width, bestRect.y + bestRect.height),
                new Scalar(0, 255, 0), 2);  // green box, 2px thick

            // Draw center crosshair
            Imgproc.circle(frame, new Point(result.centerX, result.centerY),
                5, new Scalar(255, 0, 0), -1);  // red dot

            // Draw label
            Imgproc.putText(frame, String.format("Area: %.0f", result.area),
                new Point(bestRect.x, bestRect.y - 10),
                Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 1);
        }

        // Release contour Mats
        for (MatOfPoint contour : contours) {
            contour.release();
        }

        latestResult = result;

        // Return value is passed to onDrawFrame — can be used for Canvas drawing
        return result;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                            float scaleBmpPxToCanvasPx, float scaleCanvasDensity,
                            Object userContext) {
        // Optional: draw Android Canvas overlays on the LiveView
        // The OpenCV drawing in processFrame already handles most annotation needs
    }
}
\`\`\`

### Using the Custom Processor with VisionPortal
\`\`\`java
@Autonomous(name = "Color Detection Auto", group = "Vision")
public class ColorDetectionAuto extends LinearOpMode {
    @Override
    public void runOpMode() {
        SampleColorProcessor yellowDetector = new SampleColorProcessor(
            SampleColorProcessor.SampleColor.YELLOW);

        VisionPortal portal = new VisionPortal.Builder()
            .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
            .addProcessor(yellowDetector)
            .setCameraResolution(new Size(640, 480))
            .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
            .build();

        // Set manual white balance for consistent color detection
        while (!isStopRequested() &&
               portal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            sleep(20);
        }
        WhiteBalanceControl wb = portal.getCameraControl(WhiteBalanceControl.class);
        wb.setMode(WhiteBalanceControl.Mode.MANUAL);
        wb.setWhiteBalanceTemperature(4600);

        // Detect during INIT
        while (opModeInInit()) {
            SampleColorProcessor.DetectionResult result = yellowDetector.latestResult;
            if (result.detected) {
                telemetry.addData("Yellow Sample", "Found at (%.0f, %.0f) area=%.0f",
                    result.centerX, result.centerY, result.area);
            } else {
                telemetry.addData("Yellow Sample", "Not detected");
            }
            telemetry.update();
        }

        // Use detection result in autonomous
        SampleColorProcessor.DetectionResult finalResult = yellowDetector.latestResult;
        if (finalResult.detected) {
            // Navigate to the detected sample...
        }

        portal.close();
    }
}
\`\`\`

### HSV Tuning Guide (OpenCV Convention)

OpenCV uses a different HSV range than most tools:
- **H (Hue):** 0-179 (NOT 0-360). Divide typical hue values by 2.
- **S (Saturation):** 0-255
- **V (Value/Brightness):** 0-255

**Common FTC game element colors:**

| Color | H Low | H High | S Low | S High | V Low | V High | Notes |
|---|---|---|---|---|---|---|---|
| Red (low) | 0 | 10 | 120 | 255 | 70 | 255 | Red wraps — need both ranges |
| Red (high) | 170 | 179 | 120 | 255 | 70 | 255 | Combine with OR |
| Blue | 100 | 130 | 120 | 255 | 70 | 255 | |
| Yellow | 18 | 32 | 120 | 255 | 100 | 255 | |
| Green | 40 | 80 | 100 | 255 | 70 | 255 | |
| Orange | 10 | 22 | 150 | 255 | 100 | 255 | |
| White | 0 | 179 | 0 | 40 | 200 | 255 | Low saturation, high value |

**IMPORTANT:** These are starting points. ALWAYS tune HSV ranges at the competition venue under
actual lighting conditions. Ranges that work in your shop may fail under competition LEDs.

### HSV Tuning Tips
1. Use the Limelight web UI or a desktop HSV tool to find initial ranges
2. Make the ranges tunable via \`@Config public static\` fields:
\`\`\`java
@Config
public class VisionConstants {
    public static double H_LOW = 18;
    public static double H_HIGH = 32;
    public static double S_LOW = 120;
    public static double S_HIGH = 255;
    public static double V_LOW = 100;
    public static double V_HIGH = 255;
    public static double MIN_AREA = 500;
}
\`\`\`
3. Tune live via FTC Dashboard at the competition venue
4. Set manual white balance to avoid color shifts between venues

### Running Multiple Processors Simultaneously
\`\`\`java
// AprilTag + color detection on the same camera
AprilTagProcessor aprilTag = new AprilTagProcessor.Builder().build();
SampleColorProcessor colorProc = new SampleColorProcessor(SampleColorProcessor.SampleColor.YELLOW);

VisionPortal portal = new VisionPortal.Builder()
    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
    .addProcessor(aprilTag)
    .addProcessor(colorProc)
    .setCameraResolution(new Size(640, 480))
    .build();

// Toggle processors as needed to save CPU
portal.setProcessorEnabled(colorProc, false);  // disable color during driving
portal.setProcessorEnabled(aprilTag, true);     // keep AprilTag active
\`\`\`

### OpenCV Drawing Functions Reference
\`\`\`java
// Rectangle
Imgproc.rectangle(frame, new Point(x1, y1), new Point(x2, y2),
    new Scalar(0, 255, 0), 2);  // color (BGR), thickness

// Circle
Imgproc.circle(frame, new Point(cx, cy), radius,
    new Scalar(255, 0, 0), -1);  // -1 = filled

// Line
Imgproc.line(frame, new Point(x1, y1), new Point(x2, y2),
    new Scalar(0, 0, 255), 2);

// Text
Imgproc.putText(frame, "Hello", new Point(x, y),
    Imgproc.FONT_HERSHEY_SIMPLEX, 0.7, new Scalar(255, 255, 255), 2);
\`\`\`
`,

  visionOptimization: `
## Vision Optimization — Maximum FPS, Minimum Loop Impact

Vision processing can be the biggest CPU consumer on the Control Hub. These optimization
techniques are the difference between a 10 FPS stuttering pipeline and a smooth 30+ FPS system.

### The Loop Time Impact Problem

Every frame processed by VisionPortal runs on the Control Hub CPU. If vision takes too long,
it can starve your control loop of CPU time, causing jerky driving and slow responses.

**Baseline measurements (unoptimized, Logitech C920 at 640x480):**
- AprilTag only: ~10-15 FPS (decimation=1), ~30 FPS (decimation=3)
- Custom OpenCV color detection: ~15-25 FPS depending on complexity
- AprilTag + color detection: ~8-12 FPS

**Optimized targets:**
- AprilTag: 30 FPS (decimation=3)
- Color detection: 25-30 FPS
- Combined: 15-20 FPS

### 1. Resolution Selection (Biggest Impact)

Lower resolution = fewer pixels to process = dramatically faster.

| Resolution | Pixels | Relative Speed | AprilTag Range | Color Detection |
|---|---|---|---|---|
| 320x240 | 76.8K | Fastest (~3x) | Short (2-4 feet) | Usable for close objects |
| 640x480 | 307.2K | Baseline | Medium (5-10 feet) | Good for most FTC tasks |
| 800x600 | 480K | Slower | Good | Better quality |
| 1280x720 | 921.6K | Much slower (~3x) | Long (15+ feet) | Best quality |
| 1920x1080 | 2.07M | Very slow (~7x) | Longest | Overkill for most FTC |

**Recommendation:** Use **640x480** for most FTC tasks. Only go higher for long-range
AprilTag detection during init (when FPS doesn't matter). Drop to **320x240** if you
need absolute maximum FPS for simple color detection.

### 2. AprilTag Decimation (Most Important for AprilTag FPS)

Decimation downsamples the image before tag detection. Set it at runtime:
\`\`\`java
aprilTag.setDecimation(3);  // 1 = full resolution, 2 = half, 3 = third
\`\`\`

**Strategy — change decimation dynamically:**
\`\`\`java
// During INIT: max range, FPS doesn't matter
aprilTag.setDecimation(1);

// During DRIVING: need 30 FPS, shorter range OK
aprilTag.setDecimation(3);

// During ALIGNMENT: balance range and speed
aprilTag.setDecimation(2);
\`\`\`

Decimation does NOT affect pose accuracy — it only reduces detection range.

### 3. Exposure Settings for Motion Blur

Low exposure is critical when the robot is moving. Motion blur destroys AprilTag detection.

\`\`\`java
// BEFORE: auto exposure = 30-50ms = blurry tags while driving
// AFTER: manual 5-6ms = sharp tags while driving

ExposureControl exposure = portal.getCameraControl(ExposureControl.class);
exposure.setMode(ExposureControl.Mode.Manual);
sleep(50);
exposure.setExposure(6, TimeUnit.MILLISECONDS);
GainControl gain = portal.getCameraControl(GainControl.class);
gain.setGain(250);  // compensate for dark image
\`\`\`

**This alone can improve AprilTag detection rate from 30% to 95% while driving.**

### 4. Disable LiveView in Competition

LiveView renders the camera preview on the Robot Controller screen. This costs CPU.
In competition, no one is looking at the screen — disable it:

\`\`\`java
// Option A: disable at build time
.enableLiveView(false)

// Option B: disable at runtime
portal.stopLiveView();
\`\`\`

**Savings:** ~2-5 FPS improvement, plus reduced GPU usage.

### 5. Processor Toggling (Enable Only When Needed)

Don't run processors you don't need right now:

\`\`\`java
// During driving — only need AprilTag for alignment
portal.setProcessorEnabled(colorProcessor, false);
portal.setProcessorEnabled(aprilTag, true);

// During intake — only need color detection
portal.setProcessorEnabled(aprilTag, false);
portal.setProcessorEnabled(colorProcessor, true);

// Full stop — disable all vision processing
portal.setProcessorEnabled(aprilTag, false);
portal.setProcessorEnabled(colorProcessor, false);
\`\`\`

This is the fastest toggle — no camera restart required.

### 6. Stream Format Choice

\`\`\`java
// YUY2 (default): uncompressed, higher quality, uses more USB bandwidth
.setStreamFormat(VisionPortal.StreamFormat.YUY2)

// MJPEG: compressed, saves USB bandwidth, slight quality loss
.setStreamFormat(VisionPortal.StreamFormat.MJPEG)
\`\`\`

**Use MJPEG when:**
- Running dual cameras on shared USB hub
- Camera is on a USB 2.0 port with other devices
- Streaming to FTC Dashboard (MJPEG is already compressed for network)
- Resolution is 640x480 or higher

**Use YUY2 when:**
- Single camera on dedicated USB port
- Need best possible image quality
- Low resolution (320x240 — bandwidth isn't an issue)

### 7. Region of Interest (ROI) in Custom Processors

Don't process the entire frame if you know where the target will be:

\`\`\`java
@Override
public Object processFrame(Mat frame, long captureTimeNanos) {
    // Only process the bottom half of the frame (where samples are)
    int halfHeight = frame.rows() / 2;
    Mat roi = frame.submat(halfHeight, frame.rows(), 0, frame.cols());

    Imgproc.cvtColor(roi, hsv, Imgproc.COLOR_RGB2HSV);
    Core.inRange(hsv, lowerBound, upperBound, mask);

    // ... contour detection on smaller image ...

    // IMPORTANT: adjust detected coordinates back to full-frame
    // Add halfHeight to any Y coordinates
    result.centerY += halfHeight;

    roi.release();  // release the submat view
    return result;
}
\`\`\`

**Savings:** Processing half the pixels = roughly 2x faster pipeline.

### 8. Minimize Object Allocation in processFrame()

The \`processFrame()\` method runs 30+ times per second. Avoid allocating new objects:

\`\`\`java
// BAD — allocates new Mats every frame (GC pressure)
public Object processFrame(Mat frame, long captureTimeNanos) {
    Mat hsv = new Mat();          // NEW every frame
    Mat mask = new Mat();         // NEW every frame
    // ...
}

// GOOD — reuse pre-allocated Mats
private Mat hsv = new Mat();      // allocated once
private Mat mask = new Mat();     // allocated once

public Object processFrame(Mat frame, long captureTimeNanos) {
    Imgproc.cvtColor(frame, hsv, Imgproc.COLOR_RGB2HSV);  // reuses hsv
    Core.inRange(hsv, lower, upper, mask);                  // reuses mask
    // ...
}
\`\`\`

Also avoid creating new \`Scalar\`, \`Point\`, \`Size\` objects in processFrame if possible.
Pre-allocate them as instance fields.

### 9. Reduce Morphological Operations

Each morphological operation (erode, dilate, open, close) adds processing time:

\`\`\`java
// Expensive: 3 morphological operations
Imgproc.erode(mask, mask, kernel);
Imgproc.dilate(mask, mask, kernel);
Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_CLOSE, kernel);

// Cheaper: just one OPEN (removes noise) if that's sufficient
Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_OPEN, kernel);
\`\`\`

Or skip morphological operations entirely if your HSV range is tight enough.

### 10. Contour Filtering (Reject Early)

Filter contours by area immediately to avoid processing irrelevant detections:

\`\`\`java
for (MatOfPoint contour : contours) {
    double area = Imgproc.contourArea(contour);
    if (area < 500) continue;  // skip tiny noise contours immediately

    Rect rect = Imgproc.boundingRect(contour);
    double aspect = (double) rect.width / rect.height;
    if (aspect < 0.3 || aspect > 3.0) continue;  // skip unlikely aspect ratios

    // Only expensive processing on likely candidates
    // ...
}
\`\`\`

### 11. Limelight vs VisionPortal Performance

Since the Limelight runs on its own processor, it has **zero impact on Control Hub loop time**.
This is the Limelight's biggest advantage for performance-sensitive teams.

\`\`\`
VisionPortal (USB webcam):        Limelight 3A:
┌─────────────────────┐           ┌──────────────────┐
│   Control Hub CPU   │           │  Limelight CPU   │
│                     │           │  (processes       │
│ ┌─────────────────┐ │           │   frames here)   │
│ │ Your loop()     │ │           └──────────────────┘
│ │  + motor writes │ │                    │
│ │  + sensor reads │ │             USB (results only)
│ │  + vision proc  │◄── competes!        │
│ └─────────────────┘ │           ┌──────────────────┐
│                     │           │   Control Hub    │
└─────────────────────┘           │                  │
                                  │ ┌──────────────┐ │
                                  │ │ Your loop()  │ │
                                  │ │ + motor      │ │
                                  │ │ + sensor     │ │
                                  │ │ (no vision!) │ │
                                  │ └──────────────┘ │
                                  └──────────────────┘
\`\`\`

### 12. Telemetry Throttling for Vision

Telemetry transmission can bottleneck vision data flow:

\`\`\`java
// Reduce telemetry transmission interval for faster vision data display
telemetry.setMsTransmissionInterval(11);  // 11ms instead of default 250ms
\`\`\`

### Optimization Checklist

| Optimization | Impact | Complexity | When to Apply |
|---|---|---|---|
| Set decimation=3 | High | Easy | Always for driving |
| Lower resolution to 640x480 | High | Easy | Unless need long range |
| Manual exposure 5-6ms | High | Easy | Always for AprilTags |
| Disable LiveView | Medium | Easy | Competition |
| Toggle unused processors | Medium | Easy | When switching tasks |
| Use MJPEG format | Medium | Easy | Dual cameras or Dashboard |
| ROI cropping | High | Moderate | When target position is predictable |
| Pre-allocate Mats | Medium | Moderate | Custom processors |
| Reduce morphological ops | Low-Med | Easy | Custom processors |
| Use Limelight instead | Highest | Moderate | If budget allows |
`,

  multiCamera: `
## Multi-Camera Setup (Dual Webcams)

FTC robots can run two USB webcams simultaneously using VisionPortal's MultiPortal feature.
Common use case: front camera for AprilTags, rear camera for intake/color detection.

### MultiPortal Setup
\`\`\`java
// Step 1: Create viewport IDs for split-screen preview
int[] viewportIds = VisionPortal.makeMultiPortalView(2,
    VisionPortal.MultiPortalLayout.HORIZONTAL);
// OR: VisionPortal.MultiPortalLayout.VERTICAL

// Step 2: Create separate processors for each camera
AprilTagProcessor frontAprilTag = new AprilTagProcessor.Builder()
    .setDrawAxes(true)
    .build();

SampleColorProcessor rearColor = new SampleColorProcessor(
    SampleColorProcessor.SampleColor.YELLOW);

// Step 3: Build two separate portals
VisionPortal frontPortal = new VisionPortal.Builder()
    .setCamera(hardwareMap.get(WebcamName.class, "Webcam Front"))
    .addProcessor(frontAprilTag)
    .setCameraResolution(new Size(640, 480))
    .setCameraMonitorViewId(viewportIds[0])
    .build();

VisionPortal rearPortal = new VisionPortal.Builder()
    .setCamera(hardwareMap.get(WebcamName.class, "Webcam Rear"))
    .addProcessor(rearColor)
    .setCameraResolution(new Size(320, 240))   // lower res for color = faster
    .setCameraMonitorViewId(viewportIds[1])
    .setStreamFormat(VisionPortal.StreamFormat.MJPEG)  // save USB bandwidth
    .build();
\`\`\`

### USB Bandwidth Considerations

The Control Hub has two USB buses:
- **USB 2.0 bus** — also carries the WiFi radio (shared bandwidth!)
- **USB 3.0 bus** — separate, higher bandwidth

**Best practice:**
- Put the primary camera on USB 3.0 (blue port)
- Put the secondary camera on USB 2.0 (black port) with MJPEG + lower resolution
- OR use a USB hub on the 3.0 port, but both cameras should use MJPEG

**If cameras share a USB hub:**
- Use MJPEG format for both cameras
- Reduce resolution on at least one camera
- Total bandwidth must stay under USB 2.0 limits (~40 MB/s effective)

| Config | Resolution | Format | Bandwidth per camera |
|---|---|---|---|
| 640x480 YUY2 | 640x480 | YUY2 | ~18 MB/s at 30 FPS |
| 640x480 MJPEG | 640x480 | MJPEG | ~3-5 MB/s at 30 FPS |
| 320x240 MJPEG | 320x240 | MJPEG | ~1-2 MB/s at 30 FPS |

### Lifecycle Management with Multiple Cameras
\`\`\`java
// Disable processors independently
frontPortal.setProcessorEnabled(frontAprilTag, false);   // save CPU
rearPortal.setProcessorEnabled(rearColor, true);         // keep active

// Close portals in stop()
frontPortal.close();
rearPortal.close();
\`\`\`

### Webcam + Limelight (Different Systems)
You can run a USB webcam with VisionPortal AND a Limelight 3A simultaneously.
They use separate USB ports and separate processing:

\`\`\`java
// USB webcam for color detection (runs on Control Hub CPU)
SampleColorProcessor colorProc = new SampleColorProcessor(SampleColorProcessor.SampleColor.YELLOW);
VisionPortal webcamPortal = new VisionPortal.Builder()
    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
    .addProcessor(colorProc)
    .setCameraResolution(new Size(640, 480))
    .build();

// Limelight for AprilTag localization (runs on Limelight's own CPU)
Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
limelight.setPollRateHz(100);
limelight.pipelineSwitch(0);  // AprilTag pipeline
limelight.start();

// In loop — both systems provide data independently
SampleColorProcessor.DetectionResult color = colorProc.latestResult;
LLResult llResult = limelight.getLatestResult();
\`\`\`
`,

  visionPatterns: `
## Common FTC Vision Patterns (Recipes)

### 1. Init-Phase Detection (Randomized Game Element)

Many FTC games require detecting a randomized element position during the INIT phase
(before the match starts). The detection result determines the autonomous path.

\`\`\`java
@Autonomous(name = "Auto with Init Detection", group = "Competition")
public class AutoWithInitDetection extends LinearOpMode {

    private enum ElementPosition { LEFT, CENTER, RIGHT, UNKNOWN }
    private volatile ElementPosition detectedPosition = ElementPosition.UNKNOWN;

    @Override
    public void runOpMode() {
        SampleColorProcessor detector = new SampleColorProcessor(
            SampleColorProcessor.SampleColor.RED);

        VisionPortal portal = new VisionPortal.Builder()
            .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
            .addProcessor(detector)
            .setCameraResolution(new Size(640, 480))
            .build();

        // Detect during the entire INIT phase
        while (opModeInInit()) {
            SampleColorProcessor.DetectionResult result = detector.latestResult;

            if (result.detected) {
                // Determine position based on X coordinate
                double frameWidth = 640;
                if (result.centerX < frameWidth / 3.0) {
                    detectedPosition = ElementPosition.LEFT;
                } else if (result.centerX < 2 * frameWidth / 3.0) {
                    detectedPosition = ElementPosition.CENTER;
                } else {
                    detectedPosition = ElementPosition.RIGHT;
                }
            }

            telemetry.addData("Detected", detectedPosition);
            if (result.detected) {
                telemetry.addData("At", "(%.0f, %.0f)", result.centerX, result.centerY);
            }
            telemetry.update();
        }

        // Match started — disable vision to save CPU for driving
        portal.setProcessorEnabled(detector, false);

        // Execute autonomous based on detection
        switch (detectedPosition) {
            case LEFT:
                runLeftPath();
                break;
            case CENTER:
                runCenterPath();
                break;
            case RIGHT:
                runRightPath();
                break;
            default:
                runDefaultPath();  // fallback if detection failed
                break;
        }

        portal.close();
    }

    private void runLeftPath()   { /* ... */ }
    private void runCenterPath() { /* ... */ }
    private void runRightPath()  { /* ... */ }
    private void runDefaultPath(){ /* ... */ }
}
\`\`\`

### 2. Drive-to-AprilTag Alignment (Mecanum)

Use AprilTag bearing and range for precise alignment with a scoring target:

\`\`\`java
@Config
public class AlignmentConstants {
    public static double DESIRED_RANGE   = 12.0;  // inches from tag
    public static double SPEED_GAIN      = 0.02;  // proportional gain for range
    public static double STRAFE_GAIN     = 0.015; // proportional gain for lateral
    public static double TURN_GAIN       = 0.01;  // proportional gain for heading
    public static double MAX_DRIVE_SPEED = 0.5;
    public static double MAX_STRAFE      = 0.5;
    public static double MAX_TURN        = 0.3;
    public static int    TARGET_TAG_ID   = 1;     // which tag to align to
}

// In your OpMode loop:
private void alignToAprilTag(AprilTagProcessor aprilTag) {
    List<AprilTagDetection> detections = aprilTag.getDetections();

    AprilTagDetection targetTag = null;
    for (AprilTagDetection det : detections) {
        if (det.id == AlignmentConstants.TARGET_TAG_ID && det.metadata != null) {
            targetTag = det;
            break;
        }
    }

    if (targetTag != null) {
        double rangeError   = targetTag.ftcPose.range - AlignmentConstants.DESIRED_RANGE;
        double headingError = targetTag.ftcPose.bearing;
        double yawError     = targetTag.ftcPose.yaw;

        // Proportional control
        double drive  = Range.clip(rangeError * AlignmentConstants.SPEED_GAIN,
            -AlignmentConstants.MAX_DRIVE_SPEED, AlignmentConstants.MAX_DRIVE_SPEED);
        double strafe = Range.clip(-yawError * AlignmentConstants.STRAFE_GAIN,
            -AlignmentConstants.MAX_STRAFE, AlignmentConstants.MAX_STRAFE);
        double turn   = Range.clip(headingError * AlignmentConstants.TURN_GAIN,
            -AlignmentConstants.MAX_TURN, AlignmentConstants.MAX_TURN);

        // Apply to mecanum drive
        setMecanumPowers(drive, strafe, turn);

        telemetry.addData("Tag", "Range:%.1f Bearing:%.1f Yaw:%.1f",
            targetTag.ftcPose.range, targetTag.ftcPose.bearing, targetTag.ftcPose.yaw);
    } else {
        // No tag visible — stop or use last known position
        setMecanumPowers(0, 0, 0);
        telemetry.addData("Tag", "Not visible");
    }
}

private void setMecanumPowers(double drive, double strafe, double turn) {
    double fl = drive + strafe + turn;
    double fr = drive - strafe - turn;
    double bl = drive - strafe + turn;
    double br = drive + strafe - turn;
    double max = Math.max(1.0, Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
                                         Math.max(Math.abs(bl), Math.abs(br))));
    frontLeft.setPower(fl / max);
    frontRight.setPower(fr / max);
    backLeft.setPower(bl / max);
    backRight.setPower(br / max);
}
\`\`\`

### 3. Field Localization from AprilTags (VisionPortal)

Convert AprilTag detections to robot field position using known tag positions:

\`\`\`java
/**
 * Compute robot's field position from an AprilTag detection.
 * Requires the tag's field position to be set in the tag library.
 *
 * This is a simplified 2D approach. For full 3D, use the raw pose matrix.
 */
public Pose2D getRobotFieldPose(AprilTagDetection detection, double robotHeading) {
    if (detection.metadata == null || detection.ftcPose == null) return null;

    // Camera-relative position of tag
    double tagRange   = detection.ftcPose.range;    // inches
    double tagBearing = Math.toRadians(detection.ftcPose.bearing);

    // Tag's known field position (from tag library metadata)
    // You need to set these when building your tag library
    double tagFieldX = detection.metadata.fieldPosition.get(0);  // inches
    double tagFieldY = detection.metadata.fieldPosition.get(1);  // inches

    // Robot heading (from IMU)
    double headingRad = Math.toRadians(robotHeading);

    // Camera-to-tag vector in field frame
    double dx = tagRange * Math.sin(headingRad + tagBearing);
    double dy = tagRange * Math.cos(headingRad + tagBearing);

    // Robot position = tag position - camera-to-tag vector
    // (also account for camera offset from robot center if significant)
    double robotX = tagFieldX - dx;
    double robotY = tagFieldY - dy;

    return new Pose2D(robotX, robotY, robotHeading);
}
\`\`\`

**Note:** For production localization, MegaTag2 on the Limelight is much more accurate and
easier to implement than manual localization math with VisionPortal.

### 4. Vision + Path Following (Pedro/Road Runner)

Combine vision detection during auto with path following:

\`\`\`java
// Pattern: detect during init, build path based on detection, execute path

// In init:
while (opModeInInit()) {
    // Vision processor runs, updates latestResult
}

// After start:
SampleColorProcessor.DetectionResult detection = colorProc.latestResult;

// Disable vision to free CPU for path following
portal.setProcessorEnabled(colorProc, false);

// Build path based on detection
PathChain scorePath;
if (detection.detected && detection.centerX < 320) {
    scorePath = buildLeftSidePath();
} else {
    scorePath = buildRightSidePath();
}

follower.followPath(scorePath);
while (opModeIsActive() && follower.isBusy()) {
    follower.update();
}
\`\`\`

### 5. Dashboard Camera Streaming

Stream the camera feed to FTC Dashboard for remote viewing and debugging:

\`\`\`java
import com.acmerobotics.dashboard.FtcDashboard;

// After building the portal:
FtcDashboard.getInstance().startCameraStream(portal, 0);
// The second parameter (0) = max FPS for streaming (0 = no limit, limited by Dashboard)

// To stop streaming:
FtcDashboard.getInstance().stopCameraStream();
\`\`\`

View the stream at \`http://192.168.43.1:8080/dash\` → Camera tab.

### 6. Latency Compensation

For fast-moving robots, account for the delay between frame capture and when you act on it:

\`\`\`java
// VisionPortal: capture time is available in processFrame
@Override
public Object processFrame(Mat frame, long captureTimeNanos) {
    // captureTimeNanos = when the frame was captured (System.nanoTime() scale)
    // Current time - captureTimeNanos = total pipeline latency
    long latencyMs = (System.nanoTime() - captureTimeNanos) / 1_000_000;
    // Use this to compensate: where was the robot when this frame was taken?
}

// Limelight: latency is reported directly
LLResult result = limelight.getLatestResult();
double totalLatency = result.getCaptureLatency() + result.getTargetingLatency();
double staleness = result.getStaleness();
// If staleness > 100ms, the data is likely too old to act on
\`\`\`

### 7. Graceful Vision Failure Handling

Always have a fallback when vision fails:

\`\`\`java
// Don't hang waiting for a detection — use a timeout
ElapsedTime visionTimer = new ElapsedTime();
ElementPosition position = ElementPosition.UNKNOWN;

while (opModeInInit()) {
    if (detector.latestResult.detected) {
        position = classifyPosition(detector.latestResult);
        visionTimer.reset();  // reset timeout since we got a detection
    }
    // Show detection state to driver
    telemetry.addData("Detection", position);
    telemetry.addData("Last seen", "%.1f sec ago", visionTimer.seconds());
    telemetry.update();
}

// If no detection in the last 2 seconds of init, use default
if (visionTimer.seconds() > 2.0) {
    position = ElementPosition.CENTER;  // safe default
    telemetry.addData("WARNING", "Using default position — vision timed out");
}
\`\`\`
`
};
