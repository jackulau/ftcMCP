export const GRADLE_KNOWLEDGE = {
  projectStructure: `
## FTC Gradle Project Structure

FTC projects use a multi-module Gradle build with a chain of \`apply from:\` files.
Understanding which file to edit is CRITICAL — editing the wrong file can break the SDK.

### File Hierarchy

\`\`\`
FtcRobotController/               (root project directory)
├── build.gradle                   # ROOT — DO NOT EDIT
├── build.common.gradle            # Shared build settings (compileSdk, Java version, etc.)
├── build.dependencies.gradle      # YOUR custom repos and dependencies — EDIT THIS
├── settings.gradle                # Declares project modules
├── gradle.properties              # JVM and Android settings
├── FtcRobotController/
│   └── build.gradle               # Robot Controller app module — DO NOT EDIT
└── TeamCode/
    └── build.gradle               # TeamCode module — usually no edits needed
\`\`\`

### build.gradle (Root) — DO NOT EDIT
\`\`\`groovy
// Top-level build file — DO NOT MODIFY
buildscript {
    repositories {
        google()
        mavenCentral()
    }
    dependencies {
        classpath 'com.android.tools.build:gradle:8.7.0'  // AGP 8.7.0
    }
}

// Global repositories for all modules
allprojects {
    repositories {
        google()
        mavenCentral()
    }
}
\`\`\`
- **AGP (Android Gradle Plugin)**: 8.7.0
- **DO NOT EDIT**: This file is managed by the SDK. Edits will be overwritten on SDK update.
- **Global repos**: \`google()\` and \`mavenCentral()\` are available to all modules by default.

### build.common.gradle — Shared Build Configuration
\`\`\`groovy
// Common build settings applied to all modules
android {
    compileSdk 30          // SDK version to compile against
    // NOTE: Change to 34 if using Pedro Pathing

    defaultConfig {
        minSdk 24          // Minimum Android version (Control Hub runs Android 7.0+)
        //noinspection ExpiredTargetSdkVersion
        targetSdk 28       // Target SDK version
    }

    compileOptions {
        sourceCompatibility JavaVersion.VERSION_1_8   // Java 8 — DO NOT CHANGE
        targetCompatibility JavaVersion.VERSION_1_8   // Java 8 — DO NOT CHANGE
    }

    signingConfigs {
        release {
            // Signing config for release builds
        }
    }

    ndkVersion '21.3.6528147'   // Native Development Kit version

    defaultConfig {
        ndk {
            abiFilters "armeabi-v7a", "arm64-v8a"  // ARM architectures only (no x86)
        }
    }
}
\`\`\`

**Key settings:**
- \`compileSdk 30\` — Default. Must be changed to **34** for Pedro Pathing.
- \`minSdk 24\` — Android 7.0 (Nougat). Matches REV Control Hub.
- \`targetSdk 28\` — Android 9.0 (Pie). The \`ExpiredTargetSdkVersion\` warning is expected; ignore it.
- \`JavaVersion.VERSION_1_8\` — Java 8. **DO NOT change this** even if you see JDK 21 warnings.
- \`NDK 21.3.6528147\` — Required for native libraries.
- \`abiFilters\` — Only ARM architectures. Control Hub is ARM-based.

### build.dependencies.gradle — YOUR CUSTOM DEPENDENCIES (EDIT THIS)
\`\`\`groovy
// This file is where teams add their custom Maven repositories and dependencies.
// It is applied via 'apply from:' in each module's build.gradle.

repositories {
    // Add custom Maven repositories here
    // Example:
    // maven { url = 'https://maven.brott.dev/' }
}

dependencies {
    // Add custom library dependencies here
    // Example:
    // implementation 'com.acmerobotics.dashboard:dashboard:0.5.1'
}
\`\`\`

**This is the PRIMARY file you edit** when adding third-party libraries.
You must add BOTH the repository URL AND the implementation line.

### settings.gradle — Module Declarations
\`\`\`groovy
pluginManagement {
    repositories {
        google()
        mavenCentral()
        gradlePluginPortal()
    }
}

include ':FtcRobotController'
include ':TeamCode'
\`\`\`
- Declares the two modules: \`:FtcRobotController\` (the app) and \`:TeamCode\` (your code).
- Rarely needs modification unless adding a new module.

### gradle.properties — Build Properties
\`\`\`properties
org.gradle.jvmargs=-Xmx1024M
android.useAndroidX=true
android.enableJetifier=true
\`\`\`
- \`-Xmx1024M\` — Max JVM heap for Gradle daemon. Increase to \`-Xmx2048M\` if builds run out of memory.
- \`android.useAndroidX=true\` — Uses AndroidX libraries (modern support libraries).
- \`android.enableJetifier=true\` — Converts old support library references to AndroidX automatically.

### TeamCode/build.gradle — TeamCode Module
\`\`\`groovy
// TeamCode module build file
apply from: '../build.common.gradle'        // Applies shared settings
apply from: '../build.dependencies.gradle'  // Applies custom repos + deps

android {
    namespace 'org.firstinspires.ftc.teamcode'

    packaging {
        jniLibs.useLegacyPackaging true
    }
}
\`\`\`
- **namespace**: \`org.firstinspires.ftc.teamcode\` — matches the package for all team code.
- \`apply from:\` — This is how the chain works. TeamCode applies both common settings and your dependencies.
- Usually **no edits needed** here.

### How \`apply from:\` Chains Files Together
\`\`\`
TeamCode/build.gradle
  ├── apply from: '../build.common.gradle'       → compileSdk, Java version, NDK, etc.
  └── apply from: '../build.dependencies.gradle'  → Your repos + dependencies
\`\`\`

When Gradle builds TeamCode, it:
1. Reads \`TeamCode/build.gradle\`
2. Applies \`build.common.gradle\` (like an include/import)
3. Applies \`build.dependencies.gradle\` (your custom stuff)
4. All three files together define the complete build configuration for TeamCode

This pattern lets the SDK maintain \`build.common.gradle\` while you safely edit
\`build.dependencies.gradle\` without merge conflicts on SDK updates.
`,

  addingLibraries: `
## Adding Third-Party Libraries — Step-by-Step Guide

### Step 1: Add the Maven Repository URL
Open \`build.dependencies.gradle\` and add the library's Maven repository in the \`repositories {}\` block:

\`\`\`groovy
repositories {
    maven { url = 'https://maven.brott.dev/' }   // FTC Dashboard, Road Runner
}
\`\`\`

**Important:** If the library is on Maven Central, you do NOT need to add a repository — Maven Central
is already included in the root \`build.gradle\` via \`mavenCentral()\`.

### Step 2: Add the Implementation Line
In the same \`build.dependencies.gradle\` file, add the dependency in the \`dependencies {}\` block:

\`\`\`groovy
dependencies {
    implementation 'com.acmerobotics.dashboard:dashboard:0.5.1'
}
\`\`\`

### Step 3: Gradle Sync
After saving changes, click **"Sync Now"** in the Android Studio notification bar, or go to
**File → Sync Project with Gradle Files** (Ctrl+Shift+O on Windows/Linux, Cmd+Shift+O on Mac).

### Step 4: Verify
- Check the **Build** output for errors.
- If successful, you can now \`import\` the library classes in your Java code.

### Dependency Configurations

| Configuration | Description |
|---|---|
| \`implementation\` | Standard dependency. Available to your module only. **Use this for most libraries.** |
| \`api\` | Like implementation, but also exposes the dependency to modules that depend on yours. Rarely needed in FTC. |
| \`compileOnly\` | Available at compile time only, NOT included in the APK. Use for annotation processors or compile-time-only checks. |

For FTC, you will almost always use \`implementation\`.

### Excluding Transitive Dependencies
Some libraries bundle FTC SDK classes that conflict with the SDK already in your project.
Use \`exclude\` to remove them:

\`\`\`groovy
implementation('some.library:name:1.0.0') {
    exclude group: 'org.firstinspires.ftc'
}
\`\`\`

This tells Gradle: "Use this library, but don't pull in its copy of the FTC SDK — I already have it."

### Critical Rule: MUST Add BOTH
A common mistake is adding only the \`implementation\` line without the \`repositories\` URL, or vice versa.
You need BOTH:

\`\`\`groovy
repositories {
    maven { url = 'https://example.com/maven' }  // WHERE to find it
}

dependencies {
    implementation 'com.example:library:1.0.0'    // WHAT to download
}
\`\`\`

Without the repository, Gradle can't find the artifact.
Without the implementation, Gradle doesn't know to download it.
`,

  allLibraryCoords: `
## Complete Library Reference — Exact Maven Coordinates

### Pedro Pathing (Path Following Library)
**Repository:**
\`\`\`groovy
maven { url = "https://mymaven.bylazar.com/releases" }
\`\`\`
**Dependencies:**
\`\`\`groovy
implementation 'com.pedropathing:ftc:2.0.6'
implementation 'com.pedropathing:telemetry:1.0.0'
implementation 'com.bylazar:fullpanels:1.0.9'
\`\`\`
**Note:** Pedro Pathing requires \`compileSdk 34\` in \`build.common.gradle\`.

---

### FTC Dashboard (Real-Time Tuning & Telemetry)
**Repository:**
\`\`\`groovy
maven { url = 'https://maven.brott.dev/' }
\`\`\`
**Dependencies:**
\`\`\`groovy
implementation 'com.acmerobotics.dashboard:dashboard:0.5.1'
\`\`\`
**Note:** Not allowed during competition matches (RS09). Only for practice/tuning.

---

### Road Runner (Motion Planning & Path Following)
**Repository:**
\`\`\`groovy
maven { url = 'https://maven.brott.dev/' }
\`\`\`
**Dependencies:**
\`\`\`groovy
implementation 'com.acmerobotics.roadrunner:core:1.0.1'
implementation 'com.acmerobotics.roadrunner:actions:1.0.1'
implementation 'com.acmerobotics.roadrunner:ftc:0.1.25'
\`\`\`
**Note:** Shares the same Maven repo as FTC Dashboard (maven.brott.dev).

---

### CachingHardware (Dairy Foundation — Hardware Caching)
**Repository:**
\`\`\`groovy
maven { url = 'https://repo.dairy.foundation/releases' }
\`\`\`
**Dependencies:**
\`\`\`groovy
implementation 'dev.frozenmilk.dairy:CachingHardware:1.0.0'
\`\`\`

---

### SolversLib (Command-Based Framework — Recommended)
**Repository:**
\`\`\`groovy
maven { url = "https://repo.dairy.foundation/releases" }
\`\`\`
**Dependencies:**
\`\`\`groovy
implementation "org.solverslib:core:0.3.4"
\`\`\`
**Note:** SolversLib is the actively maintained fork of FTCLib (by Seattle Solvers).
**CRITICAL:** SolversLib and FTCLib CANNOT coexist in the same project. Pick one.
Shares the same Dairy Foundation Maven repo as CachingHardware.

---

### FTCLib (Legacy — Use SolversLib Instead)
**Repository:** Maven Central (already included by default — no extra repo needed)
**Dependencies:**
\`\`\`groovy
implementation 'org.ftclib.ftclib:core:2.1.1'
\`\`\`
**Note:** FTCLib is no longer actively maintained. SolversLib is recommended.

---

### Complete Example: build.dependencies.gradle with ALL Libraries
\`\`\`groovy
repositories {
    // Pedro Pathing
    maven { url = "https://mymaven.bylazar.com/releases" }

    // FTC Dashboard + Road Runner
    maven { url = 'https://maven.brott.dev/' }

    // Dairy Foundation (CachingHardware + SolversLib)
    maven { url = 'https://repo.dairy.foundation/releases' }

    // FTCLib (legacy) — uses Maven Central, already in root build.gradle (no extra repo needed)
}

dependencies {
    // Pedro Pathing
    implementation 'com.pedropathing:ftc:2.0.6'
    implementation 'com.pedropathing:telemetry:1.0.0'
    implementation 'com.bylazar:fullpanels:1.0.9'

    // FTC Dashboard
    implementation 'com.acmerobotics.dashboard:dashboard:0.5.1'

    // Road Runner
    implementation 'com.acmerobotics.roadrunner:core:1.0.1'
    implementation 'com.acmerobotics.roadrunner:actions:1.0.1'
    implementation 'com.acmerobotics.roadrunner:ftc:0.1.25'

    // CachingHardware
    implementation 'dev.frozenmilk.dairy:CachingHardware:1.0.0'

    // SolversLib (command-based framework — recommended over FTCLib)
    implementation 'org.solverslib:core:0.3.4'

    // FTCLib (legacy — use SolversLib instead, they CANNOT coexist)
    // implementation 'org.ftclib.ftclib:core:2.1.1'
}
\`\`\`

**REMEMBER:** If using Pedro Pathing, you MUST also change \`compileSdk\` from 30 to **34** in
\`build.common.gradle\`. This is a separate file change — not in \`build.dependencies.gradle\`.
`,

  commonIssues: `
## Common Gradle Issues and Troubleshooting

### 1. Pedro Pathing Requires compileSdk 34
**Symptom:**
\`\`\`
Dependency requires compileSdk 34, but this module is compiled with compileSdk 30
\`\`\`

**Fix:** Change \`compileSdk\` in \`build.common.gradle\` (NOT in Android Studio's Project Structure dialog):
\`\`\`groovy
android {
    compileSdk 34    // Changed from 30 to 34
    // ... rest unchanged
}
\`\`\`

**IMPORTANT:** Do NOT use the Android Studio **Project Structure** UI (File → Project Structure) to change
compileSdk. It will modify the wrong file (\`TeamCode/build.gradle\` directly) and your change will conflict
with the \`apply from: '../build.common.gradle'\` chain. Always edit \`build.common.gradle\` manually.

### 2. JDK 21 Deprecation Warning — HARMLESS
**Symptom:**
\`\`\`
'compileJava' task (current target is 1.8) and target compatibility is set to 1.8
Using a Java version (21) that does not support source/target 1.8.
\`\`\`

**Fix:** This warning is **harmless**. DO NOT change the Java version from 1.8 in \`build.common.gradle\`.
\`\`\`groovy
compileOptions {
    sourceCompatibility JavaVersion.VERSION_1_8   // KEEP THIS — DO NOT CHANGE
    targetCompatibility JavaVersion.VERSION_1_8   // KEEP THIS — DO NOT CHANGE
}
\`\`\`

The FTC SDK requires Java 8 source/target compatibility for the Control Hub's Android runtime.
Your development machine can use JDK 21 — the warning just means JDK 21 is being used to compile
Java 8 compatible code, which works fine.

### 3. "Could not resolve" — Missing Repository URL
**Symptom:**
\`\`\`
Could not resolve com.pedropathing:ftc:2.0.6.
Required by: project :TeamCode
\`\`\`

**Fix:** You added the \`implementation\` line but forgot the Maven repository URL.
Add the correct repo in \`build.dependencies.gradle\`:
\`\`\`groovy
repositories {
    maven { url = "https://mymaven.bylazar.com/releases" }  // ADD THIS
}
\`\`\`

Then Gradle Sync again.

### 4. "Failed to find target with hash string 'android-34'"
**Symptom:**
\`\`\`
Failed to find target with hash string 'android-34'
\`\`\`

**Fix:** Install SDK Platform 34 via Android Studio's SDK Manager:
1. Open Android Studio
2. Go to **Tools → SDK Manager** (or **File → Settings → Appearance & Behavior → System Settings → Android SDK**)
3. In the **SDK Platforms** tab, check **Android 14.0 (API 34)**
4. Click **Apply** / **OK** to download and install
5. Gradle Sync again

### 5. Duplicate Class Errors
**Symptom:**
\`\`\`
Duplicate class com.qualcomm.robotcore.hardware.DcMotor found in modules :FtcRobotController and :some-library
\`\`\`

**Fix:** The library is bundling FTC SDK classes that conflict with the SDK already in your project.
Exclude the FTC group from that dependency:
\`\`\`groovy
implementation('com.example:library:1.0.0') {
    exclude group: 'org.firstinspires.ftc'
}
\`\`\`

### 6. Multidex Error
**Symptom:**
\`\`\`
Cannot fit requested classes in a single dex file
\`\`\`
or
\`\`\`
The number of method references in a .dex file cannot exceed 64K
\`\`\`

**Fix:** Enable multidex in \`build.common.gradle\` (or \`TeamCode/build.gradle\`):
\`\`\`groovy
android {
    defaultConfig {
        multiDexEnabled true
    }
}
\`\`\`

This happens when the total method count across all dependencies exceeds the DEX 64K method limit.
Multidex splits the code across multiple DEX files.

### 7. General Troubleshooting Steps
1. **Gradle Sync** — Always sync after any build file change.
2. **Invalidate Caches** — File → Invalidate Caches and Restart.
3. **Clean Build** — Build → Clean Project, then Build → Rebuild Project.
4. **Check build.dependencies.gradle** — Ensure both repo URL and implementation line are present.
5. **Check build.common.gradle** — Ensure compileSdk matches library requirements.
6. **Check internet connection** — Gradle downloads dependencies from the internet.
7. **Delete .gradle and build folders** — Nuclear option: delete \`~/.gradle/caches\` and all \`build/\` directories, then sync.
`,

  buildProcess: `
## How the FTC Build Process Works

### Build Pipeline Overview
\`\`\`
Java Source (.java)
    │
    ▼
  javac (Java Compiler)
    │
    ▼
Bytecode (.class files)
    │
    ▼
  D8 (DEX Compiler)
    │
    ▼
DEX files (.dex)
    │
    ▼
  AAPT2 (Android Asset Packaging Tool)
    │  Combines: DEX + resources + manifest + native libs
    ▼
Unsigned APK
    │
    ▼
  APK Signing (debug or release keystore)
    │
    ▼
Signed APK (.apk)
    │
    ▼
  ADB (Android Debug Bridge)
    │  Deploys to Control Hub / Phone via USB or Wi-Fi
    ▼
Running on Robot Controller
\`\`\`

### Stage Details

#### 1. Java Compilation (javac)
- Compiles \`.java\` source files to \`.class\` bytecode files.
- Source compatibility: Java 8 (\`JavaVersion.VERSION_1_8\`).
- Processes annotations during compilation (\`@TeleOp\`, \`@Autonomous\`, \`@Config\`, etc.).
- Output: \`.class\` files in \`build/intermediates/javac/\`.

#### 2. DEX Compilation (D8)
- D8 is Google's DEX compiler (replaced the older DX compiler).
- Converts Java \`.class\` bytecode to Dalvik Executable (\`.dex\`) format.
- DEX is the executable format for Android's ART (Android Runtime).
- If multidex is enabled, produces multiple \`.dex\` files (\`classes.dex\`, \`classes2.dex\`, etc.).
- Performs desugaring: converts Java 8 features (lambdas, streams, etc.) to Java 7 compatible bytecode.

#### 3. Resource Packaging (AAPT2)
- AAPT2 (Android Asset Packaging Tool 2) packages non-code resources.
- Combines: compiled DEX, Android resources (layouts, drawables), AndroidManifest.xml, native \`.so\` libraries.
- ABI filters (\`armeabi-v7a\`, \`arm64-v8a\`) determine which native libraries are included.
- Output: unsigned APK.

#### 4. APK Signing
- Signs the APK with a keystore (debug keystore for development).
- Required by Android — unsigned APKs cannot be installed.
- The signing config is defined in \`build.common.gradle\`.

#### 5. Deployment (ADB)
- ADB pushes the signed APK to the Control Hub or Robot Controller phone.
- Connection: USB cable or Wi-Fi (Control Hub creates its own Wi-Fi network).
- Android Studio handles this automatically when you click Run.

### OpMode Discovery — How the Robot Finds Your OpModes

The FTC SDK uses **annotation scanning of DEX files** at runtime to discover OpModes:

1. When the Robot Controller app starts, it scans all DEX files in the APK.
2. It looks for classes annotated with \`@TeleOp\` or \`@Autonomous\`.
3. Found OpModes are registered and appear in the Driver Station dropdown.
4. Classes annotated with \`@Disabled\` are found but excluded from the list.

\`\`\`java
// This class will be discovered and listed as "My TeleOp" in the TeleOp dropdown
@TeleOp(name = "My TeleOp", group = "Competition")
public class MyTeleOp extends OpMode { ... }

// This class will be discovered but NOT listed (disabled)
@TeleOp(name = "Old TeleOp", group = "Deprecated")
@Disabled
public class OldTeleOp extends OpMode { ... }
\`\`\`

**Key implication:** You do NOT need to register OpModes anywhere. Just annotate them and they appear automatically.

### @Config Discovery — FTC Dashboard

FTC Dashboard uses a similar DEX scanning process:

1. At runtime, Dashboard scans DEX files for classes annotated with \`@Config\`.
2. It reads all \`public static\` fields from those classes.
3. These fields become tunable variables on the Dashboard web interface.
4. Changes made on the Dashboard modify the static fields in real-time via reflection.

\`\`\`java
@Config  // Dashboard will discover this class and expose its static fields
public class DriveConstants {
    public static double MAX_SPEED = 0.8;        // Tunable on Dashboard
    public static double TURN_GAIN = 0.5;        // Tunable on Dashboard
    public static int ENCODER_TARGET = 1000;     // Tunable on Dashboard
    private static double hidden = 1.0;           // NOT exposed (private)
    public double notStatic = 1.0;                // NOT exposed (not static)
}
\`\`\`

**Key implication:** To make a value live-tunable:
1. Add \`@Config\` to the class.
2. Make the field \`public static\` (not final).
3. Read the static field directly each loop iteration (don't cache it in a local variable).
`,

  buildAndDeploy: `
## Building and Deploying to the Robot

The FTC SDK includes a Gradle wrapper (\`gradlew\` / \`gradlew.bat\`) that handles the entire
build process. You do NOT need Android Studio installed to build and deploy — a terminal with
JDK 17 and the Android SDK is sufficient.

### Quick Reference

| Command | What it does |
|---|---|
| \`./gradlew assembleDebug\` | Compile code and build the APK |
| \`./gradlew installDebug\` | Build + install APK to connected device |
| \`./gradlew clean assembleDebug\` | Clean everything and rebuild from scratch |
| \`./gradlew --stop\` | Stop the Gradle daemon (fixes stale build issues) |
| \`./gradlew --refresh-dependencies\` | Force re-download all dependencies |
| \`./gradlew tasks\` | List all available Gradle tasks |

**Windows users:** Use \`gradlew.bat\` instead of \`./gradlew\`.

---

### Complete Build & Deploy Workflow

#### Step 1: Clone the FTC Robot Controller
\`\`\`bash
git clone https://github.com/FIRST-Tech-Challenge/FtcRobotController.git
cd FtcRobotController
\`\`\`

Or if your team has a fork:
\`\`\`bash
git clone https://github.com/<your-team>/FtcRobotController.git
cd FtcRobotController
\`\`\`

#### Step 2: Make the Gradle Wrapper Executable (macOS/Linux)
\`\`\`bash
chmod +x gradlew
\`\`\`

#### Step 3: Build the APK
\`\`\`bash
./gradlew assembleDebug
\`\`\`

This compiles all Java source code, runs D8 (DEX compiler), packages resources, and produces
a signed debug APK. First build takes 1-3 minutes; subsequent builds are incremental and much faster.

**Build output location:**
\`\`\`
FtcRobotController/build/outputs/apk/debug/FtcRobotController-debug.apk
\`\`\`

**Note:** You do NOT need to manually find or copy the APK. The \`installDebug\` task handles deployment.

#### Step 4: Connect to the Robot

**Option A: USB Cable (most reliable)**
1. Connect a USB cable from your computer to the Control Hub's USB-C port (or the Robot Controller phone's USB port)
2. Verify ADB sees the device:
\`\`\`bash
adb devices
# Should show:
# List of devices attached
# <serial_number>    device
\`\`\`

**Option B: Wireless ADB via Control Hub WiFi**
1. Connect your computer to the Control Hub's WiFi network (the robot's WiFi hotspot)
   - SSID is typically: \`FTC-<team_number>\` or the name configured in the Control Hub
2. Connect ADB wirelessly:
\`\`\`bash
adb connect 192.168.43.1:5555
\`\`\`
3. Verify connection:
\`\`\`bash
adb devices
# Should show:
# List of devices attached
# 192.168.43.1:5555    device
\`\`\`

**Option C: Wireless ADB via Robot Controller Phone**
If using a phone as the Robot Controller (with an Expansion Hub):
\`\`\`bash
adb connect 192.168.49.1:5555
\`\`\`

**Option D: REV Hardware Client (Windows)**
The REV Hardware Client app can automatically establish ADB connections to the Control Hub.
Install from https://docs.revrobotics.com/rev-hardware-client. Once it detects the hub,
ADB is auto-connected.

#### Step 5: Deploy to the Robot
\`\`\`bash
./gradlew installDebug
\`\`\`

This builds the APK (if needed) and installs it onto the connected device via ADB.
The Robot Controller app on the Control Hub/phone will restart with your updated code.

**The OpModes you wrote will appear automatically** in the Driver Station dropdown —
no additional registration is needed. Just annotate with \`@TeleOp\` or \`@Autonomous\`.

#### Step 6: Iterate
The typical development loop is:
1. Edit Java code in your IDE/editor
2. Run \`./gradlew installDebug\` (builds + deploys in one step)
3. Test on the robot
4. Repeat

For faster iteration, keep the Gradle daemon running (it stays alive by default) — subsequent
builds are incremental and typically complete in 5-15 seconds.

---

### ADB Commands for FTC

#### Connection
\`\`\`bash
# Connect to Control Hub wirelessly
adb connect 192.168.43.1:5555

# Connect to Robot Controller phone wirelessly
adb connect 192.168.49.1:5555

# Disconnect
adb disconnect 192.168.43.1:5555

# List connected devices
adb devices
\`\`\`

#### Deployment
\`\`\`bash
# Install APK directly (alternative to gradlew installDebug)
adb install -r FtcRobotController/build/outputs/apk/debug/FtcRobotController-debug.apk

# Uninstall the Robot Controller app (nuclear option for weird bugs)
adb uninstall com.qualcomm.ftcrobotcontroller
\`\`\`

#### Debugging with Logcat
\`\`\`bash
# Stream ALL logs (very verbose)
adb logcat

# Filter for FTC-related logs (RobotCore tag)
adb logcat -s RobotCore:*

# Filter for your team's logs (use Log.d("MyTag", "message") in your code)
adb logcat -s MyTag:*

# Filter for errors and warnings only
adb logcat *:W

# Clear the log buffer and start fresh
adb logcat -c && adb logcat

# Save logcat output to a file
adb logcat > robot_log.txt
\`\`\`

**Using Log in your Java code:**
\`\`\`java
import android.util.Log;

// In your OpMode or subsystem:
Log.d("MyAuto", "Current state: " + currentState);
Log.e("MyAuto", "Error: motor not responding");
Log.i("MyAuto", "Path completed in " + timer.seconds() + "s");
\`\`\`

Then filter with: \`adb logcat -s MyAuto:*\`

#### Restart and Management
\`\`\`bash
# Restart the Robot Controller app (without reinstalling)
adb shell am force-stop com.qualcomm.ftcrobotcontroller
adb shell am start -n com.qualcomm.ftcrobotcontroller/.FtcRobotControllerActivity

# Reboot the Control Hub
adb reboot

# Get Control Hub shell access
adb shell
\`\`\`

---

### Troubleshooting Build & Deploy Issues

#### "adb: command not found"
ADB is not in your PATH. Add the Android SDK \`platform-tools\` directory to your PATH:
\`\`\`bash
export PATH="$ANDROID_HOME/platform-tools:$PATH"
\`\`\`

#### "error: no devices/emulators found"
No device is connected. Check:
1. USB cable is connected and the device shows up in \`adb devices\`
2. For wireless: ensure you're on the robot's WiFi network and ran \`adb connect\`
3. Try \`adb kill-server && adb start-server\` to restart the ADB server

#### "INSTALL_FAILED_UPDATE_INCOMPATIBLE"
The installed APK was signed with a different key. Uninstall first:
\`\`\`bash
adb uninstall com.qualcomm.ftcrobotcontroller
./gradlew installDebug
\`\`\`

#### Build fails with "SDK location not found"
Set the ANDROID_HOME environment variable or create a \`local.properties\` file in the project root:
\`\`\`properties
sdk.dir=/path/to/Android/sdk
\`\`\`

For example:
\`\`\`properties
# macOS
sdk.dir=/Users/<username>/Library/Android/sdk

# Linux
sdk.dir=/home/<username>/Android/Sdk

# Windows (use forward slashes or double backslashes)
sdk.dir=C:/Users/<username>/AppData/Local/Android/Sdk
\`\`\`

#### Build fails with "Failed to find target with hash string 'android-34'"
Install the Android SDK Platform 34. From the command line:
\`\`\`bash
$ANDROID_HOME/cmdline-tools/latest/bin/sdkmanager "platforms;android-34"
\`\`\`
Or install it via Android Studio's SDK Manager (Tools -> SDK Manager -> SDK Platforms -> Android 14.0).

#### Gradle build is extremely slow
1. Increase memory: edit \`gradle.properties\` and set \`org.gradle.jvmargs=-Xmx2048M\`
2. Enable parallel builds: add \`org.gradle.parallel=true\` to \`gradle.properties\`
3. Kill stale daemon: \`./gradlew --stop\` then rebuild
4. Clean build if incremental is broken: \`./gradlew clean assembleDebug\`

#### Wireless ADB keeps disconnecting
This is common. The Control Hub WiFi can be flaky. Tips:
- USB deployment is always more reliable for competition
- After reconnecting WiFi, re-run \`adb connect 192.168.43.1:5555\`
- REV Hardware Client (Windows) can auto-reconnect
- Some teams deploy via USB and only use WiFi for FTC Dashboard

---

### One-Command Deploy Scripts

For convenience, create a deploy script in your project root:

**deploy.sh (macOS/Linux):**
\`\`\`bash
#!/bin/bash
echo "Building and deploying to robot..."
./gradlew installDebug
if [ $? -eq 0 ]; then
    echo "Deploy successful!"
else
    echo "Deploy failed. Check errors above."
    exit 1
fi
\`\`\`

**deploy.bat (Windows):**
\`\`\`batch
@echo off
echo Building and deploying to robot...
gradlew.bat installDebug
if %ERRORLEVEL% EQU 0 (
    echo Deploy successful!
) else (
    echo Deploy failed. Check errors above.
    exit /b 1
)
\`\`\`

Make it executable: \`chmod +x deploy.sh\`

---

### VS Code Task Integration

Add a \`.vscode/tasks.json\` to run Gradle from the VS Code Command Palette (\`Ctrl+Shift+B\`):

\`\`\`json
{
    "version": "2.0.0",
    "tasks": [
        {
            "label": "FTC: Build",
            "type": "shell",
            "command": "./gradlew assembleDebug",
            "group": "build",
            "problemMatcher": ["$javac"]
        },
        {
            "label": "FTC: Deploy",
            "type": "shell",
            "command": "./gradlew installDebug",
            "group": {
                "kind": "build",
                "isDefault": true
            },
            "problemMatcher": ["$javac"]
        },
        {
            "label": "FTC: Clean Build",
            "type": "shell",
            "command": "./gradlew clean assembleDebug",
            "group": "build",
            "problemMatcher": ["$javac"]
        },
        {
            "label": "FTC: Connect ADB (Control Hub WiFi)",
            "type": "shell",
            "command": "adb connect 192.168.43.1:5555",
            "group": "none",
            "problemMatcher": []
        }
    ]
}
\`\`\`

With this, press \`Ctrl+Shift+B\` (or \`Cmd+Shift+B\` on Mac) to build and deploy in one keystroke.
`
};
