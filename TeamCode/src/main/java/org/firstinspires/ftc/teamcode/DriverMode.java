package org.firstinspires.ftc.teamcode;

//import com.acmerobotics.roadrunner.PoseVelocity2d;
//import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.util.List;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.hardwareSystems.LimiterServo;

import org.firstinspires.ftc.teamcode.AutoSettings;
import static org.firstinspires.ftc.teamcode.AutoSettings.AllianceColor;
import static org.firstinspires.ftc.teamcode.AutoSettings.TeamSide;

import org.firstinspires.ftc.teamcode.hardwareSystems.Webcam; // for Color

import com.acmerobotics.dashboard.FtcDashboard;

@TeleOp(name = "DriverMode")
public class DriverMode extends CustomLinearOp {
    // TODO: Replace the driving sensitivity with an appropriate level of sensitivity.
    /**
     * The sensitivity of the robot's driving joystick.
     */
    private static final double DRIVING_SENSITIVITY = 1.0;

    /**
     * Minimum joystick magnitude required to register movement.  Inputs
     * below this threshold will be treated as zero.  This helps prevent
     * unintended robot motion when the driver releases the sticks and
     * eliminates small negative values (e.g. -0.29) shown in telemetry
     * caused by joystick drift.
     */
    private static final double DEADBAND = 0.07;

    /**
     * Apply a deadband to the given value.  If the absolute value is
     * less than {@link #DEADBAND}, return zero; otherwise return the
     * original value.
     *
     * @param value The raw joystick value.
     * @return Zero if the value is within the deadband; otherwise the
     *         unchanged input value.
     */
    private double applyDeadband(double value) {
        return Math.abs(value) < DEADBAND ? 0.0 : value;
    }

    /**
     * Deadband for trigger and stick drift compensation.  Values within this
     * threshold after offset subtraction are treated as zero.  Increase this
     * value if your gamepad has larger drift (e.g. 0.3).
     */
    private static final double OFFSET_DEADBAND = 0.07;

    /**
     * Measured resting offsets for the driver controls.  These values are
     * sampled during the init phase (before the match begins) while the
     * driver holds all sticks and triggers at their neutral positions.  By
     * subtracting these offsets from the raw inputs each loop, we ensure
     * that small bias or drift does not cause the robot to creep when
     * released.
     */
    private double verticalOffset = 0.0;
    private double horizontalOffset = 0.0;
    private double pivotOffset = 0.0;

    /**
     * Additional subsystems for driver control.  The lazy Susan motor
     * rotates the camera and launcher platform; the launcher motor spins
     * the flywheel to shoot game pieces.  Both motors are initialised in
     * runOpMode() using names defined in the hardware configuration.
     */
    private DcMotorEx lazySusanMotor;
    private DcMotorEx launcherMotor;

    // ----- Limiter servo (shared between TeleOp and Auto) -----
    /**
     * Wrapper around the limiter servo hardware.
     * Uses two preset positions: HOME and OUT (≈90 degrees).
     */
    private LimiterServo limiterServo;

    // Positions for the limiter servo.
    // You will tune these on the real robot so that OUT is about 90 degrees
    // and HOME is the original position.
    private static final double LIMITER_HOME_POS  = 0.25; // 25 degrees
    private static final double LIMITER_OUT_POS   = 0.10; // 10 degrees

    /**
     * Servo to control launch angle of balls
     */
    private LimiterServo hoodServo;

    private static final double HOOD_HOME_POS = 0; // 0 degrees
    private static final double HOOD_OUT_POS = 0.1; // 1 degrees

    private boolean cameraStreamEnabled = true;  // set false if you want it off by default
    private boolean lastToggleBtn = false;

    /**
     * Current shooter power.  Drivers can switch between two preset
     * powers (e.g. mid‑range and long‑range) using D‑pad buttons on
     * gamepad 2.  When the launcher is enabled via the X button, this
     * power is applied to the launcher motor; otherwise the power is zero.
     */
    private double shooterPower = 1.0; // default mid‑range
    private final double SHOOTER_POWER_LOW = 0.7;
    private final double SHOOTER_POWER_HIGH = 1.0;
    private boolean launcherEnabled = false;

    /**
     * Flag indicating whether auto‑aim is engaged.  When true, the
     * driver’s strafe and turn inputs are overridden by values computed
     * from AprilTag bearing and yaw to align the robot with the target.
     */
    private boolean autoAimEnabled = false;

    private static int cameraMonitorViewId;

    /**
     * The intake motor for the robot.  This motor powers the roller/wheel
     * mechanism that pulls game pieces into the robot.  It is assumed to
     * be a REV Core Hex motor (72:1 gearbox, 288 encoder counts per output
     * revolution).  This field is initialised in {@link #runOpMode()}
     * using the hardware name "intakeMotor".  If your intake motor uses
     * a different name in the Robot Controller configuration, update
     * the call to {@code hardwareMap.get()} accordingly.
     */
    private DcMotorEx intakeMotor;

    // --- Lazy Susan instant control tuning ---
// Calibrate this once: ticksPerDeg = (tick change) / (measured degrees)
    private static final double LAZY_TICKS_PER_DEG = 10.00; // TODO: tune on-bot
    private static final double LAZY_MAX_DEG       = 180.0; // hard range from start
    private static final double LAZY_SOFT_ZONE_DEG = 30.0;  // start easing in last 30°
    private static final double LAZY_DEADBAND      = 0.07;  // ignore tiny stick wiggle
    private static final double LAZY_FILTER_ALPHA  = 0.6;   // 0..1, higher = smoother
    private static final double LAZY_GAIN          = 0.9;   // scales stick -> motor power
    private static final double LAZY_POWER_MAX     = 0.8;   // safety cap on power

    private int    lazyZeroTicks = 0;   // encoder value at start
    private double lazyTargetDeg = 0.0; // still used for telemetry/limits
    private double lazyStickFilt = 0.0; // smoothed stick value

    // --- Launcher RPM tracking ---
    // Guess for encoder ticks per revolution on the launcher shaft.
    // 28 is a common motor encoder CPR; 40 is your gear ratio.
    // You can tune this later if numbers look off.
    private static final double LAUNCHER_TICKS_PER_REV = 20.6 ;

    // Last computed RPM for telemetry
    private double lastLauncherRpm = 0.0;


    // --- Aimbot config ---
// Replace with your real IDs for this season
    private static final int[] RED_TAG_IDS  = { 24 };
    private static final int[] BLUE_TAG_IDS = { 20 };

    // --- Pixel-based turret aiming
    // motor power at full-screen error (tune 0.4 - 1.2)
    private static final double AIM_PIX_KP = 1.2;
    // within this many pixels -> stop motor (reduces shake)
    private static final double AIM_LOCK_DEADBAND_PX = 18.0;
    // must exceed this to "start moving" again (hysteresis)
    private static final double AIM_MOVE_DEADBAND_PX = 35.0;
    // 0..1, higher = more responsive, lower = smoother
    private static final double AIM_ERR_FILTER_ALPHA = 0.20;

    // tune 0.15–0.35
    private static final double AIM_REACQUIRE_POWER = 0.22;
    // give it time before scanning
    private static final int AIM_LOST_FRAMES_THRESHOLD = 18;

    // Proportional yaw -> Lazy Susan power (same as before)
    private static final double AIM_YAW_KP       = 0.02; // tune 0.015–0.03
    private static final double AIM_POWER_MAX    = 0.80;
    private static final double AIM_DEADBAND_DEG = 1.0;

    // Distance-based shooter power:
    // We want full power at about 12 ft (≈ 3.7 m),
    // and a bit less power as we get closer to the goal.
// --- Shooter power calibration (tune these on the real robot) ---
//
// AprilTag "range" is measured from the CAMERA.
// Your ball leaves from the SHOOTER, which is usually offset.
// These two numbers correct that so the distance better matches reality.
    private static final double AIM_RANGE_SCALE  = 1.00;   // usually 1.00 (only change if tag range is systematically scaled wrong)
    private static final double AIM_RANGE_OFFSET_M = -0.15; // meters: camera->shooter offset along the shooting direction
// Example: if camera is 15 cm BEHIND the shooter exit, offset should be negative (~ -0.15).
// If camera is 15 cm IN FRONT of shooter exit, offset should be positive (+0.15).


    // A piecewise calibration curve: distance (meters) -> shooter power
// Add points as you test. More points = more accurate.
    private static final double[] AIM_CAL_DIST_M = {
            0.70,  // very close
            1.50,  // mid
            2.50,  // farther
            3.70   // far
    };


    private static final double[] AIM_CAL_PWR = {
            0.80,  // power at 0.70 m
            0.88,  // power at 1.50 m
            0.95,  // power at 2.50 m
            1.00   // power at 3.70 m
    };

    // When aimbot is ON but no tag is visible, we will "scan" left/right
    // with the Lazy Susan to look for the target.
    private static final double AIM_SCAN_POWER        = 0.25; // how fast to scan
    private static final double AIM_SEARCH_LIMIT_DEG  = 180; // max search angle left/right

    // Extra smoothing / hysteresis for turret control.
    //  - AIM_LOCK_DEADBAND_DEG: inside this we treat it as "locked" and set motor power = 0.
    //  - AIM_MOVE_DEADBAND_DEG: if error is bigger than this, we allow movement again.
    private static final double AIM_LOCK_DEADBAND_DEG = 1.0;  // tight center band
    private static final double AIM_MOVE_DEADBAND_DEG = 3.0;  // start moving if error bigger

    // Low-pass filter for noisy yaw readings.
    // NewYawFiltered = α * NewRaw + (1-α) * OldFiltered
    private static final double AIM_YAW_FILTER_ALPHA = 0.3;

    // ---- Aimbot runtime state ----
    private boolean aimHasLock = false;
    private int aimFramesWithTag = 0;
    private int aimFramesWithoutTag = 0;
    private double aimFilteredYawDeg = 0.0;

    // Shooter info that aimbot computes (distance -> power).
    // We will use this in the normal launcher block instead of
    // spinning the motor directly from updateAimbot().
    private boolean aimHasTarget = false;
    private double aimLastShooterPower = 1.0;

    // --- Pixel error filtering state ---
    private double aimFilteredErrPx = 0.0;

    private double aimLastErrorPx = 0.0;
    private double aimLastTurretCmd = 0.0;
    private int aimLockedTagId = -1;

    // A/B edge detect (A=enable, B=disable)
    private boolean prevG2_A = false;
    private boolean prevG2_B = false;
    private boolean allianceIsRed = true; // we will sync this with AutoSettings

    // Aimbot scan state:
    // +1  = scanning to the right
    // -1  = scanning to the left
    private int aimScanDirection = 1;

    // Pick “best” detection: prefer alliance IDs
    private AprilTagDetection pickBestDetection(List<AprilTagDetection> dets) {
        if (dets == null || dets.isEmpty()) return null;

        int[] ids = allianceIsRed ? RED_TAG_IDS : BLUE_TAG_IDS;

        // Need frame width for pixel scoring; fall back safely if webcam is null.
        double frameW = (WEBCAM != null) ? WEBCAM.getWidthPx() : 800.0;
        double cx = frameW / 2.0;

        AprilTagDetection best = null;
        double bestScore = Double.NEGATIVE_INFINITY;

        for (AprilTagDetection d : dets) {
            if (d == null || d.ftcPose == null || d.center == null) continue;

            boolean idMatch = false;
            for (int id : ids) {
                if (d.id == id) { idMatch = true; break; }
            }
            if (!idMatch) continue;

            // Prefer detections closest to camera center, then closer range.
            double errPx = Math.abs(d.center.x - cx);
            double rangeM = d.ftcPose.range * 0.0254; // inches -> meters

            // Higher score is better: small pixel error dominates.
            double score = -errPx - (25.0 * rangeM);

            if (score > bestScore) {
                bestScore = score;
                best = d;
            }
        }

        if (best != null) return best;

        // Fallback: choose smallest pixel error among all detections
        for (AprilTagDetection d : dets) {
            if (d == null || d.center == null) continue;
            double score = -Math.abs(d.center.x - cx);
            if (score > bestScore) { bestScore = score; best = d; }
        }
        return best;
    }

    // Auto-aim: G2.A = ON, G2.B = OFF.
    // When ON:
    //  - The turret (Lazy Susan) scans until it finds the correct AprilTag.
    //  - Once locked, it centers the tag and *remembers* the proper shooter power.
    //  - We no longer spin the launcher motor here; we just compute power
    //    and let the normal launcher code decide when to shoot.
    private void updateAimbot() {
        // 1) Read buttons from gamepad 2 to turn aimbot ON or OFF.
        boolean aNow = gamepad2.a;
        boolean bNow = gamepad2.b;


        // If A was just pressed (rising edge), turn aimbot ON and reset state.
        if (aNow && !prevG2_A) {
            autoAimEnabled = true;
            aimHasLock = false;
            aimFramesWithTag = 0;
            aimFramesWithoutTag = 0;
            aimHasTarget = false;
        }
        // If B was just pressed, turn aimbot OFF and clear lock.
        if (bNow && !prevG2_B) {
            autoAimEnabled = false;
            aimHasLock = false;
            aimHasTarget = false;
        }


        // Save current button states for next loop.
        prevG2_A = aNow;
        prevG2_B = bNow;


        // 2) If aimbot is OFF, do nothing else here.
        if (!autoAimEnabled) {
            telemetry.addData("Aimbot", "OFF");
            return;
        }


        // 3) If we do not have a webcam, we cannot aim automatically.
        if (WEBCAM == null) {
            telemetry.addData("Aimbot", "ON (no webcam)");
            return;
        }


        // 4) Get the AprilTag processor from our webcam wrapper.
        AprilTagProcessor atp;
        try {
            atp = WEBCAM.getAprilTag();
        } catch (Exception e) {
            telemetry.addData("Aimbot", "No AprilTag processor");
            return;
        }
        if (atp == null) {
            telemetry.addData("Aimbot", "ON (no AprilTag pipeline)");
            return;
        }


        // 5) Ask the processor for all current detections and pick the best one
        //    for the current alliance (red uses tag 24, blue uses tag 20).  [oai_citation:0‡DECODE_CM_Section9_V7.pdf](sediment://file_00000000afb471fd8cf92749e6da45f8)
        List<AprilTagDetection> dets = atp.getDetections();
        AprilTagDetection tgt = pickBestDetection(dets);


        // 6) If we do NOT see any valid tag this frame:
        if (tgt == null || tgt.ftcPose == null) {
            aimHasTarget = false;
            aimFramesWithTag = 0;

            // If we had a lock, do NOT instantly go to full scan.
            // Instead, keep turning in the last known correct direction for a bit.
            if (aimHasLock || aimLockedTagId != -1) {
                aimFramesWithoutTag++;
                telemetry.addData("Aimbot", "ON (reacquiring)");

                // Keep following for a while before giving up.
                if (lazySusanMotor != null && aimFramesWithoutTag <= AIM_LOST_FRAMES_THRESHOLD) {

                    // Use last known motor direction first (best),
                    // else fallback to last error sign,
                    // else fallback to scan direction.
                    double dir = Math.signum(aimLastTurretCmd);
                    if (dir == 0.0) dir = Math.signum(aimLastErrorPx);
                    if (dir == 0.0) dir = aimScanDirection;

                    double cmd = dir * AIM_REACQUIRE_POWER;

                    // Clamp for safety
                    if (cmd >  AIM_POWER_MAX) cmd =  AIM_POWER_MAX;
                    if (cmd < -AIM_POWER_MAX) cmd = -AIM_POWER_MAX;

                    if (lazySusanMotor.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
                        lazySusanMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    }

                    lazySusanMotor.setPower(cmd);
                    telemetry.addData("AIM reacqCmd", "%.2f", cmd);
                    return;
                }

                // Too many lost frames: give up lock and go back to full scan.
                aimHasLock = false;
                aimLockedTagId = -1;
            }

            telemetry.addData("Aimbot", "ON (searching)");
            updateAimbotSearch();
            return;
        }

        // 7) We have a valid target this frame.
        aimFramesWithoutTag = 0;
        aimFramesWithTag++;

        // Extract yaw (angle left/right) and range (distance) from the tag.
        double yawRawDeg = tgt.ftcPose.yaw;               // + = target appears to the right
        double rangeM    = tgt.ftcPose.range * 0.0254;    // inches -> meters
        // --- Pixel error (center.x relative to camera center) ---
        double frameW = WEBCAM.getWidthPx();
        double cx = frameW / 2.0;

        telemetry.addData("AIM range(in)", "%.1f", tgt.ftcPose.range);
        telemetry.addData("AIM range(m)", "%.2f", rangeM);
        telemetry.addData("AIM corrected(m)", "%.2f", (rangeM * AIM_RANGE_SCALE) + AIM_RANGE_OFFSET_M);

        // Positive errorPx means tag is to the RIGHT of center.
        // If your turret turns the wrong direction, flip the sign of errorPx.
        double errorPxRaw = cx - tgt.center.x;


        // Low-pass filter the pixel error to reduce jitter.
        if (aimFramesWithTag == 1 && !aimHasLock) {
            aimFilteredErrPx = errorPxRaw;
        } else {
            aimFilteredErrPx = AIM_ERR_FILTER_ALPHA * errorPxRaw
                    + (1.0 - AIM_ERR_FILTER_ALPHA) * aimFilteredErrPx;
        }


        double errorPx = aimFilteredErrPx;


        // Lock logic (hysteresis):
        if (!aimHasLock && Math.abs(errorPx) <= AIM_MOVE_DEADBAND_PX) {
            aimHasLock = true;
            aimLockedTagId = tgt.id;
        }

        telemetry.addData("Aimbot", aimHasLock ? "ON (locked)" : "ON (tracking)");
        telemetry.addData("AIM errPx(raw/filt)", "%.1f / %.1f", errorPxRaw, errorPx);
        telemetry.addData("AIM centerX", "%.1f", tgt.center.x);


        // --- Turret motor command (proportional to pixel error) ---
        if (lazySusanMotor != null) {
            double cmd;


            // Stop in the tight lock band to eliminate shaking.
            if (Math.abs(errorPx) <= AIM_LOCK_DEADBAND_PX) {
                cmd = 0.0;
            } else {
                // Normalize error to [-1..1] then scale to motor power
                double norm = errorPx / cx;  // cx = half-width
                cmd = AIM_PIX_KP * norm;


                // Clamp to safe range
                if (cmd >  AIM_POWER_MAX) cmd =  AIM_POWER_MAX;
                if (cmd < -AIM_POWER_MAX) cmd = -AIM_POWER_MAX;


                // Soft-limit near mechanical bounds using encoder math.
                int ticksNow = lazySusanMotor.getCurrentPosition();
                double angleNowDeg = (ticksNow - lazyZeroTicks) / LAZY_TICKS_PER_DEG;


                double margin = LAZY_MAX_DEG - Math.abs(angleNowDeg);
                if (margin <= 0) {
                    boolean pushingOut =
                            (angleNowDeg >=  LAZY_MAX_DEG && cmd > 0) ||
                                    (angleNowDeg <= -LAZY_MAX_DEG && cmd < 0);
                    if (pushingOut) cmd = 0.0;
                }


                if (lazySusanMotor.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
                    lazySusanMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
            }


            lazySusanMotor.setPower(cmd);

            aimLastTurretCmd = cmd;
            aimLastErrorPx = errorPx;

            telemetry.addData("AIM turretCmd", "%.2f", cmd);
        }


        // --- Shooter power from distance (just compute & store, do NOT spin here) ---
        double autoPower = computeAimbotShooterPower(rangeM);
        aimHasTarget = true;
        aimLastShooterPower = autoPower;
        telemetry.addData("AIM shooterPower (suggested)", "%.2f", autoPower);
    }

    /**
     * Compute shooter power from distance (in meters).
     *
     * At AIM_RANGE_NEAR_M  (very close) -> use AIM_PWR_NEAR.
     * At AIM_RANGE_FAR_M   (~12 ft)     -> use AIM_PWR_FAR (full power).
     *
     * Everything in between is smoothly blended.
     */
    private double computeAimbotShooterPower(double rangeM) {
        // If distance looks crazy, fall back to a safe value.
        if (Double.isNaN(rangeM) || Double.isInfinite(rangeM)) {
            return 0.90;
        }


        // Correct the camera-based distance so it better matches shooter distance.
        double corrected = (rangeM * AIM_RANGE_SCALE) + AIM_RANGE_OFFSET_M;


        // Piecewise-linear interpolation through the calibration table.
        double power = interp1D(corrected, AIM_CAL_DIST_M, AIM_CAL_PWR);


        // Safety clamp.
        if (power < 0.0) power = 0.0;
        if (power > 1.0) power = 1.0;


        return power;
    }


    /**
     * 1D piecewise-linear interpolation.
     * If x is below xs[0], returns ys[0].
     * If x is above xs[last], returns ys[last].
     */
    private double interp1D(double x, double[] xs, double[] ys) {
        if (xs == null || ys == null || xs.length < 2 || ys.length != xs.length) {
            return 0.90; // safe fallback
        }


        if (x <= xs[0]) return ys[0];
        int last = xs.length - 1;
        if (x >= xs[last]) return ys[last];


        for (int i = 0; i < last; i++) {
            double x0 = xs[i];
            double x1 = xs[i + 1];
            if (x >= x0 && x <= x1) {
                double t = (x - x0) / (x1 - x0);
                return ys[i] + t * (ys[i + 1] - ys[i]);
            }
        }
        return ys[last];
    }

    /**
     * When aimbot is ON but no tag is visible, this method slowly
     * scans the turret left and right to look for the AprilTag.
     *
     * It uses encoder ticks to keep motion within ±AIM_SEARCH_LIMIT_DEG.
     */
    private void updateAimbotSearch() {
        // If we do not have a Lazy Susan motor, we cannot scan.
        if (lazySusanMotor == null) return;

        // Read current angle from encoder ticks.
        int ticksNow = lazySusanMotor.getCurrentPosition();
        double angleNowDeg = (ticksNow - lazyZeroTicks) / LAZY_TICKS_PER_DEG;

        // If we go past our search limit on one side, flip direction.
        if (angleNowDeg > AIM_SEARCH_LIMIT_DEG) {
            aimScanDirection = -1;  // start scanning back to the left
        } else if (angleNowDeg < -AIM_SEARCH_LIMIT_DEG) {
            aimScanDirection = 1;   // start scanning back to the right
        }

        // Compute a simple constant power in the current scan direction.
        double cmd = aimScanDirection * AIM_SCAN_POWER;

        // Make sure we are in RUN_USING_ENCODER for instant response.
        if (lazySusanMotor.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
            lazySusanMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        // Send command to the motor.
        lazySusanMotor.setPower(cmd);

        telemetry.addData("AIM searchAngle(deg)", "%.1f", angleNowDeg);
        telemetry.addData("AIM searchCmd",        "%.2f", cmd);
    }

    public void applyAllianceToWebcam() {
        if (WEBCAM == null) return;
        Webcam.Color color = AutoSettings.getAlliance() == AllianceColor.RED
                ? Webcam.Color.RED : Webcam.Color.BLUE;
        WEBCAM.setTargetColor(color);
    }

    /**
     * the loop once.
     */
    private void runLoop() {
        /* Gamepad 1 (Wheel and Webcam Controls) */

        /* Wheel Controls */
        /*
         * Drive robot based on joystick input from gamepad1
         * Right stick moves the robot forwards and backwards and turns it.
         * The triggers control strafing.  A positive left trigger causes
         * leftward strafe; a positive right trigger causes rightward strafe.
         */
        // Compute raw inputs from gamepad controls.  Subtract the offsets
        // measured during init so that small bias from imperfectly centred
        // sticks/triggers does not cause the robot to move by itself.  Then
        // apply a deadband to each value to clamp tiny drift to zero.
        //
        // Forward/backward comes from the right stick Y-axis (up = forward).
        // Negate the value so pushing forward yields positive.  Subtract
        // verticalOffset measured during init.
        double verticalRaw = -gamepad1.right_stick_y - verticalOffset;

        // Turning (pivot) comes from the left stick X-axis.  Subtract
        // pivotOffset measured during init.  Positive values produce
        // clockwise rotation.
        double pivotRaw = gamepad1.left_stick_x - pivotOffset;

        // Strafing (horizontal) comes from the triggers: right trigger (ZR)
        // minus left trigger (ZL).  Subtract horizontalOffset measured during
        // init.  Positive values strafe right, negative values strafe left.
        double horizontalRaw = (gamepad1.right_trigger - gamepad1.left_trigger) - horizontalOffset;

        // Apply deadband to each input to eliminate small stick drift and
        // unintended motion.  Scale the inputs by the driving sensitivity.
        double vertical   = applyDeadband(verticalRaw)   * DRIVING_SENSITIVITY;
        double horizontal = applyDeadband(horizontalRaw) * DRIVING_SENSITIVITY;
        double pivot      = applyDeadband(pivotRaw)      * DRIVING_SENSITIVITY;

        double frontRightPower = (-pivot + (vertical - horizontal));
        double backRightPower  = (-pivot + vertical + horizontal);
        double frontLeftPower  = (pivot + vertical + horizontal);
        double backLeftPower   = (pivot + (vertical - horizontal));

        // If all inputs are within the deadband, stop all four drive motors.
        // This prevents the robot from creeping when the sticks return to
        // centre.  By writing zero to each motor directly, we avoid any
        // lingering motion from previous commands.  Note that we do not
        // attempt to drive using MECANUM_DRIVE here; direct motor control
        // provides more predictable behaviour with the old mixing logic.
        if (vertical == 0.0 && horizontal == 0.0 && pivot == 0.0) {
            if (WHEELS != null) {
                // Access the individual motors and set their powers to zero.
                org.firstinspires.ftc.teamcode.hardwareSystems.MecanumWheels mech =
                        (org.firstinspires.ftc.teamcode.hardwareSystems.MecanumWheels) WHEELS;
                mech.getFrontLeftMotor().setPower(0);
                mech.getFrontRightMotor().setPower(0);
                mech.getBackLeftMotor().setPower(0);
                mech.getBackRightMotor().setPower(0);
            } else if (MECANUM_DRIVE != null) {
                // For Road Runner fallback, send zero drive powers.
//                MECANUM_DRIVE.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
            }
        } else {
            // Compute wheel powers using the same equations from the old
            // participant’s code.  These equations assume that the left
            // motors have their directions set to REVERSE and the right
            // motors are set to FORWARD.  See the initialisation in
            // CustomLinearOp.initWheels().
            frontRightPower = (-pivot + (vertical - horizontal));
            backRightPower  = (-pivot + vertical + horizontal);
            frontLeftPower  = (pivot + vertical + horizontal);
            backLeftPower   = (pivot + (vertical - horizontal));

            // Normalize the powers so that no motor command exceeds ±1.0.
            double maxMagnitude = Math.max(1.0, Math.max(Math.abs(frontLeftPower),
                    Math.max(Math.abs(frontRightPower), Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)))));
            frontLeftPower  /= maxMagnitude;
            frontRightPower /= maxMagnitude;
            backLeftPower   /= maxMagnitude;
            backRightPower  /= maxMagnitude;

            // Apply the powers to the individual motors.  By setting the
            // powers directly we bypass MecanumWheels.drive() and avoid any
            // inconsistencies arising from mismatched mixing formulas.
            if (WHEELS != null) {
                org.firstinspires.ftc.teamcode.hardwareSystems.MecanumWheels mech =
                        (org.firstinspires.ftc.teamcode.hardwareSystems.MecanumWheels) WHEELS;
                mech.getFrontLeftMotor().setPower(frontLeftPower);
                mech.getFrontRightMotor().setPower(frontRightPower);
                mech.getBackLeftMotor().setPower(backLeftPower);
                mech.getBackRightMotor().setPower(backRightPower);
            } else if (MECANUM_DRIVE != null) {
                // For Road Runner fallback, convert our directional commands to the
                // +y forward/+x right convention.  Note that vertical controls
                // forward/backward; horizontal controls strafe; pivot controls
                // rotation.  The forward value must be negated because
                // PoseVelocity2d expects +y forward.
//                PoseVelocity2d velocity = new PoseVelocity2d(
//                        new Vector2d(horizontal, -vertical),
//                        pivot
//                );
//                MECANUM_DRIVE.setDrivePowers(velocity);
            }
        }

        // Telemetry: report the raw and processed inputs as well as the
        // computed motor powers.  This aids in diagnosing drift or
        // inversion issues when testing on the field.
        telemetry.addData("Vertical/raw", "%5.2f / %5.2f", vertical, verticalRaw);
        telemetry.addData("Horizontal/raw", "%5.2f / %5.2f", horizontal, horizontalRaw);
        telemetry.addData("Pivot/raw", "%5.2f / %5.2f", pivot, pivotRaw);
        if (WHEELS != null ) {
            telemetry.addData("Front left wheel power",  frontLeftPower);
            telemetry.addData("Front right wheel power", frontRightPower);
            telemetry.addData("Back left wheel power",   backLeftPower);
            telemetry.addData("Back right wheel power",  backRightPower);
        }

        // ===== GAMEPAD 1 LIMITER SERVO CONTROL =====
        // X  -> move limiter OUT (≈90 degrees)
        // Y  -> move limiter back HOME.
        if (limiterServo != null) {
            // When driver presses X on gamepad 1, extend the limiter.
            if (gamepad1.x) {
                limiterServo.setOut();
            }


            // When driver presses Y on gamepad 1, retract the limiter.
            if (gamepad1.y) {
                limiterServo.setHome();
            }


//            // Telemetry so you can confirm the code is actually running.
//            telemetry.addData("LimiterPos", "%.2f", limiterServo.getPosition());
//            telemetry.addData("LimiterG1.X", gamepad1.x);
//            telemetry.addData("LimiterG1.Y", gamepad1.y);
        }



        if (hoodServo != null) {
            if (gamepad2.dpad_left) {
                hoodServo.setOut();

            } else if (gamepad2.dpad_right) {
                hoodServo.setHome();

            }


        }

        /* Webcam controls */
        // Save CPU resources; can resume streaming when needed.  When the
        // driver presses D‑pad down, stop streaming; D‑pad up resumes
        // streaming.  Check that WEBCAM is not null before calling its
        // methods.
//        if (gamepad1.dpad_down) {
//            if (WEBCAM != null) {
//                WEBCAM.getVisionPortal().stopLiveView();
//            }
//        } else if (gamepad1.dpad_up) {
//            if (WEBCAM != null) {
//                WEBCAM.getVisionPortal().resumeLiveView();
//            }
//        }

        // Note: If you see a "camera not identified" error on the Driver Station
        // telemetry, verify that your webcam is configured with the name
        // "Webcam 1" in the Control Hub configuration.  If it uses a
        // different name (for example "Webcam"), change the name passed to
        // hardwareMap.get() in CustomLinearOp.initWebcam().

        /* Intake control (Gamepad 2)
         *
         * Use the left trigger on gamepad2 to control the intake.  When the
         * left trigger is held down, run the intake at full power; when the
         * trigger is released, stop the intake.  This allows drivers to
         * toggle the intake on demand rather than running continuously.
         * The opposite should happen when the right trigger is held down
         */
        if (intakeMotor != null) {
            // The left trigger on gamepad2 runs the intake forward; the
            // right trigger runs it in reverse.  If neither trigger is
            // pressed beyond the threshold, stop the intake.  Adjust the
            // threshold if you want partial trigger pull to be ignored.
            if (gamepad2.x) {
                intakeMotor.setPower(1.0);
            } else if (gamepad2.y) {
                intakeMotor.setPower(-1.0);
            } else {
                intakeMotor.setPower(0.0);
            }
        }

        if (launcherMotor != null) {
            // Read encoder velocity in ticks/second and convert to RPM.
            // RPM = (ticksPerSecond / ticksPerRev) * 60.
            double ticksPerSecond = launcherMotor.getVelocity();
            double rpm = (ticksPerSecond / LAUNCHER_TICKS_PER_REV) * 60.0;
            lastLauncherRpm = rpm;
            lastLauncherRpm = rpm;


            // Show launcher RPM on telemetry.
            telemetry.addData("Launcher RPM", "%.0f", rpm);


            if (gamepad2.left_trigger > 0.05) {
                // Reverse to clear balls or unjam
                launcherMotor.setPower(-1.0);
            } else if (gamepad2.right_trigger > 0.05) {
                // Normal shooting direction.
                // If aimbot is ON and we currently have a valid tag,
                // use the auto-calculated power from distance.
                double power = 1.0;  // default manual power
                if (autoAimEnabled && aimHasTarget) {
                    power = aimLastShooterPower;
                }


                launcherMotor.setPower(power);
            } else if (gamepad2.right_bumper) {
                launcherMotor.setPower(0.65);   // clipped to 1.0 internally
            } else if (gamepad2.left_bumper) {
                launcherMotor.setPower(-85);  // clipped to -1.0 internally
            } else {
                // No trigger -> do not spin the launcher at all.
                launcherMotor.setPower(0.0);
            }
        }

// Gamepad 2 LEFT STICK (X): Lazy Susan instant control with ±180° soft limit
        if (!autoAimEnabled && lazySusanMotor != null) {
            // 1) Read stick with small deadband
            double lsxRaw = gamepad2.left_stick_x;
            double lsx    = (Math.abs(lsxRaw) < LAZY_DEADBAND) ? 0.0 : lsxRaw;

            // 2) Smooth it (simple EMA) so motion feels fluid
            lazyStickFilt = LAZY_FILTER_ALPHA * lazyStickFilt + (1.0 - LAZY_FILTER_ALPHA) * lsx;

            // 3) Compute current angle from encoder ticks
            int    ticksNow        = lazySusanMotor.getCurrentPosition();
            double angleNowDeg     = (ticksNow - lazyZeroTicks) / LAZY_TICKS_PER_DEG;

            // 4) Update target angle for telemetry (not required to move)
            lazyTargetDeg += lazyStickFilt * 3.0; // a small “mental” target for display
            if (lazyTargetDeg >  LAZY_MAX_DEG) lazyTargetDeg =  LAZY_MAX_DEG;
            if (lazyTargetDeg < -LAZY_MAX_DEG) lazyTargetDeg = -LAZY_MAX_DEG;

            // 5) Soft-limit scaling as we approach ±180°
            double margin = LAZY_MAX_DEG - Math.abs(angleNowDeg);
            double softScale = 1.0;
            if (margin <= 0) {
                // already at/over hard limit -> only allow power back toward center
                boolean pushingOut = (angleNowDeg >=  LAZY_MAX_DEG && lazyStickFilt > 0) ||
                        (angleNowDeg <= -LAZY_MAX_DEG && lazyStickFilt < 0);
                softScale = pushingOut ? 0.0 : 1.0;
            } else if (margin < LAZY_SOFT_ZONE_DEG) {
                // ease power in last few degrees
                softScale = Math.max(0.0, margin / LAZY_SOFT_ZONE_DEG);
            }

            // 6) Power command: instant, smoothed, limited near bounds
            double cmd = LAZY_GAIN * lazyStickFilt * softScale;
            cmd = Math.max(-LAZY_POWER_MAX, Math.min(LAZY_POWER_MAX, cmd));

            // Ensure we’re in the instant-response mode
            if (lazySusanMotor.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
                lazySusanMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            lazySusanMotor.setPower(cmd);

            telemetry.addData("LazySusan ang(deg)", "%.1f", angleNowDeg);
            telemetry.addData("LazySusan cmd",      "%.2f", cmd);
        }

        updateAimbot();

        telemetry.update();
    }

    @Override
    public void runOpMode() {
        super.runOpMode();
        if (cameraStreamEnabled) {
            FtcDashboard.getInstance().startCameraStream(WEBCAM.getVisionPortal(), 0);
        }

        boolean toggleBtn = gamepad2.dpad_up;
        if (toggleBtn && !lastToggleBtn) {
            cameraStreamEnabled = !cameraStreamEnabled;
            if (cameraStreamEnabled) {
                FtcDashboard.getInstance().startCameraStream(WEBCAM.getVisionPortal(), 0);
            } else {
                FtcDashboard.getInstance().stopCameraStream();
            }
        }
        lastToggleBtn = toggleBtn;

        AutoSettings.readFromFile();
        applyAllianceToWebcam();
        AllianceColor ac = AutoSettings.getAlliance();
        TeamSide ts      = AutoSettings.getTeamSide();
        telemetry.addData("Alliance", ac);
        telemetry.addData("Team Side", ts);
        telemetry.update();

        telemetry.addLine("Select: X=RED, B=BLUE, up=FAR, down=NEAR");
        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.dpad_left)  AutoSettings.set(AllianceColor.RED,  AutoSettings.getTeamSide(), false);
            if (gamepad1.dpad_right)  AutoSettings.set(AllianceColor.BLUE, AutoSettings.getTeamSide(), false);
            if (gamepad1.dpad_up)   AutoSettings.set(AutoSettings.getAlliance(), TeamSide.FAR, false);
            if (gamepad1.dpad_down) AutoSettings.set(AutoSettings.getAlliance(), TeamSide.NEAR, false);
            telemetry.addData("Alliance", AutoSettings.getAlliance());
            telemetry.addData("Team Side", AutoSettings.getTeamSide());
            telemetry.update();
        }
        AutoSettings.writeToFile();
        applyAllianceToWebcam();

        // Sync aimbot alliance flag with AutoSettings
        allianceIsRed = (AutoSettings.getAlliance() == AllianceColor.RED);

        // -----------------------------------------------------------------
        // Calibrate driver control offsets.  Ask the driver to release all
        // sticks and triggers during the init period.  Sample the raw
        // values over a brief interval to compute average offsets.  These
        // offsets are subtracted from the raw inputs each loop to cancel
        // out any bias caused by imperfect centring of the controls.
        // -----------------------------------------------------------------
        telemetry.addLine("Calibrating controls... release sticks/triggers");
        telemetry.update();
        long sampleEnd = System.currentTimeMillis() + 500; // sample for 0.5 s
        double vSum = 0.0;
        double hSum = 0.0;
        double pSum = 0.0;
        int samples = 0;
        while (!isStopRequested() && System.currentTimeMillis() < sampleEnd) {
            // Sample the raw inputs using the same axes used in runLoop.
            // Use the same conventions as runLoop: negate right stick Y for forward,
            // and use the difference of triggers for strafe.  Do not apply
            // deadband here; we want the true rest position.
            double vRaw = -gamepad1.right_stick_y;
            double pRaw = gamepad1.left_stick_x;
            double hRaw = gamepad1.right_trigger - gamepad1.left_trigger;

            vSum += vRaw;
            hSum += hRaw;
            pSum += pRaw;
            samples++;
            sleep(10);
        }
        if (samples > 0) {
            verticalOffset = vSum / samples;
            horizontalOffset = hSum / samples;
            pivotOffset = pSum / samples;
        }
        telemetry.addData("Control offsets", "V=%.2f H=%.2f P=%.2f", verticalOffset, horizontalOffset, pivotOffset);
        telemetry.update();

        // Initialise additional subsystem motors: lazy Susan and launcher.  Both
        // are DcMotorEx so we can access advanced functions if needed.  If
        // either motor is not found, report a warning and continue; null
        // checks in runLoop() will prevent NullPointerExceptions.
        try {
            lazySusanMotor = hardwareMap.get(DcMotorEx.class, "lazySusanMotor");
            // Flip to REVERSE if left/right feels backward (or negate LAZY_GAIN below).
            lazySusanMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            lazySusanMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            // Zero encoder so “0°” = start facing
            lazySusanMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lazySusanMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // <<< instant response
            lazySusanMotor.setPower(0.0);

            lazyZeroTicks = 0;
            lazyTargetDeg = 0.0;
            telemetry.addLine("Lazy Susan motor initialised (RUN_USING_ENCODER)");
        } catch (Exception e) {
            telemetry.addLine("WARNING: Lazy Susan motor not found");
        }

        try {
            launcherMotor = hardwareMap.get(DcMotorEx.class, "launcherMotor");
            // The launcher spins counter‑clockwise when viewed from the front
            // of the robot; set direction accordingly.  Reverse if needed.
            launcherMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            launcherMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            launcherMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            launcherMotor.setPower(0.0);
            telemetry.addLine("Launcher motor initialised");
        } catch (Exception e) {
            telemetry.addLine("WARNING: Launcher motor not found");
        }


        // Initialise the intake motor.  This motor is configured as a
        // REV Core Hex motor (72:1 gearbox, 288 counts per revolution) with an
        // integrated encoder.  In the Robot Controller (Control Hub)
        // configuration, ensure you have added a DC motor device named
        // "intakeMotor" connected to an unused motor port (for example
        // port 0 on the Expansion Hub).  See "Hardware configuration"
        // instructions below for details.
        try {
            intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");

            // Set motor direction.  Forward is assumed to pull game pieces
            // into the robot; reverse this if your intake spins the wrong way.
            intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);

            // Use RUN_WITHOUT_ENCODER because we are driving the motor at a
            // constant power and do not need velocity control.  This mode
            // disables the built‑in velocity PID and simply applies a
            // percentage of available voltage.
            intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            // Allow the motor to coast when zero power is applied.  If you
            // prefer the motor to brake when stopped, change to BRAKE.
            intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            // Do not start the intake motor here.  Intake power is controlled
            // by the left trigger on gamepad 2 in runLoop().  Initialise
            // the motor with zero power so it remains stopped until the
            // driver presses the trigger.
            intakeMotor.setPower(0.0);

            telemetry.addLine("Intake motor initialised.  Hold LT on gamepad 2 to run.");
        } catch (Exception e) {
            telemetry.addLine("WARNING: Intake motor not found.\n" + e.getMessage());
        }

        // ----- Limiter servo init -----
        try {
            // Create the limiter servo wrapper using the shared hardware class.
            // "limiterServo" must match the name in the RC config.
            limiterServo = new LimiterServo(hardwareMap,
                    "limiterServo",
                    LIMITER_HOME_POS,
                    LIMITER_OUT_POS);


            telemetry.addLine("Limiter servo initialised");

        } catch (Exception e) {
            telemetry.addLine("WARNING: Limiter servo not found.\n" + e.getMessage());

        }

        try {
            // Create the limiter servo wrapper using the shared hardware class.
            // "hoodServo" must match the name in the RC config.
            hoodServo = new LimiterServo(hardwareMap,
                    "hoodServo",
                    HOOD_HOME_POS,
                    HOOD_OUT_POS);


            telemetry.addLine("Hood servo initialised");

        } catch (Exception e) {
            telemetry.addLine("WARNING: Hood servo not found.\n" + e.getMessage());

        }

        while (opModeIsActive()) {
            try {
                runLoop();
            } catch (Exception e) {
                telemetry.addLine("\nWARNING AN ERROR OCCURRED!!!");
                telemetry.addLine(e.getMessage());
            }
        }

        // Stop the intake motor when the OpMode ends.  This ensures the
        // motor does not continue running after you press Stop on the
        // Driver Station.  Check for null in case the motor failed
        // initialisation.
        if (intakeMotor != null) {
            intakeMotor.setPower(0.0);
        }
    }
}