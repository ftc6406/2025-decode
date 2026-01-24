package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.hardwareSystems.LimiterServo;
import org.firstinspires.ftc.teamcode.hardwareSystems.MecanumWheels;

/**
 * Hard‑coded autonomous routine for FTC.
 *
 * <p>This OpMode performs the following sequence twice:
 *
 * <ol>
 *   <li>Spin up the launcher and fire three pre‑loaded balls at full power.</li>
 *   <li>Strafe to a pickup zone.</li>
 *   <li>Extend the limiter to hold picked balls in place.</li>
 *   <li>Drive forward while running the intake to collect three lined‑up balls.</li>
 *   <li>Back up to the original distance.</li>
 *   <li>Retract the limiter.</li>
 *   <li>Strafe back to the starting position.</li>
 *   <li>Repeat from step&nbsp;1 to shoot the newly collected balls.</li>
 * </ol>
 *
 * <p>Distances and durations in this routine are approximate and should be
 * tuned on the real robot.  Positive drive power moves the robot forward;
 * negative drive power moves it backward.  Positive strafe power moves the
 * robot to the right; negative strafe power moves it to the left.  The
 * intake motor runs in reverse to feed balls into the shooter and forward
 * to collect balls off the field.</p>
 */
@Autonomous(name = "RedAuto")
public class BlueAuto extends CustomLinearOp {

    // Hardware references
    private DcMotorEx launcherMotor;
    private DcMotorEx intakeMotor;
    private LimiterServo limiterServo;

    // Limiter servo positions (tuned on the real robot)
    private static final double LIMITER_HOME_POS = 0.25; // retracted
    private static final double LIMITER_OUT_POS  = 0.10; // extended ≈90°

    // Timing constants (milliseconds)
    private static final long LAUNCHER_SPINUP_MS    = 3000; // time to spin up the flywheel
    private static final long FEED_ALL_BALLS_MS     = 2000; // feed time for three balls
    private static final long EXTRA_FEED_SECOND_SHOT_MS = 2000; // +2 seconds on cycle 1
    private static final long STRAFE_MS             = 1725; // time to strafe to pickup zone
    private static final long EXIT_STRAFE_MS        = 2000; // tune this on-field
    private static final long DRIVE_PICKUP_MS       = 2000; // drive forward/back to collect balls

    // Motor powers
    private static final double LAUNCHER_POWER      = 0.70;  // full power for shooting
    private static final double INTAKE_PICK_POWER   = 1.0;  // positive to pick up balls
    private static final double INTAKE_FEED_POWER   = -1.0; // negative to feed into shooter
    private static final double STRAFE_POWER        = 0.6;  // strafe speed (0.0‑1.0)
    private static final double DRIVE_POWER         = 0.3;  // drive speed (0.0‑1.0)

    @Override
    public void runOpMode() {
        // Initialise hardware and wait for the start command.  This will also
        // apply any alliance settings and configure the drive train.
        super.runOpMode();

        // Map the launcher motor
        try {
            launcherMotor = hardwareMap.get(DcMotorEx.class, "launcherMotor");
            // The launcher motor spins a flywheel.  Reverse its direction so
            // positive power shoots balls out.  The motor is run without an
            // encoder because precise speed control is not required here.
            launcherMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            launcherMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            launcherMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            launcherMotor.setPower(0.0);
            telemetry.addLine("Launcher motor initialised");
        } catch (Exception e) {
            telemetry.addLine("WARNING: launcherMotor not found in HardAuto");
        }

        // Map the intake motor
        try {
            intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
            intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            intakeMotor.setPower(0.0);
            telemetry.addLine("Intake motor initialised");
        } catch (Exception e) {
            telemetry.addLine("WARNING: intakeMotor not found in HardAuto");
        }

        // Map the limiter servo.  If it is not present in the RC config, this
        // call will throw an exception which is caught below.
        try {
            limiterServo = new LimiterServo(hardwareMap,
                    "limiterServo",
                    LIMITER_HOME_POS,
                    LIMITER_OUT_POS);
            telemetry.addLine("Limiter servo initialised");
        } catch (Exception e) {
            telemetry.addLine("WARNING: limiterServo not found in HardAuto");
        }

        telemetry.update();

        // Abort if stop is requested before the match begins
        if (isStopRequested()) return;

        // Perform two cycles: shoot initial preload, then collect and shoot again
        for (int cycle = 0; cycle < 2 && opModeIsActive(); cycle++) {

            // Only reposition before the FIRST shooting cycle
            if (cycle == 0) {
                driveForward(-0.5, 500);
                sleep(1000);
            }

            // 1. Spin up launcher
            if (launcherMotor != null) {
                launcherMotor.setPower(LAUNCHER_POWER);
            }
            sleep(LAUNCHER_SPINUP_MS);

            // 2. Feed all pre‑loaded balls into the launcher
            if (intakeMotor != null) {
                intakeMotor.setPower(INTAKE_FEED_POWER);
            }
            sleep(4000);
            if (intakeMotor != null) {
                intakeMotor.setPower(0.0);
            }
            // Stop the launcher after shooting
            if (launcherMotor != null) {
                launcherMotor.setPower(0.0);
            }

            // After the first cycle, collect new balls and shoot again
            if (cycle == 0) {
                // 3. Strafe to pickup zone
                strafe(-STRAFE_POWER, STRAFE_MS);

//                // 4. Extend limiter to capture incoming balls
//                if (limiterServo != null) {
//                    limiterServo.setOut();
//                }

                // Run intake for ONLY 4 seconds total during pickup + return
                final long INTAKE_RUN_MS = 1800;

                // Turn intake on right before moving
                if (intakeMotor != null) {
                    intakeMotor.setPower(-INTAKE_PICK_POWER);
                    telemetry.addData("Intake cmd", "%.2f", -INTAKE_PICK_POWER);
                } else {
                    telemetry.addLine("ERROR: intakeMotor is NULL during pickup");
                }
                telemetry.update();

                sleep(1000);

                // Forward pickup (intake running)
                long forwardMs = Math.min(DRIVE_PICKUP_MS, INTAKE_RUN_MS);
                driveForward(DRIVE_POWER, forwardMs);

                long remainingIntakeMs = INTAKE_RUN_MS - forwardMs;

                // Back up (intake runs only for remaining time)
                long backWithIntakeMs = Math.min(DRIVE_PICKUP_MS, Math.max(0, remainingIntakeMs));
                if (backWithIntakeMs > 0) {
                    driveForward(-DRIVE_POWER, backWithIntakeMs);
                }

                // Stop intake exactly at 4 seconds total
                if (intakeMotor != null) {
                    intakeMotor.setPower(0.0);
                }

                // Finish any remaining backup time with intake OFF
                long backRemainingMs = DRIVE_PICKUP_MS - backWithIntakeMs;
                if (backRemainingMs > 0) {
                    driveForward(-DRIVE_POWER, backRemainingMs);
                }

//
//                // 7. Retract limiter
//                if (limiterServo != null) {
//                    limiterServo.setHome();
//                }

                // 8. Strafe back to original position
                strafe(STRAFE_POWER, STRAFE_MS);

                // 9. Small reposition first (no intake yet)
                driveForward(0.5, 500);
            }
        }

        // After completing both cycles, strafe LEFT out of shooting range
        if (opModeIsActive()) {
            strafe(-STRAFE_POWER, EXIT_STRAFE_MS);

            sleep(2000);

            driveForward(0.6, 500);
        }

        // Ensure all motors are stopped at the end of the routine
        if (launcherMotor != null) launcherMotor.setPower(0.0);
        if (intakeMotor != null)   intakeMotor.setPower(0.0);
        if (WHEELS != null) {
            try {
                MecanumWheels mech = (MecanumWheels) WHEELS;
                mech.getFrontLeftMotor().setPower(0.0);
                mech.getFrontRightMotor().setPower(0.0);
                mech.getBackLeftMotor().setPower(0.0);
                mech.getBackRightMotor().setPower(0.0);
            } catch (Exception ignore) {}
        }
        telemetry.addLine("HardAuto routine complete");
        telemetry.update();
    }

    /**
     * Strafe the robot left or right by applying opposite power to the
     * appropriate wheels.  Positive power strafes right; negative power
     * strafes left.
     *
     * @param power magnitude of the strafe (0.0‑1.0)
     * @param ms    duration in milliseconds
     */
// Tune this if strafe drifts forward/backward.
// If it drifts FORWARD, make negative (ex: -0.08).
// If it drifts BACKWARD, make positive (ex: +0.08).
    private static final double STRAFE_Y_COMP = -0.08;

    private void strafe(double power, long ms) {
        // power > 0 = strafe right, power < 0 = strafe left
        double verticalComp = STRAFE_Y_COMP * Math.abs(power);
        driveRobotCentric(verticalComp, power, 0.0, ms);
    }

    /**
     * Drive the robot forward or backward by setting all wheel powers to the
     * same value.  Positive power moves the robot forward; negative power
     * moves it backward.
     *
     * @param power magnitude of drive (0.0‑1.0)
     * @param ms    duration in milliseconds
     */
    private void driveForward(double power, long ms) {
        if (WHEELS == null) {
            sleep(ms);
            return;
        }
        try {
            MecanumWheels mech = (MecanumWheels) WHEELS;
            mech.getFrontLeftMotor().setPower(power);
            mech.getFrontRightMotor().setPower(power);
            mech.getBackLeftMotor().setPower(power);
            mech.getBackRightMotor().setPower(power);
            sleep(ms);
            mech.getFrontLeftMotor().setPower(0.0);
            mech.getFrontRightMotor().setPower(0.0);
            mech.getBackLeftMotor().setPower(0.0);
            mech.getBackRightMotor().setPower(0.0);
        } catch (Exception e) {
            sleep(ms);
        }
    }

    private void driveRobotCentric(double vertical, double horizontal, double pivot, long ms) {
        if (WHEELS == null) { sleep(ms); return; }

        MecanumWheels mech = (MecanumWheels) WHEELS;

        double frontRightPower = (-pivot + (vertical - horizontal));
        double backRightPower  = (-pivot + vertical + horizontal);
        double frontLeftPower  = ( pivot + vertical + horizontal);
        double backLeftPower   = ( pivot + (vertical - horizontal));

        double maxMagnitude = Math.max(1.0, Math.max(Math.abs(frontLeftPower),
                Math.max(Math.abs(frontRightPower), Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)))));
        frontLeftPower  /= maxMagnitude;
        frontRightPower /= maxMagnitude;
        backLeftPower   /= maxMagnitude;
        backRightPower  /= maxMagnitude;

        mech.getFrontLeftMotor().setPower(frontLeftPower);
        mech.getFrontRightMotor().setPower(frontRightPower);
        mech.getBackLeftMotor().setPower(backLeftPower);
        mech.getBackRightMotor().setPower(backRightPower);

        sleep(ms);

        mech.getFrontLeftMotor().setPower(0);
        mech.getFrontRightMotor().setPower(0);
        mech.getBackLeftMotor().setPower(0);
        mech.getBackRightMotor().setPower(0);
    }
}