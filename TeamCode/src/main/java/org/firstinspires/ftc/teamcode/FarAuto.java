package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.hardwareSystems.LimiterServo;
import org.firstinspires.ftc.teamcode.hardwareSystems.MecanumWheels;

@Autonomous(name = "FarAuto")
public class FarAuto extends CustomLinearOp {
    // Limiter servo positions (tuned on the real robot)
    private static final double LIMITER_HOME_POS = 0.25; // retracted
    private static final double LIMITER_OUT_POS = 0.10; // extended ≈90°
    // Timing constants (milliseconds)
    private static final long LAUNCHER_SPINUP_MS = 3000; // time to spin up
    // the flywheel
    private static final long FEED_ALL_BALLS_MS = 2000; // feed time for
    // three balls
    private static final long EXTRA_FEED_SECOND_SHOT_MS = 2000; // +2 seconds
    // on cycle 1
    private static final long STRAFE_MS = 1725; // time to strafe to pickup zone
    private static final long EXIT_STRAFE_MS = 2000; // tune this on-field
    private static final long DRIVE_PICKUP_MS = 2000; // drive forward/back
    // Motor powers
    private static final double LAUNCHER_POWER = 0.70;  // full power for
    // shooting
    private static final double INTAKE_PICK_POWER = 1.0;  // positive to pick
    // up balls
    private static final double INTAKE_FEED_POWER = -1.0; // negative to feed
    // to collect balls
    // into shooter
    private static final double STRAFE_POWER = 0.6;  // strafe speed (0.0‑1.0)
    private static final double DRIVE_POWER = 0.3;  // drive speed (0.0‑1.0)
    // Hardware references
    private DcMotorEx launcherMotor;
    private DcMotorEx intakeMotor;
    private LimiterServo limiterServo;

    /**
     * Drive the robot forward or backward by setting all wheel powers to the
     * same value.  Positive power moves the robot forward; negative power moves
     * it backward.
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

    @Override
    public void runOpMode() {
        // Initialise hardware and wait for the start command. This will also
        // apply any alliance settings and configure the drive train.
        super.runOpMode();

        // Map the launcher motor.
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

        // Map the intake motor.
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
            limiterServo = new LimiterServo(hardwareMap, "limiterServo",
                    LIMITER_HOME_POS, LIMITER_OUT_POS);
            telemetry.addLine("Limiter servo initialised");
        } catch (Exception e) {
            telemetry.addLine("WARNING: limiterServo not found in HardAuto");
        }

        telemetry.update();

        // Abort if stop is requested before the match begins
        if (isStopRequested()) {
            return;
        }

        driveForward(0.5, 1000);
    }
}
