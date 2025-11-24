package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

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

    /**
     * the loop once.
     */
    private void runLoop() {
        /* Gamepad 1 (Wheel and Webcam Controls) */

        /* Wheel Controls */
        /*
         * Drive robot based on joystick input from gamepad1.
         * Right stick Y controls forward/backward motion.
         * Left stick X controls turning (pivoting in place).
         * Triggers control strafing: hold the right trigger (ZR) to strafe right,
         * hold the left trigger (ZL) to strafe left.  If both triggers are
         * pressed, the left trigger (strafe left) takes precedence.
         *
         * This mixing logic is adapted from last season's successful driver code.
         * It computes individual wheel powers based on three components:
         *   - vertical (forward/back) = -gamepad1.right_stick_y
         *   - pivot (turn) = gamepad1.left_stick_x
         *   - horizontal (strafe) = (right_trigger - left_trigger)
         * The left motors are reversed in the hardware configuration.  The
         * resulting pattern ensures that when strafing left the left wheels move
         * toward each other and the right wheels move away from each other, and
         * vice versa for strafing right.  Normal driving (forward/back/turn) is
         * preserved.  A deadband is applied to each input to suppress small
         * joystick drift.
         */

        // Compute raw components from gamepad1.  Forward/back is on the
        // right stick Y axis (push up for forward).  Turning uses the left
        // stick X axis (right is clockwise).  Strafe uses the difference
        // between triggers: right trigger gives positive (right strafe), left
        // trigger gives negative (left strafe).  If both triggers are pressed,
        // the left trigger will dominate due to subtraction.
        double rawVertical = -gamepad1.right_stick_y;
        double rawPivot    = gamepad1.left_stick_x;
        double rawHorizontal;
        if (gamepad1.left_trigger > 0) {
            // Left trigger pressed: negative strafe (left)
            rawHorizontal = -gamepad1.left_trigger;
        } else {
            // Else use right trigger: positive strafe (right)
            rawHorizontal = gamepad1.right_trigger;
        }

        // Apply deadband and sensitivity scaling.  This prevents the robot
        // from creeping when the sticks are near centre and scales the
        // response for driver comfort.  Note: do not invert the signs here.
        double vertical   = applyDeadband(rawVertical)   * DRIVING_SENSITIVITY;
        double pivot      = applyDeadband(rawPivot)      * DRIVING_SENSITIVITY;
        double horizontal = applyDeadband(rawHorizontal) * DRIVING_SENSITIVITY;

        // If all inputs are within the deadband, stop the robot.
        if (vertical == 0.0 && horizontal == 0.0 && pivot == 0.0) {
            if (WHEELS != null) {
                // Stop motors directly to avoid drift.
                if (WHEELS instanceof org.firstinspires.ftc.teamcode.hardwareSystems.MecanumWheels) {
                    org.firstinspires.ftc.teamcode.hardwareSystems.MecanumWheels mech =
                            (org.firstinspires.ftc.teamcode.hardwareSystems.MecanumWheels) WHEELS;
                    mech.getFrontLeftMotor().setPower(0);
                    mech.getFrontRightMotor().setPower(0);
                    mech.getBackLeftMotor().setPower(0);
                    mech.getBackRightMotor().setPower(0);
                } else {
                    // Fallback: use generic drive if not a MecanumWheels instance.
                    WHEELS.drive(0, 0, 0);
                }
            } else if (MECANUM_DRIVE != null) {
                MECANUM_DRIVE.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
            }
        } else {
            // Compute wheel powers using old participant's formula adapted to
            // our control mapping.  vertical = forward/back, pivot = turn,
            // horizontal = strafe.  The left motors are reversed in the
            // hardware configuration so we use the same signs as before.
            double frontRightPower = (-pivot + (vertical - horizontal));
            double backRightPower  = (-pivot + vertical + horizontal);
            double frontLeftPower  = (pivot + vertical + horizontal);
            double backLeftPower   = (pivot + (vertical - horizontal));

            // Normalize the powers so no value exceeds |1|.
            double max = Math.max(
                    Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                    Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))
            );
            if (max > 1.0) {
                frontLeftPower  /= max;
                frontRightPower /= max;
                backLeftPower   /= max;
                backRightPower  /= max;
            }

            // Apply the computed powers to the motors.  If WHEELS is a
            // MecanumWheels instance, set each motor directly; otherwise
            // construct a PoseVelocity2d for Road Runner using our components.
            if (WHEELS != null && WHEELS instanceof org.firstinspires.ftc.teamcode.hardwareSystems.MecanumWheels) {
                org.firstinspires.ftc.teamcode.hardwareSystems.MecanumWheels mech =
                        (org.firstinspires.ftc.teamcode.hardwareSystems.MecanumWheels) WHEELS;
                mech.getFrontLeftMotor().setPower(frontLeftPower);
                mech.getFrontRightMotor().setPower(frontRightPower);
                mech.getBackLeftMotor().setPower(backLeftPower);
                mech.getBackRightMotor().setPower(backRightPower);
            } else if (MECANUM_DRIVE != null) {
                // Convert our vertical/pivot/horizontal into Road Runner's
                // coordinate system: +y forward, +x right.  Our vertical is
                // positive when pushing stick up (forward), so no inversion is
                // needed.  Horizontal is positive for right strafe.  Pivot is
                // positive for clockwise rotation, which matches Road Runner.
                PoseVelocity2d velocity = new PoseVelocity2d(
                        new Vector2d(horizontal, vertical),
                        pivot
                );
                MECANUM_DRIVE.setDrivePowers(velocity);
            }

            // Telemetry: output wheel powers when available.  This helps
            // diagnose issues with strafe direction.  Only show when using
            // MecanumWheels to avoid clutter with Road Runner.
            if (WHEELS != null && WHEELS instanceof org.firstinspires.ftc.teamcode.hardwareSystems.MecanumWheels) {
                telemetry.addData("Front left wheel power",  frontLeftPower);
                telemetry.addData("Front right wheel power", frontRightPower);
                telemetry.addData("Back left wheel power",   backLeftPower);
                telemetry.addData("Back right wheel power",  backRightPower);
            }
        }

        /* Webcam controls */
        // Save CPU resources; can resume streaming when needed.  When the
        // driver presses D‑pad down, stop streaming; D‑pad up resumes
        // streaming.  Check that WEBCAM is not null before calling its
        // methods.
        if (gamepad1.dpad_down) {
            if (WEBCAM != null) {
                WEBCAM.getVisionPortal().stopStreaming();
            }
        } else if (gamepad1.dpad_up) {
            if (WEBCAM != null) {
                WEBCAM.getVisionPortal().resumeStreaming();
            }
        }

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
            if (gamepad2.left_trigger > 0.05) {
                intakeMotor.setPower(1.0);
            } else if (gamepad2.right_trigger > 0.05) {
                intakeMotor.setPower(-1.0);
            } else {
                intakeMotor.setPower(0.0);
            }
        }

        telemetry.update();
    }

    @Override
    public void runOpMode() {
        super.runOpMode();

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

        /*
        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName()
        );
        WEBCAM.getVisionPortal().stopStreaming();
         */

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