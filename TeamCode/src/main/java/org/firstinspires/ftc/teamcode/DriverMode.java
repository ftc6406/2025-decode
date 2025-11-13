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
         * Drive robot based on joystick input from gamepad1
         * Right stick moves the robot forwards and backwards and turns it.
         * The triggers control strafing.  A positive left trigger causes
         * leftward strafe; a positive right trigger causes rightward strafe.
         */
        // Compute raw inputs from gamepad controls.  Right stick Y controls
        // forward/back (invert so up is positive); left stick X controls
        // turning; triggers control strafing.
        // Compute raw inputs from gamepad controls.  Right stick Y controls
        // forward/back; left stick X controls turning; triggers control
        // strafing.  Use the difference between right and left triggers
        // so that each trigger contributes in the proper direction: a
        // positive right trigger strafes right and a positive left trigger
        // strafes left.  Invert the forward axis so pushing up on the
        // joystick results in positive forward motion (the FTC SDK uses
        // negative Y for forward on the joystick).
        double rawForward = -gamepad1.right_stick_y;
        double rawTurn    = gamepad1.left_stick_x;
        double rawStrafe  = gamepad1.right_trigger - gamepad1.left_trigger;

        // Apply the deadband and sensitivity scaling.  This prevents
        // unintentional drift when sticks are near center.
        double strafe  = applyDeadband(rawStrafe)  * DRIVING_SENSITIVITY;
        double forward = applyDeadband(rawForward) * DRIVING_SENSITIVITY;
        double turn    = applyDeadband(rawTurn)    * DRIVING_SENSITIVITY;

        // Note: We no longer invert the forward value here because we
        // explicitly negated the raw value above.  Forward motion on
        // gamepad1.right_stick_y now correctly maps to positive forward
        // robot motion.

        // Stop the wheels completely if all inputs are within the deadband.
        if (strafe == 0.0 && forward == 0.0 && turn == 0.0) {
            if (WHEELS != null) {
                WHEELS.drive(0, 0, 0);
            } else if (MECANUM_DRIVE != null) {
                MECANUM_DRIVE.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
            }
        } else {
            // Drive using the Wheels subsystem if available; otherwise fall
            // back to the Road Runner drive.  The order of arguments for
            // WHEELS.drive() is (xPower, yPower, rotation).
            if (WHEELS != null) {
                WHEELS.drive(strafe, forward, turn);

                // Provide telemetry feedback of individual wheel powers when
                // using a MecanumWheels implementation.  This helps drivers
                // understand how their inputs translate to motor output.
                if (WHEELS instanceof org.firstinspires.ftc.teamcode.hardwareSystems.MecanumWheels) {
                    org.firstinspires.ftc.teamcode.hardwareSystems.MecanumWheels mech =
                            (org.firstinspires.ftc.teamcode.hardwareSystems.MecanumWheels) WHEELS;
                    telemetry.addData("Front left wheel power", mech.getFrontLeftMotor().getPower());
                    telemetry.addData("Front right wheel power", mech.getFrontRightMotor().getPower());
                    telemetry.addData("Back left wheel power", mech.getBackLeftMotor().getPower());
                    telemetry.addData("Back right wheel power", mech.getBackRightMotor().getPower());
                }
            } else if (MECANUM_DRIVE != null) {
                // Fallback: convert the same inputs into a PoseVelocity2d.  Road
                // Runner uses +y forward and +x right.  Since we invert the
                // forward component above to correct for motor orientation,
                // pass forward directly into the Y component here.
                PoseVelocity2d velocity = new PoseVelocity2d(
                        new Vector2d(strafe, forward),
                        turn
                );
                MECANUM_DRIVE.setDrivePowers(velocity);
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