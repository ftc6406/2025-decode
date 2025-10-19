package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "DriverMode")
public class DriverMode extends CustomLinearOp {
    // TODO: Replace the driving sensitivity with an appropriate level of
    //  sensitivity.
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
     * be a REV HD Hex motor (40:1, 150 RPM spur gearbox).  This field is
     * initialised in {@link #runOpMode()} using the hardware name
     * "intakeMotor".  If your intake motor uses a different name in
     * the Robot Controller configuration, update the call to
     * {@code hardwareMap.get()} accordingly.
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
        double rawStrafe  = (gamepad1.left_trigger > 0) ? -gamepad1.left_trigger : gamepad1.right_trigger;
        double rawForward = -gamepad1.right_stick_y;  // invert so pushing up moves forward
        double rawTurn    = gamepad1.left_stick_x;

        // Apply the deadband and sensitivity scaling.  This prevents
        // unintentional drift when sticks are near center.
        double strafe  = applyDeadband(rawStrafe)  * DRIVING_SENSITIVITY;
        double forward = applyDeadband(rawForward) * DRIVING_SENSITIVITY;
        double turn    = applyDeadband(rawTurn)    * DRIVING_SENSITIVITY;

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
                // Fallback: convert the same inputs into a PoseVelocity2d.  Note
                // that Road Runner uses +y forward and +x right.  We negate
                // forward to maintain consistency with the WHEELS.drive() call.
                PoseVelocity2d velocity = new PoseVelocity2d(
                        new Vector2d(strafe, -forward),
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

        /* Gamepad 2 (Additional controls)
         *
         * Gamepad2 is reserved for future subsystems.  Since the DECODE
         * challenge does not use an arm or intake claw, no functional code is
         * provided here.  Implement your own control logic for other
         * attachments as needed.
         */

        telemetry.update();
    }

    @Override
    public void runOpMode() {
        super.runOpMode();

        // Initialise the intake motor.  This motor is configured as a
        // REV HD Hex motor (40:1, 150 RPM) with an integrated encoder.  In
        // the Robot Controller (Control Hub) configuration, ensure you
        // have added a DC motor device named "intakeMotor" connected to
        // an unused motor port (e.g. motor port 0 on the Expansion Hub).
        // See "Hardware configuration" section below for details.
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

            // Set the intake motor to full power (100%).  This will run the
            // intake continuously while the teleop mode is active.
            intakeMotor.setPower(1.0);

            telemetry.addLine("Intake motor initialised and running at full power.");
        } catch (Exception e) {
            telemetry.addLine("WARNING: Intake motor not found.\n" + e.getMessage());
        }

        /*
        cameraMonitorViewId = hardwareMap.appContext.getResources()
        .getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext
                .getPackageName()
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