package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "DriverMode")
public class DriverMode extends CustomLinearOp {
    // TODO: Replace the driving sensitivity with an appropriate level of sensitivity.
    /**
     * The sensitivity of the robot's driving joystick.
     */
    private static final double DRIVING_SENSITIVITY = 1.0;

    private static int cameraMonitorViewId;

    /**
     * the loop once.
     */
    private void runLoop() {
        /* Gamepad 1
         * Run(Wheel and Webcam Controls) */

        /* Wheel Controls */
        /*
         * Drive robot based on joystick input from gamepad1.
         */
        PoseVelocity2d velocity = new PoseVelocity2d(
                new Vector2d(gamepad1.left_stick_x, gamepad1.right_stick_y),
                0.0
        );
        MECANUM_DRIVE.setDrivePowers(velocity);

        /* Webcam controls */
        // Save CPU resources; can resume streaming when needed.
        /*
        if (gamepad1.dpad_down) {
            WEBCAM.getVisionPortal().stopStreaming();
        } else if (gamepad1.dpad_up) {
            WEBCAM.getVisionPortal().resumeStreaming();
        }
         */

        /* Gamepad 2 (Arm and Claw Controls) */

        telemetry.update();
    }

    @Override
    public void runOpMode() {
        super.runOpMode();

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
    }
}