package org.firstinspires.ftc.teamcode.pedroPathing;


import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.CustomLinearOp;


/**
 * PedroAuto (Bridge Version)
 *
 * Keeps your Pedro file structure while avoiding version-specific Pedro follower API errors.
 * Uses your PedroMecanumDrive + RR localizer to run a PID-style autonomous.
 *
 * This is already much better than hard-coded timed movement because it continuously
 * corrects heading while driving.
 */
@Autonomous(name = "PedroAuto", group = "Autonomous")
public class PedroAuto extends CustomLinearOp {


    private PedroMecanumDrive drive;


    // Heading PID-style gains (start here, tune on field)
    private double headingKp = 0.02;
    private double headingKi = 0.0;
    private double headingKd = 0.001;


    private double headingIntegral = 0.0;
    private double lastHeadingError = 0.0;


    @Override
    public void runOpMode() {
        /*
         * CustomLinearOp handles your normal init and waitForStart().
         * After super.runOpMode() returns, the match has started (unless stopped).
         */
        super.runOpMode();


        if (isStopRequested()) return;


        // Create drivetrain wrapper
        drive = new PedroMecanumDrive(hardwareMap);


        // Use your Paths file (keeps structure you already made)
        drive.setPoseEstimate(Paths.startPoseRR());


        telemetry.addLine("PedroAuto (bridge mode) running...");
        telemetry.update();


        // =========================
        // Example auto sequence
        // =========================


        // Drive forward while holding heading 0 degrees
        driveRobotCentricWithHeadingHold(0.0, 0.30, 0.0, 1000);


        // Strafe right while holding heading 0
        driveRobotCentricWithHeadingHold(0.30, 0.0, 0.0, 800);


        // Turn to 90 degrees
        turnToHeadingDegrees(90.0, 2500);


        // Drive forward while holding 90 degrees
        driveRobotCentricWithHeadingHold(0.0, 0.25, 90.0, 900);


        // Turn back to 0 degrees
        turnToHeadingDegrees(0.0, 2500);


        drive.stop();


        telemetry.addLine("PedroAuto complete.");
        telemetry.update();


        sleep(1000);
    }


    /**
     * Drives robot-centric for a fixed time while correcting heading using PID-style control.
     *
     * @param x strafe power (-1..1), + right
     * @param y forward power (-1..1), + forward
     * @param targetHeadingDeg heading to hold in degrees
     * @param durationMs time to drive in milliseconds
     */
    private void driveRobotCentricWithHeadingHold(double x, double y, double targetHeadingDeg, long durationMs) {
        long startTime = System.currentTimeMillis();
        resetHeadingPid();


        while (opModeIsActive() && (System.currentTimeMillis() - startTime) < durationMs) {
            Pose2d pose = drive.update();


            double currentHeadingDeg = Math.toDegrees(pose.heading.toDouble());
            double turnCorrection = headingPidOutput(targetHeadingDeg, currentHeadingDeg);


            // keep turn correction from overpowering translation
            turnCorrection = clamp(turnCorrection, -0.40, 0.40);


            drive.setWeightedDrivePower(x, y, turnCorrection);


            telemetry.addData("Mode", "Drive + Heading Hold");
            telemetry.addData("Target Heading", targetHeadingDeg);
            telemetry.addData("Current Heading", currentHeadingDeg);
            telemetry.addData("Heading Error", angleWrapDegrees(targetHeadingDeg - currentHeadingDeg));
            telemetry.addData("Turn Correction", turnCorrection);
            telemetry.addData("x cmd", x);
            telemetry.addData("y cmd", y);
            telemetry.update();
        }


        drive.stop();
    }


    /**
     * Turns in place to a target heading.
     *
     * @param targetHeadingDeg target heading in degrees
     * @param timeoutMs max time allowed for turn
     */
    private void turnToHeadingDegrees(double targetHeadingDeg, long timeoutMs) {
        long startTime = System.currentTimeMillis();
        resetHeadingPid();


        while (opModeIsActive() && (System.currentTimeMillis() - startTime) < timeoutMs) {
            Pose2d pose = drive.update();
            double currentHeadingDeg = Math.toDegrees(pose.heading.toDouble());


            double error = angleWrapDegrees(targetHeadingDeg - currentHeadingDeg);
            double turnPower = headingPidOutput(targetHeadingDeg, currentHeadingDeg);


            turnPower = clamp(turnPower, -0.50, 0.50);


            if (Math.abs(error) < 2.0) {
                break;
            }


            drive.setWeightedDrivePower(0.0, 0.0, turnPower);


            telemetry.addData("Mode", "Turn To Heading");
            telemetry.addData("Target Heading", targetHeadingDeg);
            telemetry.addData("Current Heading", currentHeadingDeg);
            telemetry.addData("Heading Error", error);
            telemetry.addData("Turn Power", turnPower);
            telemetry.update();
        }


        drive.stop();
    }


    /**
     * PID-style heading output (P + I + D).
     */
    private double headingPidOutput(double targetHeadingDeg, double currentHeadingDeg) {
        double error = angleWrapDegrees(targetHeadingDeg - currentHeadingDeg);


        headingIntegral += error;
        headingIntegral = clamp(headingIntegral, -1000, 1000);


        double derivative = error - lastHeadingError;
        lastHeadingError = error;


        return (headingKp * error) + (headingKi * headingIntegral) + (headingKd * derivative);
    }


    private void resetHeadingPid() {
        headingIntegral = 0.0;
        lastHeadingError = 0.0;
    }


    /**
     * Wrap angle into [-180, 180] so robot turns shortest direction.
     */
    private double angleWrapDegrees(double degrees) {
        while (degrees > 180) degrees -= 360;
        while (degrees < -180) degrees += 360;
        return degrees;
    }


    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}