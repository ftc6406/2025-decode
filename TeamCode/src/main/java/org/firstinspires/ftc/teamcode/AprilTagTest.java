package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.hardwareSystems.MecanumWheels;
import org.firstinspires.ftc.teamcode.hardwareSystems.MotorType;
import org.firstinspires.ftc.teamcode.hardwareSystems.Webcam;
import org.firstinspires.ftc.teamcode.hardwareSystems.Wheels;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@TeleOp(name = "AprilTagTest", group = "Test")
public class AprilTagTest extends LinearOpMode {
    private MecanumWheels WHEELS;
    private Webcam WEBCAM;

    private void printAllAprilTags() {
        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : WEBCAM.getAprilTagDetections()) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s",
                        detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  " +
                                "(inch)", detection.ftcPose.x,
                        detection.ftcPose.y,
                        detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)"
                        , detection.ftcPose.pitch, detection.ftcPose.roll,
                        detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  " +
                                "(inch, deg, deg)", detection.ftcPose.range,
                        detection.ftcPose.bearing,
                        detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown",
                        detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   " +
                        "(pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop
    }

    private void driveToTargetAprilTag(int targetId) {
        // Step through the list of detections and find the target AprilTag
        for (AprilTagDetection detection : WEBCAM.getAprilTagDetections()) {
            if (detection.metadata != null && detection.metadata.id == targetId) {
                WHEELS.drive(1);

            } else {
                WHEELS.drive(0);
            }
        } // end for() loop
    }

    public void initWheels() {
        /*
         * Define wheels system hardware here.  To make the robot
         * configuration less brittle, try several common hardware names for
         * each motor.  Update the candidate lists to match your team’s
         * naming conventions.  If none of the candidate names are found,
         * an IllegalArgumentException will be thrown and caught below.
         */
        DcMotor frontLeft;
        DcMotor frontRight;
        DcMotor backLeft;
        DcMotor backRight;
        try {
            frontLeft = pickMotor("frontLeftWheel", "frontLeft", "lf",
                    "leftFront");
            frontRight = pickMotor("frontRightWheel", "frontRight", "rf",
                    "rightFront");
            backLeft = pickMotor("backLeftWheel", "backLeft", "lb", "leftBack");
            backRight = pickMotor("backRightWheel", "backRight", "rb",
                    "rightBack");

        } catch (IllegalArgumentException e) {
            telemetry.addLine("ERROR: Unable to find one or more drive " +
                    "motors. " + "Check that the motor names in " +
                    "initWheels() match" + " your robot configuration.");
            telemetry.addLine(e.getMessage());
            telemetry.update();
            return;
        }

        MecanumWheels.MotorSet motorSet =
                new MecanumWheels.MotorSet(
                        frontLeft,
                        frontRight,
                        backLeft,
                        backRight
                );

        // Approximately measured from the CAD model in inches
        double wheelCircumference = 4.0 * Math.PI;
        double gearRatio = 1.0;
        double ticksPerInch =
                MotorType.TETRIX_TORQUENADO.getTicksPerRotation() * gearRatio / wheelCircumference;
        // Approximately measured from CAD
        Wheels.WheelDistances wheelDistances = new Wheels.WheelDistances(
                8.5,
                14.5
        );

        WHEELS = new MecanumWheels(motorSet, wheelDistances, ticksPerInch);
    }

    /**
     * Helper method to fetch a DC motor using any of the provided names.
     * Iterates through the names and returns the first motor found. If none of
     * the names exist in the hardware map, throws an IllegalArgumentException.
     *
     * @param candidates possible hardware names for the motor
     * @return the corresponding DcMotor
     */
    private DcMotor pickMotor(String... candidates) {
        for (String name : candidates) {
            try {
                return hardwareMap.get(DcMotor.class, name);
            } catch (IllegalArgumentException e) {
                // Try next candidate
            }
        }
        throw new IllegalArgumentException("Unable to find a hardware device "
                + "with names " + java.util.Arrays.toString(candidates));
    }


    @Override
    public void runOpMode() {
        int[] resolution = {640, 480};

        initWheels();
        WEBCAM = new Webcam(hardwareMap.get(WebcamName.class, "Webcam 1"),
                resolution);

        waitForStart();

        int targetId = 16;

        while (opModeIsActive()) {
            List<AprilTagDetection> currentDetections =
                    WEBCAM.getAprilTag().getDetections();
            telemetry.addData("# AprilTags Detected", currentDetections.size());

            driveToTargetAprilTag(targetId);

            // Add "key" information to telemetry
            telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) " + "dist.");
            telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
            telemetry.addLine("RBE = Range, Bearing & Elevation");

            telemetry.update();
        }
    }
}