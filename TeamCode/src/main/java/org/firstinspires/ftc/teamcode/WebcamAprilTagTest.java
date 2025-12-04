package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.hardwareSystems.Webcam;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

/**
 * Simple TeleOp to test a standard USB webcam for AprilTag detection.
 *
 * <p>This OpMode demonstrates how to instantiate the {@link Webcam}
 * helper, start streaming from a standard webcam, and read AprilTag
 * detections.  It prints the ID, range, bearing, yaw and raw pose
 * coordinates of each detected tag to the Driver Station telemetry.
 *
 * <p>Unlike the built‑in AprilTag sample that embeds all of the vision
 * logic directly inside an opmode, this test uses the reusable
 * {@link Webcam} class from the hardwareSystems package.  It
 * automatically configures a {@link VisionPortal} with an
 * {@link org.firstinspires.ftc.vision.apriltag.AprilTagProcessor}, and
 * exposes a simple API to retrieve detections.  No Road Runner or
 * drivetrain code is required for this test.
 *
 * <p>To use this test:
 * <ul>
 *   <li>Connect a standard USB webcam (e.g. Logitech C920 or Lenovo USB
 *       webcam) to the Control Hub.</li>
 *   <li>In the Robot Controller configuration on the Control/Driver Hub,
 *       add a camera device named <strong>"Webcam 1"</strong> (or modify
 *       the string passed to {@code hardwareMap.get()} below).</li>
 *   <li>Place one or more AprilTags within view of the camera.</li>
 *   <li>Select this opmode on the Driver Station and press Init/Start.</li>
 * </ul>
 *
 * The telemetry will show how many tags are detected as well as the
 * computed range (inches and metres), bearing and yaw (degrees).  This
 * information can be used to verify that your camera is mounted
 * correctly and that the AprilTag library is functioning as expected.
 */
@TeleOp(name = "WebcamAprilTagTest", group = "Test")
public class WebcamAprilTagTest extends LinearOpMode {
    /** Webcam wrapper used for AprilTag detection. */
    private Webcam webcam;

    @Override
    public void runOpMode() {
        // Choose a resolution supported by your webcam.  640×480 is
        // typically available on most USB webcams and provides good
        // detection range while keeping CPU usage reasonable.
        int[] resolution = {640, 480};

        // Offset of the camera relative to the robot centre (inches).  If
        // your webcam is mounted off‑centre, adjust these values to
        // obtain distances relative to the robot rather than the camera.
        double[] poseAdjust = {0.0, 0.0, 0.0};

        // Instantiate the helper.  Passing only a WebcamName and a
        // resolution uses the default VisionPortal behaviour (no custom
        // viewport).  Ensure the hardware name matches your RC config.
        webcam = new Webcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"),
                resolution,
                poseAdjust
        );

        telemetry.addLine("Webcam initialised. Waiting for start...");
        telemetry.update();
        waitForStart();

        // Main loop: read detections and display distance/angle info.
        while (opModeIsActive()) {
            List<AprilTagDetection> detections = webcam.getAprilTagDetections();
            telemetry.addData("#Tags", detections.size());

            // Loop through all detected tags.  If multiple tags are in view
            // simultaneously, each one will be reported separately.
            for (AprilTagDetection det : detections) {
                // Raw pose values relative to the camera (inches for X/Y/Z
                // and degrees for pitch/roll/yaw).  See FTC docs for
                // coordinate definitions: +X = right, +Y = forward, +Z = up.
                double x = det.ftcPose.x;
                double y = det.ftcPose.y;
                double z = det.ftcPose.z;
                double rangeIn = det.ftcPose.range;
                double bearingDeg = det.ftcPose.bearing;
                double yawDeg = det.ftcPose.yaw;

                // Convert range to metres for convenience (1 inch ≈ 0.0254 m).
                double rangeM = rangeIn * 0.0254;

                telemetry.addLine(String.format("ID %d:", det.id));
                telemetry.addLine(String.format(
                        "\u2022 Range: %.1f in (%.2f m)", rangeIn, rangeM));
                telemetry.addLine(String.format(
                        "\u2022 Bearing: %.1f°", bearingDeg));
                telemetry.addLine(String.format(
                        "\u2022 Yaw: %.1f°", yawDeg));
                telemetry.addLine(String.format(
                        "\u2022 Raw XYZ: x=%.1f in, y=%.1f in, z=%.1f in",
                        x, y, z));
            }
            telemetry.update();

            // Sleep briefly to allow other tasks (e.g. vision pipeline) to run.
            sleep(40);
        }
    }
}