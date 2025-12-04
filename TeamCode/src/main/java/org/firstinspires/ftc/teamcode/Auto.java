package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import java.util.List;

import org.firstinspires.ftc.teamcode.hardwareSystems.Webcam;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import org.firstinspires.ftc.teamcode.AutoSettings;
import static org.firstinspires.ftc.teamcode.AutoSettings.AllianceColor;
import static org.firstinspires.ftc.teamcode.AutoSettings.TeamSide;

@Autonomous(name = "Auto")
public class Auto extends CustomLinearOp {
    /**
     * Autonomous routine that uses AprilTag detections to choose a
     * trajectory.  This implementation leverages the {@link Webcam}
     * subsystem to identify a tag at the start of the match and then
     * commands the drivetrain to move to one of three parking zones.
     *
     * <p>Tags with IDs 1/20 select the left trajectory, IDs 2/21 select
     * the centre trajectory, and IDs 3/22 select the right trajectory.
     * If no tag is detected, the robot drives forward a shorter
     * distance as a safe fallback.  Distances are specified in inches
     * and can be tuned to suit your field layout.</p>
     */
    @Override
    public void runOpMode() {
        // Initialise all hardware (wheels, webcam, etc.) defined in
        // CustomLinearOp.  This also reads the alliance/side from the
        // auto settings file and sets up the VisionPortal.
        super.runOpMode();

        AutoSettings.readFromFile();
        applyAllianceToWebcam();
        AllianceColor ac = AutoSettings.getAlliance();
        TeamSide ts      = AutoSettings.getTeamSide();
        telemetry.addData("Alliance", ac);
        telemetry.addData("Team Side", ts);
        telemetry.update();

        AutoSettings.readFromFile();
        applyAllianceToWebcam();
        double strafe = 12.0;
        double forward = 24.0;
        if (AutoSettings.getAlliance() == AutoSettings.AllianceColor.BLUE) strafe = -strafe;
        if (AutoSettings.getTeamSide() == AutoSettings.TeamSide.FAR) forward = -forward;


        // Wait for the driver to press Start on the Driver Station.  At
        // this point the webcam is streaming and the AprilTag processor
        // is running in the background.
        waitForStart();

        // Attempt to identify an AprilTag for up to 2 seconds.  Continually
        // poll the AprilTag processor and update telemetry so drivers
        // know whether a tag has been seen.  If no tag is detected
        // within the timeout, fall back to a default path.
        long startTime = System.currentTimeMillis();
        int selectedTagId = -1;
        while (opModeIsActive() && selectedTagId == -1 &&
                System.currentTimeMillis() - startTime < 2000) {
            if (WEBCAM != null && WEBCAM.getAprilTag() != null) {
                List<org.firstinspires.ftc.vision.apriltag.AprilTagDetection> detections =
                        WEBCAM.getAprilTag().getDetections();
                telemetry.addData("Tags detected", detections.size());
                if (!detections.isEmpty()) {
                    selectedTagId = detections.get(0).id;
                    telemetry.addData("Selected tag", selectedTagId);
                    break;
                }
            }
            telemetry.addLine("Scanning for tag...");
            telemetry.update();
            sleep(50);
        }

        // Determine which path to take based on the tag ID and the
        // alliance/side selected via AutoConfig.  We map both the
        // current season (tags 1–3) and legacy Decode season (20–22)
        // identifiers to three possible trajectories.  Distances are in
        // inches and can be tuned for your specific robot and field.
        // Near side: use positive forward values; Far side: reverse the
        // forward component.  Alliance colour determines whether we
        // mirror the strafe direction (blue side is mirrored).
        boolean isNear = TEAM_SIDE == TeamSide.NEAR;
        boolean isRed = ALLIANCE_COLOR == AllianceColor.RED;

        // Distance definitions.  Adjust these values to suit your
        // starting position.  A positive strafe value moves to the
        // robot's right; a negative value moves left.  A positive
        // forward value moves forward on the field; negative is
        // backward (used for far side).  Blue alliance mirrors the
        // strafe direction.
        double leftStrafe  = -12.0;
        double centerStrafe = 0.0;
        double rightStrafe = 12.0;
        double forwardDist = 24.0;

        // Mirror strafe for blue alliance so that tag 1/20 still
        // corresponds to the left side of the field.
        if (!isRed) {
            leftStrafe = -leftStrafe;
            rightStrafe = -rightStrafe;
        }
        // Reverse forward distance for far side to drive away from
        // alliance wall instead of toward it.  Typically the far
        // starting position faces the centre of the field.
        if (!isNear) {
            forwardDist = -forwardDist;
        }

        double strafeDist = 12.0;
        if (ac == AllianceColor.BLUE) strafeDist = -strafeDist;
        if (ts == TeamSide.FAR) forwardDist = -forwardDist;

        // Execute the selected path.  Use driveDistance for simple
        // movements.  If Road Runner is configured (MECANUM_DRIVE !=
        // null), you can replace these calls with trajectories.
        if (selectedTagId == 1 || selectedTagId == 20) {
            // Left tag: strafe left then forward.
            if (WHEELS != null) {
                WHEELS.driveDistance(leftStrafe, forwardDist);
                autoSleep();
            }
        } else if (selectedTagId == 2 || selectedTagId == 21) {
            // Centre tag: drive forward only.
            if (WHEELS != null) {
                WHEELS.driveDistance(centerStrafe, forwardDist);
                autoSleep();
            }
        } else if (selectedTagId == 3 || selectedTagId == 22) {
            // Right tag: strafe right then forward.
            if (WHEELS != null) {
                WHEELS.driveDistance(rightStrafe, forwardDist);
                autoSleep();
            }
        } else {
            // Fallback: drive a shorter distance.  Use half of the
            // forward distance to position the robot safely.
            if (WHEELS != null) {
                WHEELS.driveDistance(0.0, forwardDist / 2.0);
                autoSleep();
            }
        }

        telemetry.addLine("Autonomous complete");
        telemetry.update();
    }
}