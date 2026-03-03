package org.firstinspires.ftc.teamcode.pedroPathing;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;


/**
 * True Pedro auto: follows Visualizer paths (Path1 -> Path2 -> Path3)
 */
@Autonomous(name = "PedroAutoPaths", group = "Autonomous")
public class PedroAutoPaths extends OpMode {


    private Follower follower;
    private Paths paths;


    private enum State { PATH1, PATH2, PATH3, DONE }
    private State state = State.PATH1;


    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);


        // Set start pose (Pedro pose, not RR)
        Pose start = Paths.startPosePedro();
        follower.setStartingPose(start);


        // Build path chains using the follower
        paths = new Paths(follower);
    }


    @Override
    public void start() {
        // Start first path
        follower.followPath(paths.Path1);
    }


    @Override
    public void loop() {
        follower.update();


        // When follower finishes current path, move to next
        if (!follower.isBusy()) {
            switch (state) {
                case PATH1:
                    follower.followPath(paths.Path2);
                    state = State.PATH2;
                    break;


                case PATH2:
                    follower.followPath(paths.Path3);
                    state = State.PATH3;
                    break;


                case PATH3:
                    state = State.DONE;
                    break;


                case DONE:
                    // stop drive
                    follower.startTeleopDrive(true);
                    follower.setTeleOpDrive(0, 0, 0, true);
                    break;
            }
        }


        telemetry.addData("State", state);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }
}