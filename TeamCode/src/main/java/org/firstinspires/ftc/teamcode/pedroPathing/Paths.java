package org.firstinspires.ftc.teamcode.pedroPathing;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierLine;

/*
 * IMPORTANT:
 * This file will NOT compile until you add the Pedro Quickstart classes
 * that define: Follower, PathChain, Pose, BezierLine, etc.
 *
 * Fix #1: top-level class cannot be "static"
 */


public class Paths {
    // Start pose for PEDRO follower (uses Pedro Pose class)
    public static Pose startPosePedro() {
        // Update these to match your real start position on field
        return new Pose(56.000, 8.000, Math.toRadians(90));
    }


    // Start pose for your PID bridge auto (uses Road Runner Pose2d)
    public static com.acmerobotics.roadrunner.Pose2d startPoseRR() {
        return new com.acmerobotics.roadrunner.Pose2d(0, 0, 0);
    }

    public PathChain Path1;
    public PathChain Path2;
    public PathChain Path3;


    public Paths(Follower follower) {


        Path1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(56.000, 8.000),
                        new Pose(56.000, 36.000)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                .build();


        Path2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(56.000, 36.000),
                        new Pose(18.067, 35.738)
                ))
                .setTangentHeadingInterpolation()
                .build();


        Path3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(18.067, 35.738),
                        new Pose(115.394, 116.228)
                ))
                .setTangentHeadingInterpolation()
                .build();
    }
}
