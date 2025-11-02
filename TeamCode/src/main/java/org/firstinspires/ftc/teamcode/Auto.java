package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto")
public class Auto extends CustomLinearOp {
    /**
     * Automatically runs after pressing the "Init" button on the Control Hub
     */
    @Override
    public void runOpMode() {
        super.runOpMode();

        new TrajectoryBuilder(new Pose2d(0, 0, 0))
                .forward(40)
                .build();

        MECANUM_DRIVE.actionBuilder(
                new Pose2d(0, 0, 0)
        );

        telemetry.update();
    }
}