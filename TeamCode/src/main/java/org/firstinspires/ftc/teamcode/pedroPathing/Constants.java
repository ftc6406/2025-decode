package org.firstinspires.ftc.teamcode.pedroPathing;


import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.paths.PathConstraints;
import com.pedropathing.ftc.drivetrains.MecanumConstants;   // <-- If this import fails, use Alt/Option+Enter to auto-import
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;


/**
 * Pedro Constants (for Pedro 2.0.6)
 * This matches the Pedro docs style and makes your drivetrain + tuning work correctly.
 */
public class Constants {


    // Pedro follower constants object
    public static final FollowerConstants followerConstants = new FollowerConstants();


    // Constraints (start conservative; tune later)
    public static final PathConstraints pathConstraints = new PathConstraints(
            0.99,  // maxPower (0..1)
            100,   // maxVel (Pedro units; tune later)
            1,     // maxAccel
            1      // maxJerk
    );


    /**
     * Mecanum drivetrain constants (THIS is what you were missing)
     * Use YOUR motor names from your config / RR drive:
     * leftFront, leftBack, rightFront, rightBack
     */
    public static final MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("rightFront")
            .rightRearMotorName("rightBack")
            .leftRearMotorName("leftBack")
            .leftFrontMotorName("leftFront")
            // Typical FTC mecanum: reverse the left side (adjust if your robot is opposite)
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);


    /**
     * This is what Tuning.java calls.
     * Now it also sets the mecanum drivetrain so Pedro can drive using encoders/IMU properly.
     */
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}