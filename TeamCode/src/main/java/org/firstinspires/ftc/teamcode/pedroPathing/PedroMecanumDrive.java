package org.firstinspires.ftc.teamcode.pedroPathing;


import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.teamcode.MecanumDrive;


/**
 * Pedro‑compatible drivetrain wrapper around your existing Road Runner MecanumDrive.
 *
 * This class forwards power commands to the four motors and delegates pose updates
 * to the Road Runner MecanumDrive’s updatePoseEstimate() method.
 */
public class PedroMecanumDrive {
    private final DcMotorEx leftFront, leftBack, rightFront, rightBack;
    private final IMU imu;
    private final MecanumDrive rrDrive;
    private Pose2d poseEstimate;


    public PedroMecanumDrive(HardwareMap hardwareMap) {
        // Map motors using the same names your project already uses
        leftFront  = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack   = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack  = hardwareMap.get(DcMotorEx.class, "rightBack");


        // Map the IMU (same name used in MecanumDrive)
        imu = hardwareMap.get(IMU.class, "imu");


        // Instantiate your existing MecanumDrive to reuse its localizer and feedforward logic.
        // Starting pose is (0,0,0); adjust if your robot starts elsewhere.
        rrDrive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        poseEstimate = new Pose2d(0, 0, 0);
    }


    /** Set mecanum wheel powers based on desired translation (x, y) and rotation. */
    public void setWeightedDrivePower(Vector2d translation, double rotation) {
        // Robot‑centric: +x strafe right, +y forward, +rotation counter‑clockwise
        double x  = translation.x;
        double y  = translation.y;
        double rx = rotation;

        // Standard mecanum mix (same as in Road Runner)
        double fl = y + x + rx;
        double bl = y - x + rx;
        double fr = y - x - rx;
        double br = y + x - rx;


        // Normalise so no power exceeds |1.0|
        double max = Math.max(1.0,
                Math.max(Math.max(Math.abs(fl), Math.abs(bl)),
                        Math.max(Math.abs(fr), Math.abs(br))));
        fl /= max; bl /= max; fr /= max; br /= max;


        leftFront.setPower(fl);
        leftBack.setPower(bl);
        rightFront.setPower(fr);
        rightBack.setPower(br);
    }

    public void setWeightedDrivePower(double x, double y, double rotation) {
        setWeightedDrivePower(new com.acmerobotics.roadrunner.Vector2d(x, y), rotation);
    }

    /** Get the current pose estimate. */
    public Pose2d getPoseEstimate() {
        return poseEstimate;
    }


    /** Set the pose estimate (useful before starting a path). */
    public void setPoseEstimate(Pose2d pose) {
        poseEstimate = pose;
        rrDrive.localizer.setPose(pose);
    }


    /** Update the pose estimate using Road Runner’s localizer. */
    public Pose2d update() {
        rrDrive.updatePoseEstimate();
        poseEstimate = rrDrive.localizer.getPose();
        return poseEstimate;
    }


    /** Stop the drive by setting all motor powers to zero. */
    public void stop() {
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }
}