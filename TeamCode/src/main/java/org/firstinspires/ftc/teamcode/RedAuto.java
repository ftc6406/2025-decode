package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;


@Autonomous(name = "RedAuto")
public class RedAuto extends CustomLinearOp {


    // Shooter / launcher motor for firing into the goal.
    private DcMotorEx launcherMotor;


    // Intake motor that feeds pre-loaded balls into the launcher.
    private DcMotorEx intakeMotor;


    // *** robot drives "forward" with a NEGATIVE distance ***
    // So we bake the sign in here.
    private static final double FORWARD_TO_LAUNCH_ZONE_INCHES = -15.0; // start value, tune on field


    // How long to run intake for each shot (ms)
    private static final long INTAKE_FEED_TIME_MS  = 1000; // 5 seconds


    // How long to pause between shots (ms)
    private static final long INTAKE_PAUSE_TIME_MS = 5000; // 5 seconds


    @Override
    public void runOpMode() {


        // This sets up WHEELS and waits for START.
        super.runOpMode();


        // === Map shooter and intake motors (same as TeleOp) ===
        try {
            launcherMotor = hardwareMap.get(DcMotorEx.class, "launcherMotor");
            launcherMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            launcherMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            launcherMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            launcherMotor.setPower(0.0);
            telemetry.addLine("Launcher motor initialised (auto)");
        } catch (Exception e) {
            telemetry.addLine("WARNING: launcherMotor not found in Auto");
        }

        try {
            intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
            intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            intakeMotor.setPower(0.0);
            telemetry.addLine("Intake motor initialised (auto)");
        } catch (Exception e) {
            telemetry.addLine("WARNING: intakeMotor not found in Auto");
        }

        if (isStopRequested()) return;

        // =========================================
        // STEP 2: SPIN UP LAUNCHER (FULL POWER)
        // =========================================


        if (launcherMotor != null) {
//            telemetry.addLine("Spinning launcher at full power");
            telemetry.addLine("Spinning launcher at 0.65 power");
            telemetry.update();
            launcherMotor.setPower(0.65);
        } else {
            telemetry.addLine("No launcherMotor – skipping shooting");
            telemetry.update();
        }

        sleep(5000);  // 5 seconds feed

        // =========================================
        // STEP 3: FIRE THREE BALLS USING INTAKE
        // =========================================


        for (int shot = 1; shot <= 3 && opModeIsActive(); shot++) {


            telemetry.addData("Shot", shot);
            telemetry.addLine("Running intake to feed ball");
            telemetry.update();


            if (intakeMotor != null) {
                // If this direction pulls balls IN instead of shooting,
                // change to intakeMotor.setPower(-1.0);
                intakeMotor.setPower(-0.8);
            }


            sleep(INTAKE_FEED_TIME_MS);  // 5 seconds feed


            if (intakeMotor != null) {
                intakeMotor.setPower(0.0);
            }


            if (shot < 3) {
                telemetry.addLine("Pause between shots");
                telemetry.update();
                sleep(INTAKE_PAUSE_TIME_MS);  // 5 seconds pause
            }
        }

        // ================
        // STEP 1: DRIVE IN
        // ================


        if (WHEELS != null) {
            // Access the individual motors and set their powers to zero.
            org.firstinspires.ftc.teamcode.hardwareSystems.MecanumWheels mech =
                    (org.firstinspires.ftc.teamcode.hardwareSystems.MecanumWheels) WHEELS;

            mech.getFrontLeftMotor().setPower(-1);
            mech.getFrontRightMotor().setPower(1);
            mech.getBackLeftMotor().setPower(1);
            mech.getBackRightMotor().setPower(-1);
            sleep(5000);
//            telemetry.addData("Step", "Driving into launch zone");
//            telemetry.addData("Forward (in)", FORWARD_TO_LAUNCH_ZONE_INCHES);
//            telemetry.update();
//
//
//            // No strafe, just forward/back
//            WHEELS.driveDistance(0.0, FORWARD_TO_LAUNCH_ZONE_INCHES);

        } else {
            telemetry.addLine("WARNING: WHEELS is null – cannot drive.");
            telemetry.update();

        }

        // ===========================
        // STEP 4: SHUT DOWN AND FINISH
        // ===========================


        if (intakeMotor != null) {
            intakeMotor.setPower(0.0);
        }

        if (launcherMotor != null) {
            launcherMotor.setPower(0.0);
        }


        telemetry.addLine("Autonomous complete");
        telemetry.update();
    }
}