package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;


@Autonomous(name = "Auto")
public class Auto extends CustomLinearOp {


    // Shooter / launcher motor for firing into the goal.
    private DcMotorEx launcherMotor;


    // Intake motor that feeds pre-loaded balls into the launcher.
    private DcMotorEx intakeMotor;


    // Distance (in inches) to drive forward from the starting tile
    // until we are just inside the LAUNCH ZONE.
    // Start with 48 in and tune on a real field if needed.
    private static final double FORWARD_TO_LAUNCH_ZONE_INCHES = -5.0;


    // How long to run the intake (in milliseconds) for each shot.
    private static final long INTAKE_FEED_TIME_MS = 5000;


    // How long to pause between shots (in milliseconds).
    private static final long INTAKE_PAUSE_TIME_MS = 5000;


    @Override
    public void runOpMode() {
        // Initialise all common hardware (wheels, webcam, alliance/side, etc.)
        // and wait for the driver to press START. After this returns, the
        // match clock is running.
        super.runOpMode();


        // Map the launcher and intake motors using the SAME hardware names
        // as TeleOp (DriverMode). Do not change these names unless you
        // also change the Robot Controller configuration.
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


        telemetry.addData("Auto alliance", ALLIANCE_COLOR);
        telemetry.addData("Auto side",    TEAM_SIDE);
        telemetry.addLine("Plan: drive into launch zone and fire 3 preloaded balls.");
        telemetry.update();


        if (isStopRequested()) {
            return;
        }


        // =============================
        // STEP 1: DRIVE INTO LAUNCH ZONE
        // =============================


        double forward = FORWARD_TO_LAUNCH_ZONE_INCHES;


        // In this codebase, NEAR starts facing toward the centre of the field
        // (positive forward). FAR starts facing the opposite direction, so we
        // flip the sign for FAR just like the previous Auto implementation.
        if (TEAM_SIDE == AutoSettings.TeamSide.FAR) {
            forward = -forward;
        }


        if (WHEELS != null) {
            // Drive straight forward; no strafe needed.
            WHEELS.driveDistance(0.0, forward);


            // Wait until the drive motors finish their RUN_TO_POSITION motion.
            autoSleep();
        }


        // =========================================
        // STEP 2: SPIN UP THE LAUNCHER (FULL POWER)
        // =========================================


        if (launcherMotor != null) {
            // Run the launcher at full power for the entire autonomous.
            launcherMotor.setPower(1.0);
        }


        // =========================================
        // STEP 3: FIRE THREE BALLS USING THE INTAKE
        // =========================================


        for (int shot = 1; shot <= 3 && opModeIsActive(); shot++) {


            // 3a) Run intake to feed a ball into the spinning launcher.
            telemetry.addData("Auto", "Firing ball %d", shot);
            telemetry.update();


            if (intakeMotor != null) {
                // Positive power should match whatever direction you use
                // in TeleOp to feed balls into the shooter. If the robot
                // sucks balls in instead of feeding them out, swap the sign.
                intakeMotor.setPower(1.0);
            }


            sleep(INTAKE_FEED_TIME_MS);


            // 3b) Stop the intake after feeding this ball.
            if (intakeMotor != null) {
                intakeMotor.setPower(0.0);
            }


            // 3c) Pause before the next shot (skip pause after the last one).
            if (shot < 3) {
                telemetry.addData("Auto", "Pausing between shots...");
                telemetry.update();
                sleep(INTAKE_PAUSE_TIME_MS);
            }
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