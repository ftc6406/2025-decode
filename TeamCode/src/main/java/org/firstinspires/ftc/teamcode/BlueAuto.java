package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.hardwareSystems.LimiterServo;
import org.firstinspires.ftc.teamcode.hardwareSystems.mousemotion.src.inputanalysis.*;
import org.firstinspires.ftc.teamcode.hardwareSystems.mousemotion.src.eventclassification.eventcodes.*;
import org.firstinspires.ftc.teamcode.hardwareSystems.mousemotion.src.eventclassification.*;
import org.firstinspires.ftc.teamcode.hardwareSystems.mousemotion.src.devicemanagement.*;
import org.firstinspires.ftc.teamcode.hardwareSystems.mousemotion.src.inputanalysis.MouseMotionTracker;

import org.firstinspires.ftc.teamcode.hardwareSystems.MecanumWheels;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.ArrayList;
import java.util.HashMap;

@Autonomous(name = "BlueAuto")
public class BlueAuto extends CustomLinearOp {


    // Shooter / launcher motor for firing into the goal.
    private DcMotorEx launcherMotor;


    // Intake motor that feeds pre-loaded balls into the launcher.
    private DcMotorEx intakeMotor;

    // ----- Limiter servo (shared between TeleOp and Auto) -----
    /**
     * Wrapper around the limiter servo hardware.
     * Uses two preset positions: HOME and OUT (≈90 degrees).
     */
    private LimiterServo limiterServo;

    // Positions for the limiter servo.
    // You will tune these on the real robot so that OUT is about 90 degrees
    // and HOME is the original position.
    private static final double LIMITER_HOME_POS  = 0.25; // 25 degrees
    private static final double LIMITER_OUT_POS   = 0.10; // 10 degrees


    // Tracker for mouse
    private MouseMotionTracker mouseTracker;

    // Thread to run mouse tracker
    private Thread mouseThread;
    // *** robot drives "forward" with a NEGATIVE distance ***
    // So we bake the sign in here.
    private static final double FORWARD_TO_LAUNCH_ZONE_INCHES = -15.0; // start value, tune on field


    // How long to run intake for each shot (ms)
    private static final long INTAKE_FEED_TIME_MS  = 2500; // 5 seconds


    // How long to pause between shots (ms)
    private static final long INTAKE_PAUSE_TIME_MS = 5000; // 5 seconds

    // width of robot
    private static final double ROBOT_WIDTH = 16.2;

    private static final double ROBOT_LENGTH = 17.2;

    @Override
    public void runOpMode() {
        // This sets up WHEELS and waits for START.
        super.runOpMode();

        ArrayList<InputDevice> devices = KernalInputDevices.getDevices();

        HashMap<EventTypes, EventCode[]> fullCapabilitiesFilter = new HashMap<>();
        EventCode[] filter = {Rel.REL_X, Rel.REL_Y};
        EventCode[] eventCodeFilterMsc = null;

        fullCapabilitiesFilter.put(EventTypes.REL, filter);
        fullCapabilitiesFilter.put(EventTypes.MSC, eventCodeFilterMsc);

        ArrayList<InputDevice> filteredDeviceList = KernalInputDevices.getDevices(fullCapabilitiesFilter);

        telemetry.addLine("Filtered device list size: " + filteredDeviceList.size());
        if (!filteredDeviceList.isEmpty()) {
//                telemetry.addLine(filteredDeviceList.get(0).getName());
//                mouseTracker = new MouseMotionTracker(new Mouse(filteredDeviceList.get(0), 1000));
//
//                mouseThread = new Thread(mouseTracker);
//                mouseThread.start();

        } else {
            telemetry.addLine("WARNING: MOUSE CANNOT BE FOUND");

        }


        try {
            // Create the limiter servo wrapper using the shared hardware class.
            // "limiterServo" must match the name in the RC config.
            limiterServo = new LimiterServo(hardwareMap,
                    "limiterServo",
                    LIMITER_HOME_POS,
                    LIMITER_OUT_POS);


            telemetry.addLine("Limiter servo initialised");

        } catch (Exception e) {
            telemetry.addLine("WARNING: Limiter servo not found.\n" + e.getMessage());

        }

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
        telemetry.addLine("Test line 1");

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

        // X  -> move limiter OUT (≈90 degrees)
        // Y  -> move limiter back HOME.
        if (limiterServo != null) {
            limiterServo.setOut();

//            // When driver presses Y on gamepad 1, retract the limiter.
//            if (gamepad1.y) {
//                limiterServo.setHome();
//            }


//            // Telemetry so you can confirm the code is actually running.
//            telemetry.addData("LimiterPos", "%.2f", limiterServo.getPosition());
//            telemetry.addData("LimiterG1.X", gamepad1.x);
//            telemetry.addData("LimiterG1.Y", gamepad1.y);
        }


        if (WHEELS != null) {
            // Access the individual motors and set their powers to zero.
            org.firstinspires.ftc.teamcode.hardwareSystems.MecanumWheels mech =
                    (org.firstinspires.ftc.teamcode.hardwareSystems.MecanumWheels) WHEELS;

//            if (mouseTracker != null) {
//                // Strafe leftwards to the first ball row
//                // motionData is in SI units (i.e. displacement is in meters)
//                while (mouseTracker.getMotionData()[0][0] * 39.3701 <= (60  - (ROBOT_WIDTH / 2))) {
//                    mech.getFrontLeftMotor().setPower(-1);
//                    mech.getFrontRightMotor().setPower(1);
//                    mech.getBackLeftMotor().setPower(1);
//                    mech.getBackRightMotor().setPower(-1);
//                }
//
//                // consume balls
//                if (intakeMotor != null) {
//                    intakeMotor.setPower(-1);
//
//                }
//
//                // Drive forward into balls
//                while (mouseTracker.getMotionData()[0][1] * 39.3701 >= (-24 - (ROBOT_LENGTH / 2))) {
//                    mech.getFrontLeftMotor().setPower(1);
//                    mech.getFrontRightMotor().setPower(1);
//                    mech.getBackLeftMotor().setPower(1);
//                    mech.getBackRightMotor().setPower(1);
//
//                }
//
//                if (intakeMotor != null) {
//                    intakeMotor.setPower(0);
//
//                }
//
//                if (launcherMotor != null) {
//                    launcherMotor.setPower(0.8);
//
//                }
//
//                // Drive backwards
//                while (mouseTracker.getMotionData()[0][1] * 39.3701 <= 0) {
//                    mech.getFrontLeftMotor().setPower(-1);
//                    mech.getFrontRightMotor().setPower(-1);
//                    mech.getBackLeftMotor().setPower(-1);
//                    mech.getBackRightMotor().setPower(-1);
//
//                }
//
//                // Drive to shooting line rightwards
//                while (mouseTracker.getMotionData()[0][1] * 39.3701 >= (36 - (ROBOT_WIDTH / 2))) {
//                    mech.getFrontLeftMotor().setPower(1);
//                    mech.getFrontRightMotor().setPower(-1);
//                    mech.getBackLeftMotor().setPower(-1);
//                    mech.getBackRightMotor().setPower(1);
//
//                }

                emptyMag();

//            }


//            sleep(5000);
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
         // Thread termination testing
         mouseTracker.terminate();

         try {
             // Time bound termination; can be adjusted as needed
             mouseThread.join(500);

         } catch (InterruptedException e) {
             e.printStackTrace();
         }


        if (intakeMotor != null) {
            intakeMotor.setPower(0.0);
        }

        if (launcherMotor != null) {
            launcherMotor.setPower(0.0);
        }


        telemetry.addLine("Autonomous complete");
        telemetry.update();
    }

    public void emptyMag() {
        limiterServo.setHome();

        for (int shot = 1; shot <= 3 && opModeIsActive(); shot++) {
            telemetry.addData("Shot", shot);
            telemetry.addLine("Running intake to feed ball");
            telemetry.update();


            if (intakeMotor != null) {
                // If this direction pulls balls IN instead of shooting,
                // change to intakeMotor.setPower(-1.0);
                intakeMotor.setPower(-0.8);
            }


            sleep(INTAKE_FEED_TIME_MS);


            if (intakeMotor != null) {
                intakeMotor.setPower(0.0);
            }


            if (shot < 3) {
                telemetry.addLine("Pause between shots");
                telemetry.update();
                sleep(INTAKE_PAUSE_TIME_MS);  // 5 seconds pause
            }
        }

        limiterServo.setOut();

    }
}