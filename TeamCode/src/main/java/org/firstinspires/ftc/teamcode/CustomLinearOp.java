package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.AutoSettings;
import static org.firstinspires.ftc.teamcode.AutoSettings.AllianceColor;
import static org.firstinspires.ftc.teamcode.AutoSettings.TeamSide;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
/*
 * Import statements for Arm and Claw have been removed because the
 * DECODE challenge does not use an articulated arm or intake claw.  If
 * you later reintroduce these subsystems, import the appropriate
 * classes here and add back their fields and initialisation methods.
 */
import org.firstinspires.ftc.teamcode.hardwareSystems.Webcam;
import org.firstinspires.ftc.teamcode.hardwareSystems.Wheels;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.Arrays;
import java.util.HashSet;
import java.util.Map;

import org.firstinspires.ftc.teamcode.hardwareSystems.Webcam;

public class CustomLinearOp extends LinearOpMode {
    /**
     * Whether the robot will automatically sleep after each command.
     */
    protected boolean autoSleepEnabled;

    /* Robot systems */

    protected Wheels WHEELS;
    /*
     *  TODO: For default purposes, the class is set to MecanumDrive.
     *      Replace as necessary
     */
    protected MecanumDrive MECANUM_DRIVE;

    /*
     * Arm and Claw subsystems are unused in this season's game.  The
     * corresponding fields have been removed to avoid compile errors
     * when those classes are deleted from the project.  If you add
     * arm/claw subsystems in a future season, declare them here and
     * initialise them appropriately.
     */
    // protected Arm ARM;
    // protected Claw CLAW;
    protected Webcam WEBCAM;

    /**
     * Store which alliance the robot is on.
     */
    protected AllianceColor ALLIANCE_COLOR;
    /**
     * Store which side the robot is on(i.e. far or near).
     */
    protected TeamSide TEAM_SIDE;



    /**
     * Apply the currently selected alliance to the webcam’s color target.
     * Called in both DriverMode and Auto after AutoSettings.readFromFile().
     */
    protected void applyAllianceToWebcam() {
        if (WEBCAM == null) {
            return;     // no camera configured
        }

        AutoSettings.AllianceColor ac = AutoSettings.getAlliance();

        // Map alliance to webcam color enum
        if (ac == AutoSettings.AllianceColor.RED) {
            WEBCAM.setTargetColor(org.firstinspires.ftc.teamcode.hardwareSystems.Webcam.Color.RED);
        } else if (ac == AutoSettings.AllianceColor.BLUE) {
            WEBCAM.setTargetColor(org.firstinspires.ftc.teamcode.hardwareSystems.Webcam.Color.BLUE);
        }
    }

    public HashSet<DcMotor> getAllDcMotors() {
        HashSet<DcMotor> motors = new HashSet<>();
        // hardware.dcMotor stores all the DcMotors as name-device pairs.
        for (Map.Entry<String, DcMotor> ele : hardwareMap.dcMotor.entrySet()) {
            motors.add(ele.getValue());
        }

        return motors;
    }

    /**
     * Gets all CR servos if they are present.
     *
     * @return A HashSet containing all the CR servos used by this robot.
     */
    public HashSet<CRServo> getAllCrServos() {
        HashSet<CRServo> crServos = new HashSet<>();
        // `hardwareMap.crservo` stores all the CRServos as name-device pairs.
        for (Map.Entry<String, CRServo> hardwareDevice : hardwareMap.crservo.entrySet()) {
            crServos.add(hardwareDevice.getValue());
        }

        return crServos;
    }

    /**
     * Get all the names in the `HardwareMap` that that are not connected to a device.
     * <br>
     * TODO: <em><strong>THIS METHOD IS NOT WORKING CURRENTLY!!!</strong></em>
     *
     * @return A `HashSet` of all the hardware devices that can not be found.
     */
    public HashSet<String> getMissingHardwareDevices() {
        HashSet<String> missingHardwareDevices = new HashSet<>();

        // Loop through each `DeviceMapping`(e.g. `Servo`s and `DcMotor`s).
        for (HardwareMap.DeviceMapping<? extends HardwareDevice> deviceMapping : hardwareMap.allDeviceMappings) {
            // Check if each device in the mapping is null.
            for (Map.Entry<String, ? extends HardwareDevice> hardwareDevice : deviceMapping.entrySet()) {
                if (hardwareDevice.getValue() == null) {
                    missingHardwareDevices.add(hardwareDevice.getKey());
                }
            }
        }

        return missingHardwareDevices;
    }

    /**
     * Try to retrieve a {@link DcMotor} from the hardware map using one of
     * several candidate names.  This helper makes the drive train code
     * resilient to different naming conventions in the Robot Controller
     * configuration.  It iterates through the provided names and
     * returns the first motor that exists.  If none of the names are
     * present, an IllegalArgumentException is thrown.
     *
     * @param candidates one or more hardware device names to try
     * @return the {@link DcMotor} associated with the first name found
     * @throws IllegalArgumentException if no candidate names match a motor
     */
    private DcMotor pickMotor(String... candidates) {
        for (String name : candidates) {
            try {
                return hardwareMap.get(DcMotor.class, name);
            } catch (IllegalArgumentException e) {
                // Continue to next candidate
            }
        }
        throw new IllegalArgumentException("Unable to find a hardware device with names "
                + java.util.Arrays.toString(candidates));
    }

    /**
     * Sleeps the robot while any motors are running.
     */
    public void autoSleep() {
        autoSleep(getAllDcMotors());
    }

    /**
     * Sleeps the robot while the given motors are running.
     * @param motors The motors to wait for.
     */
    public void autoSleep(DcMotor... motors) {
        autoSleep(new HashSet<>(Arrays.asList(motors)));
    }

    /**
     * Sleeps the robot while the given motors are running.
     *
     * @param motors The motors that are running.
     */
    public void autoSleep(HashSet<DcMotor> motors) {
        // Sleep while any of the motors are still running.
        while (motors.stream().anyMatch(DcMotor::isBusy)) {
            sleep(1);
        }
    }

    /**
     * Initiates all hardware needed for the wheels.
     * <br>
     * <strong>When starting a new season, change the return type from `Wheels` to the desired return type.</strong>
     */
    private void initWheels() {
        // Prevent multiple instantiation.
        if (WHEELS != null) {
            return;
        }

        /*
         * Instantiate the wheels system.  For a mecanum drive robot we use the
         * MecanumWheels implementation.  The expected hardware names for the
         * four wheel motors are "frontLeftWheel", "frontRightWheel",
         * "backLeftWheel", and "backRightWheel".  Adjust these names to
         * match your robot configuration.  The wheel distances and ticks per
         * inch are approximate; tune them for your specific robot.
         */
        try {
            /*
             * Acquire each of the four drive motors.  To be tolerant of
             * different naming conventions in the Robot Controller config,
             * we attempt to fetch several candidate names for each motor.
             * Update the candidate lists if your team uses different
             * names (for example, "frontLeft", "lf", "leftFront", etc.).
             */
            DcMotor frontLeft = pickMotor(
                    "frontLeftWheel", "frontLeft", "lf", "leftFront"
            );
            DcMotor frontRight = pickMotor(
                    "frontRightWheel", "frontRight", "rf", "rightFront"
            );
            DcMotor backLeft = pickMotor(
                    "backLeftWheel", "backLeft", "lb", "leftBack"
            );
            DcMotor backRight = pickMotor(
                    "backRightWheel", "backRight", "rb", "rightBack"
            );

            // Group the motors into a MecanumWheels.MotorSet.  MotorSet is a
            // convenient container for passing all four motors into the
            // MecanumWheels constructor.
            org.firstinspires.ftc.teamcode.hardwareSystems.MecanumWheels.MotorSet motorSet =
                    new org.firstinspires.ftc.teamcode.hardwareSystems.MecanumWheels.MotorSet(
                            frontLeft, frontRight, backLeft, backRight
                    );

            // Approximate measurements from the CAD model (in inches).  The
            // wheel circumference is 4 inches in diameter multiplied by π.
            double wheelCircumference = 4.0 * Math.PI;
            double gearRatio = 1.0;
            double ticksPerInch = org.firstinspires.ftc.teamcode.hardwareSystems.MotorType.TETRIX_TORQUENADO.getTicksPerRotation()
                    * gearRatio / wheelCircumference;
            // Approximate distances between wheels.  Adjust as necessary if
            // your robot's chassis dimensions differ.
            org.firstinspires.ftc.teamcode.hardwareSystems.Wheels.WheelDistances wheelDistances =
                    new org.firstinspires.ftc.teamcode.hardwareSystems.Wheels.WheelDistances(
                            8.5,  // lateral distance (left‑to‑right)
                            14.5  // longitudinal distance (front‑to‑back)
                    );

            // Create a MecanumWheels instance.  This will automatically
            // configure motor directions and reset encoders.
            WHEELS = new org.firstinspires.ftc.teamcode.hardwareSystems.MecanumWheels(
                    motorSet, wheelDistances, ticksPerInch
            );

        } catch (Exception e) {
            // If any motor could not be found, report the error.  This keeps
            // the telemetry output informative and avoids a null pointer
            // exception later on.  Leave WHEELS as null to signal an
            // initialization failure.
            telemetry.addLine("ERROR: Failed to initialize MecanumWheels.\n" + e.getMessage());
        }

        /*
         * Assume the robot starts at (0, 0, 0) in the Road Runner field
         * coordinate frame.  If your autonomous program uses a different
         * starting pose, modify the pose here accordingly.
         */
        MECANUM_DRIVE = new MecanumDrive(hardwareMap, new Pose2d(0.0, 0.0, 0.0));
    }

    /*
     * initArm() and initClaw() methods have been removed.  They were used
     * previously to initialise stub arm and claw subsystems.  Since the
     * DECODE challenge does not include these subsystems, these methods
     * would never be called.  Removing them also avoids compilation
     * errors when the Arm and Claw classes are deleted from the project.
     */

    /**
     * Initiate the webcam.
     *
     * <p>
     * This method ignores the supplied {@code cameraMonitorViewId} and
     * always constructs the {@link Webcam} using the default EasyOpenCV
     * behaviour (i.e. no custom viewport container).  Passing a
     * non‑empty container ID into EasyOpenCV can lead to the exception
     * “Viewport container specified by user is not empty.”  By always
     * using the three‑argument {@link Webcam} constructor, we avoid that
     * error.
     * </p>
     *
     * @param cameraMonitorViewId an unused resource ID.  Kept for
     *                            compatibility with existing call sites.
     */
    public void initWebcam(int cameraMonitorViewId) {
        // Lenovo webcams typically support 640×480 resolution.  Use this
        // as a sensible default.  If your camera can benefit from
        // higher resolution, adjust the numbers here.
        int[] resolution = {640, 480};

        // Adjust the camera pose offsets in inches.  Positive x is to the
        // right, positive y is forward, and positive z is up.  Tweak
        // these values based on the physical mounting of your webcam.
        double[] poseAdjust = new double[]{
                0.0, // x offset (inches)
                0.0, // y offset (forward/back)
                0.0  // z offset (height)
        };

        // Initialise the webcam without specifying a viewport container.
        // The fourth parameter (view ID) is intentionally omitted to
        // prevent the OpenCvCameraException related to a non‑empty
        // container.  VisionPortal manages its own view.
        WEBCAM = new Webcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"),
                resolution,
                poseAdjust
        );
    }

    /**
     * Retrieve the contents of the Auto Settings file as a `String`,
     * or `null` if there is nothing to read.
     *
     * @param autoSettingsFile A String representing the file path to be read.
     * @return A String representation of the setting file's contents.
     */
    public String readAutoSettingsFile(String autoSettingsFile) {
        // Try to read the auto settings
        try (BufferedReader reader = new BufferedReader(new FileReader(autoSettingsFile))) {
            // Read first line.
            String data = reader.readLine();
            telemetry.addData("Starting position: ", data);

            return data;

        } catch (IOException | NullPointerException e) {
            telemetry.addLine(
                    (e instanceof IOException)
                            ? "ERROR: FAILED TO READ AUTO_SETTINGS FILE!"
                            : "The position file is blank."
            );
            telemetry.addLine("Defaulting to RED NEAR");

            return null;
        }
    }

    /**
     * Overloads {@link CustomLinearOp#readAutoSettingsFile(String)}.
     * {@code autoSettingsFile} defaults to {@link AutoSettings#getPositionFile()}.
     *
     * @see CustomLinearOp#readAutoSettingsFile(String)
     */
    /**
     * Convenience wrapper: read the auto-settings file using
     * the standard path from AutoSettings.
     */
    public String readAutoSettingsFile() {
        // Use the path string of the File object
        return readAutoSettingsFile(AutoSettings.getPositionFile());
    }


    /**
     * Run automatically after pressing "Init."
     * Initiate all the robot's hardware.
     * Wait until the driver presses "Start."
     */
    @Override
    public void runOpMode() {
        autoSleepEnabled = true;

        initWheels();
        /*
         * Arms and intake claws are not used for the DECODE challenge.  The
         * following initialisation methods would normally create stub
         * subsystems, but are intentionally disabled to remove any arm or
         * claw hardware from this configuration.  Leaving them commented
         * prevents the unused placeholders from being instantiated.
         */
        // initArm();
        // initClaw();

        /*
         * Get camera ID to stream.
         * TODO: Currently not working.
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName()
        );
        telemetry.addData("cameraMonitorViewId", cameraMonitorViewId);
        telemetry.update();
        initWebcam(cameraMonitorViewId);

        // Try to read the auto settings
        String autoSettings = readAutoSettingsFile(AutoSettings.getPositionFile());
        ALLIANCE_COLOR = autoSettings != null ? AllianceColor.valueOf(autoSettings.split(",")[0]) : AllianceColor.RED;
        TEAM_SIDE = autoSettings != null ? TeamSide.valueOf(autoSettings.split(",")[1]) : TeamSide.NEAR;

        // Set the camera color.
        /*
        switch (ALLIANCE_COLOR) {
            case RED:
                WEBCAM.setTargetColor(Webcam.Color.RED);
                break;

            case BLUE:
                WEBCAM.setTargetColor(Webcam.Color.BLUE);
                break;
        }
         */
        telemetry.addData("Starting position", ALLIANCE_COLOR.name() + ", " + TEAM_SIDE.name());

        waitForStart();
    }
}