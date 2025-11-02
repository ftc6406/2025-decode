package org.firstinspires.ftc.teamcode;

import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;

/**
 * Run to set the robot's alliance color and side before running autonomous.
 */
@TeleOp(name = "AutoConfig")
public class AutoConfig extends OpMode {

    /**
     * The directory that all the files are saved to.
     */
    private static final String DIRECTORY =
            Environment.getExternalStorageDirectory().getAbsolutePath() +
                    "/ftc/";

    /**
     * Name of the config file, which is inside the directory specified by
     * `DIRECTORY`.
     */
    private static final String CONFIG_FILE = DIRECTORY + "position.txt";

    public static String getPositionFile() {
        return CONFIG_FILE;
    }

    /**
     * Reads the current position stored in the external storage file and prints
     * it. Prints an error message if it fails.
     */
    private void printRobotPosition() {
        try (BufferedReader reader =
                     new BufferedReader(new FileReader(CONFIG_FILE))) {
            // Read first line.
            String data = reader.readLine();
            telemetry.addData("Current position", data);

        } catch (IOException e) {
            telemetry.addLine("ERROR: FAILED TO READ ROBOT POSITION FROM " +
                    "STORAGE FILE!");
        }
    }

    @Override
    public void init() {
        printRobotPosition();
    }

    @Override
    public void loop() {
        telemetry.update();
        printRobotPosition();

        // If the directory and file do not exist, create them.
        try {
            File directory = new File(DIRECTORY);
            if (!directory.mkdirs()) {
                throw new IOException("ftc/ couldn't be created.");
            }

            File file = new File(CONFIG_FILE);
            if (!file.createNewFile()) {
                throw new IOException("position.txt could not be created.");
            }

        } catch (IOException e) {
            telemetry.addLine(e.getMessage());
        }

        String positionString = null;
        if (gamepad1.y || gamepad2.y) {
            // Orange button
            positionString =
                    AllianceColor.RED.name() + "," + TeamSide.NEAR.name();

        } else if (gamepad1.b || gamepad2.b) {
            // Red button
            positionString =
                    AllianceColor.RED.name() + "," + TeamSide.FAR.name();

        } else if (gamepad1.a || gamepad2.a) {
            // Green button
            positionString =
                    AllianceColor.BLUE.name() + "," + TeamSide.NEAR.name();

        } else if (gamepad1.x || gamepad2.x) {
            // Blue button
            positionString =
                    AllianceColor.BLUE.name() + "," + TeamSide.FAR.name();
        }

        // Do nothing if the driver didn't press any buttons.
        if (positionString == null) {
            return;
        }

        // Write the string to the file.
        try (FileWriter writer = new FileWriter(CONFIG_FILE, false)) {
            writer.write(positionString);

        } catch (IOException e) {
            telemetry.addLine("ERROR: FAILED TO WRITE ROBOT POSITION TO " +
                    "STORAGE FILE!");
            telemetry.addLine(e.toString());
        }
    }
}
