package org.firstinspires.ftc.teamcode;


import android.os.Environment;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.BufferedReader;

/**
 * Holds alliance and side settings for autonomous and teleop.
 * The settings are persisted to /sdcard/FIRST/settings/auto_settings.txt.
 */
public final class AutoSettings {
    public enum AllianceColor { RED, BLUE }
    public enum TeamSide { NEAR, FAR }

    private static AllianceColor alliance = AllianceColor.RED;
    private static TeamSide teamSide     = TeamSide.NEAR;

    public static String getPositionFile() {
        File dir = new File(Environment.getExternalStorageDirectory(), "FIRST/settings");
        if (!dir.exists()) dir.mkdirs();
        return new File(dir, "auto_settings.txt").getAbsolutePath();
    }

    /** Read the saved alliance and side from the settings file. */
    public static void readFromFile() {
        File f = new File(getPositionFile());
        if (!f.exists()) return;
        try (BufferedReader br = new BufferedReader(new FileReader(f))) {
            String line = br.readLine();
            if (line != null) {
                String[] parts = line.trim().split("\\s*,\\s*");
                if (parts.length >= 2) {
                    try {
                        alliance = AllianceColor.valueOf(parts[0]);
                        teamSide = TeamSide.valueOf(parts[1]);
                    } catch (IllegalArgumentException ignore) {
                        // leave defaults on parse error
                    }
                }
            }
        } catch (Exception ignore) {
            // leave defaults on error
        }
    }


    /** Write the current alliance and side to the settings file. */
    public static void writeToFile() {
        try (FileWriter fw = new FileWriter(getPositionFile(), false)) {
            fw.write(alliance.name() + "," + teamSide.name());
        } catch (Exception ignore) {}
    }


    // Getters and setters
    public static AllianceColor getAlliance() {
        return alliance;
    }
    public static TeamSide getTeamSide() {
        return teamSide;
    }
    /** Set the alliance and side in memory (optionally write to file). */
    public static void set(AllianceColor ac, TeamSide ts, boolean save) {
        alliance = ac;
        teamSide = ts;
        if (save) {
            writeToFile();
        }
    }
}