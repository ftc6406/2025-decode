package org.firstinspires.ftc.teamcode;


import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;


/**
 * <p>
 * Central place to store where the auto settings file lives.
 * CustomLinearOp and AutoConfig both call getPositionFile().
 * </p>
 *
 * <p>
 * NOTE: We use AppUtil.getSettingsFile() so the file is created
 * under /sdcard/FIRST/settings/ on the Control/Driver Hub,
 * which is readable/writable by FTC apps.
 * </p>
 */
public final class AutoSettings {
    // File name only. Keep names consistent with existing code usage.
    private static final String POSITION_FILE_NAME = "auto_settings.txt";

    private AutoSettings() { /* no instances */ }

    /**
     * Absolute path to the settings file used by AutoConfig & CustomLinearOp.
     */
    public static String getPositionFile() {
        File f = AppUtil.getInstance().getSettingsFile(POSITION_FILE_NAME);

        // Make sure parent directory exists.
        File parent = f.getParentFile();
        if (parent != null && !parent.exists()) {
            // best-effort
            parent.mkdirs();
        }

        return f.getAbsolutePath();
    }
}