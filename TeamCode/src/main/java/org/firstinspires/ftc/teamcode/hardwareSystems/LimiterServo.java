package org.firstinspires.ftc.teamcode.hardwareSystems;


import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


/**
 * Simple hardware wrapper for the limiter servo.
 * This uses a standard positional Servo (0.0 - 1.0 position).
 *
 * You can call setHome() and setOut() from both TeleOp and Auto.
 */
public class LimiterServo {


    // The underlying servo object from the hardware map.
    private final Servo limiter;


    // Saved positions for home (retracted) and out (≈90 degrees from home).
    private final double homePos;
    private final double outPos;


    /**
     * Create the limiter servo.
     *
     * @param hardwareMap robot hardware map (passed in from OpMode)
     * @param servoName   name of the servo in the RC configuration (e.g. "limiterServo")
     * @param homePos     home position (0.0 - 1.0)
     * @param outPos      out position (0.0 - 1.0), about 90 degrees from home
     */
    public LimiterServo(HardwareMap hardwareMap,
                        String servoName,
                        double homePos,
                        double outPos) {
        this.homePos = homePos;
        this.outPos  = outPos;


        // Look up the servo by name in the configuration.
        limiter = hardwareMap.get(Servo.class, servoName);


        // Start in the home position.
        limiter.setPosition(this.homePos);
    }


    /**
     * Move the limiter back to the home / original position.
     */
    public void setHome() {
        limiter.setPosition(homePos);
    }


    /**
     * Move the limiter out (about 90 degrees from home).
     */
    public void setOut() {
        limiter.setPosition(outPos);
    }


    /**
     * Read back the current position for telemetry or debugging.
     */
    public double getPosition() {
        return limiter.getPosition();
    }
}
