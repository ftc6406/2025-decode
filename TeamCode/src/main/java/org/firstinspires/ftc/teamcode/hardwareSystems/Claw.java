package org.firstinspires.ftc.teamcode.hardwareSystems;

import com.qualcomm.robotcore.hardware.Servo;

import java.util.HashSet;

public abstract class Claw {
    private final HashSet<Servo> SERVOS;
    /**
     * How much to gradually move the servo.
     */
    private double servoIncrement;
    /**
     * The servo that rotates the claw about the X-axis(roll).
     */
    protected final Servo ROLL_SERVO;
    /**
     * The servo that rotates the claw about the Y-axis(pitch).
     */
    protected final Servo PITCH_SERVO;
    /**
     * The servo that rotates the claw about the Z-axis(yaw).
     */
    protected final Servo YAW_SERVO;

    public Claw(Servo rollServo, Servo pitchServo, Servo yawServo) {
        this(rollServo, pitchServo, yawServo, 0.1);
    }

    public Claw(Servo rollServo, Servo pitchServo, Servo yawServo, double servoIncrement) {
        SERVOS = new HashSet<>();
        SERVOS.add(rollServo);
        SERVOS.add(pitchServo);
        SERVOS.add(yawServo);

        ROLL_SERVO = rollServo;
        PITCH_SERVO = pitchServo;
        YAW_SERVO = yawServo;

        this.servoIncrement = servoIncrement;
    }

    /**
     * Get all the {@code Servo}s that are included in this arm system.
     *
     * @return A {@code HashSet} that contains every Servo included in this arm system.
     */
    public HashSet<Servo> getServos() {
        return SERVOS;
    }

    public double getServoIncrement() {
        return servoIncrement;
    }

    public void setServoIncrement(double servoIncrement) {
        this.servoIncrement = servoIncrement;
    }

    /**
     * Rotate the X-axis(roll) servo in a certain direction by `servoIncrement`.
     *
     * @param direction The direction to rotate the servo in.
     *                  Positive values rotate it clockwise, negative values rotate it counterclockwise.
     */
    public void rotateRollServo(double direction) {
        double targetPosition = ROLL_SERVO.getPosition() + Math.signum(direction) * servoIncrement;
        ROLL_SERVO.setPosition(targetPosition);
    }

    /**
     * Rotate the roll servo to a position specified in degrees.
     * @param degrees The target angle of the roll servo in degrees.
     */
    public void rotateRollServoToAngle(double degrees) {
        ROLL_SERVO.setPosition(degrees / 360.0);
    }

    /**
     * Rotate the Y-axis(pitch) servo in a certain direction by `servoIncrement`.
     *
     * @param direction The direction to rotate the servo in.
     *                  Positive values rotate it clockwise, negative values rotate it counterclockwise.
     */
    public void rotatePitchAxisServo(double direction) {
        double targetPosition = PITCH_SERVO.getPosition() + Math.signum(direction) * servoIncrement;
        PITCH_SERVO.setPosition(targetPosition);
    }

    /**
     * Rotate the pitch servo to a position specified in degrees.
     * @param degrees The target angle of the pitch servo in degrees.
     */
    public void rotatePitchServoToAngle(double degrees) {
        PITCH_SERVO.setPosition(degrees / 360.0);
    }

    /**
     * Rotate the Z-axis(yaw) servo in a certain direction by `servoIncrement`.
     *
     * @param direction The direction to rotate the servo in.
     *                  Positive values rotate it clockwise, negative values rotate it counterclockwise.
     */
    public void rotateYawServo(double direction) {
        double targetPosition = YAW_SERVO.getPosition() + Math.signum(direction) * servoIncrement;
        YAW_SERVO.setPosition(targetPosition);
    }

    /**
     * Rotate the yaw servo to a position specified in degrees.
     * @param degrees The target angle of the yaw servo in degrees.
     */
    public void rotateYawServoToAngle(double degrees) {
        YAW_SERVO.setPosition(degrees / 360.0);
    }
}