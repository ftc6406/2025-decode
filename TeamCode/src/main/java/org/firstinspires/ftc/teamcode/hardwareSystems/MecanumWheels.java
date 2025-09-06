package org.firstinspires.ftc.teamcode.hardwareSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.Arrays;
import java.util.Collections;
import java.util.HashSet;
import java.util.List;

public class MecanumWheels extends Wheels {
    /**
     * The motor powering the front left wheel.
     */
    private final DcMotor FRONT_LEFT_MOTOR;
    /**
     * The motor powering the front right wheel.
     */
    private final DcMotor FRONT_RIGHT_MOTOR;
    /**
     * The motor powering the back left wheel.
     */
    private final DcMotor BACK_LEFT_MOTOR;
    /**
     * The motor powering the back right wheel.
     */
    private final DcMotor BACK_RIGHT_MOTOR;

    public MecanumWheels(MotorSet motorSet, WheelDistances wheelDistances, double ticksPerInch) {
        super(motorSet.MOTORS, wheelDistances, ticksPerInch);

        this.FRONT_LEFT_MOTOR = motorSet.FRONT_LEFT_MOTOR;
        this.FRONT_RIGHT_MOTOR = motorSet.FRONT_RIGHT_MOTOR;
        this.BACK_LEFT_MOTOR = motorSet.BACK_LEFT_MOTOR;
        this.BACK_RIGHT_MOTOR = motorSet.BACK_RIGHT_MOTOR;

        // Reset position to 0
        for (DcMotor motor: MOTORS) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        /*
         * Set the directions of the motors.
         * The right and left motors run in opposite directions of each other.
         * Positive is forward for all motors.
         */
        FRONT_LEFT_MOTOR.setDirection(DcMotorSimple.Direction.REVERSE);
        FRONT_RIGHT_MOTOR.setDirection(DcMotorSimple.Direction.FORWARD);
        BACK_LEFT_MOTOR.setDirection(DcMotorSimple.Direction.REVERSE);
        BACK_RIGHT_MOTOR.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public DcMotor getFrontLeftMotor() {
        return FRONT_LEFT_MOTOR;
    }

    public DcMotor getFrontRightMotor() {
        return FRONT_RIGHT_MOTOR;
    }

    public DcMotor getBackLeftMotor() {
        return BACK_LEFT_MOTOR;
    }

    public DcMotor getBackRightMotor() {
        return BACK_RIGHT_MOTOR;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void drive(double x, double y, double theta) {
        for (DcMotor motor : MOTORS) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        /*
        double frontLeftPower = y + x + theta;
        double frontRightPower = y - x - theta;
        double backLeftPower = y - x + theta;
        double backRightPower = y + x - theta;
         */
        double frontLeftPower = -theta + x + y;
        double frontRightPower = theta + x + y;
        double backLeftPower = -theta - x + y;
        double backRightPower = theta - x + y;

        // Scale the motor powers to be within +/- 1.0
        List<Double> powers = Arrays.asList(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
        double max = Collections.max(powers);
        if (max > 1.0) {
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }

        FRONT_LEFT_MOTOR.setPower(frontLeftPower);
        FRONT_RIGHT_MOTOR.setPower(frontRightPower);
        BACK_LEFT_MOTOR.setPower(backLeftPower);
        BACK_RIGHT_MOTOR.setPower(backRightPower);
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void driveDistance(double forwardDistance) {
        driveDistance(0, forwardDistance);
    }

    /**
     * {@inheritDoc}
     *
     * <strong><i>THIS METHOD IS STILL EXPERIMENTAL!</i></strong>
     */
    @Override
    public void driveDistance(double sidewaysDistance, double forwardDistance) {
        // Apply Pythagorean's Theorem to find the Euclidean distance
        double totalDistance = Math.sqrt(Math.pow(forwardDistance, 2) + Math.pow(sidewaysDistance, 2));

        // Scale the motor motor power based on trigonometry
        double xPower = MOTOR_POWER * (sidewaysDistance / totalDistance);
        double yPower = MOTOR_POWER * (forwardDistance / totalDistance);
        drive(xPower, yPower, 0);

        int frontLeftTickPosition = FRONT_LEFT_MOTOR.getCurrentPosition() + (int) ((sidewaysDistance - forwardDistance) * TICKS_PER_INCH);
        int frontRightTickPosition = FRONT_RIGHT_MOTOR.getCurrentPosition() - (int) ((-sidewaysDistance + forwardDistance) * TICKS_PER_INCH);
        int backLeftTickPosition = BACK_LEFT_MOTOR.getCurrentPosition() + (int) ((-sidewaysDistance - forwardDistance) * TICKS_PER_INCH);
        int backRightTickPosition = BACK_RIGHT_MOTOR.getCurrentPosition() - (int) ((sidewaysDistance + forwardDistance) * TICKS_PER_INCH);

        FRONT_LEFT_MOTOR.setTargetPosition(frontLeftTickPosition);
        FRONT_RIGHT_MOTOR.setTargetPosition(frontRightTickPosition);
        BACK_LEFT_MOTOR.setTargetPosition(backLeftTickPosition);
        BACK_RIGHT_MOTOR.setTargetPosition(backRightTickPosition);

        for (DcMotor motor : MOTORS) {
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void turn(double degrees) {
        // The diameter of the circle that the wheels make when rotating 360 degrees.
        double diameter = Math.sqrt(Math.pow(LATERAL_DISTANCE, 2) + Math.pow(LONGITUDINAL_DISTANCE, 2));
        double circumference = diameter * Math.PI;

        // How far the wheels have to move.
        double arcLength = (degrees / 360.0) * circumference;
        int ticks = (int) Math.round(arcLength * TICKS_PER_INCH) * 4 / 3;

        // Left wheels
        FRONT_LEFT_MOTOR.setTargetPosition(FRONT_LEFT_MOTOR.getCurrentPosition() - ticks);
        FRONT_LEFT_MOTOR.setPower(-MOTOR_POWER);
        BACK_LEFT_MOTOR.setTargetPosition(BACK_LEFT_MOTOR.getCurrentPosition() - ticks);
        BACK_LEFT_MOTOR.setPower(-MOTOR_POWER);

        // Right wheels
        FRONT_RIGHT_MOTOR.setTargetPosition(FRONT_RIGHT_MOTOR.getCurrentPosition() + ticks);
        FRONT_RIGHT_MOTOR.setPower(MOTOR_POWER);
        BACK_RIGHT_MOTOR.setTargetPosition(BACK_RIGHT_MOTOR.getCurrentPosition() + ticks);
        BACK_RIGHT_MOTOR.setPower(MOTOR_POWER);

        for (DcMotor motor : MOTORS) {
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    /**
     * Passed into the `MecanumWheels` constructor.
     * Contains all four motors.
     */
    public static class MotorSet {
        public final HashSet<DcMotor> MOTORS;
        /* The DcMotors powering the wheels */
        private final DcMotor FRONT_LEFT_MOTOR;
        private final DcMotor FRONT_RIGHT_MOTOR;
        private final DcMotor BACK_LEFT_MOTOR;
        private final DcMotor BACK_RIGHT_MOTOR;

        public MotorSet(DcMotor frontLeftMotor, DcMotor frontRightMotor, DcMotor backLeftMotor, DcMotor backRightMotor) {
            MOTORS = new HashSet<>();
            MOTORS.add(frontLeftMotor);
            MOTORS.add(frontRightMotor);
            MOTORS.add(backLeftMotor);
            MOTORS.add(backRightMotor);

            FRONT_LEFT_MOTOR = frontLeftMotor;
            FRONT_RIGHT_MOTOR = frontRightMotor;
            BACK_LEFT_MOTOR = backLeftMotor;
            BACK_RIGHT_MOTOR = backRightMotor;
        }

        public MotorSet() {
            MOTORS = new HashSet<>();

            FRONT_LEFT_MOTOR = null;
            FRONT_RIGHT_MOTOR = null;
            BACK_LEFT_MOTOR = null;
            BACK_RIGHT_MOTOR = null;
        }
    }
}