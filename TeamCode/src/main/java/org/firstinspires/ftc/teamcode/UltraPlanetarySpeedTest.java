package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * An OpMode that spins a single REV Core Hex motor at full speed and reports
 * its rotational and linear velocity.  This is useful for characterising motor
 * performance and verifying that the encoder readings are correct.
 *
 * <p>According to REV Robotics documentation, the Core Hex motor has a 72:1
 * gearbox and a quadrature encoder with 4 counts per motor revolution,
 * resulting in 288 counts per output shaft
 * revolution【629472301279711†L219-L231】. Knowing the counts per revolution
 * allows us to convert the encoder velocity (ticks per second) into revolutions
 * per second and RPM.  We also estimate the linear tip speed by assuming the
 * 5 mm hex output has a diameter of approximately 5 mm (0.005 m).
 */
@TeleOp(name = "UltraPlanetarySpeedTest")
public class UltraPlanetarySpeedTest extends LinearOpMode {
    /**
     * Counts per output shaft revolution for the Core Hex motor.  The motor
     * contains a magnetic quadrature encoder with 4 counts per internal
     * revolution and a 72:1 gearbox, giving 4 × 72 = 288 counts per output
     * revolution【629472301279711†L219-L231】.
     */
    private static final double COUNTS_PER_REV = 28.0;

    /**
     * Approximate diameter of the Core Hex motor’s 5 mm hex output, in meters.
     * The across-flats measurement of the hex is 5 mm, so we use 0.005 m.
     */
    private static final double SHAFT_DIAMETER_M = 0.005;

    /**
     * Circumference of the hex output, computed from its diameter.  This is
     * used to estimate linear tip speed (m/s).  Note that the hex profile makes
     * the actual path non-circular; this value is a simplification.
     */
    private static final double SHAFT_CIRCUMFERENCE_M =
            Math.PI * SHAFT_DIAMETER_M;

    @Override
    public void runOpMode() {
        // Fetch the Core Hex motor from the hardware map.  The name
        // "coreHexMotor" must match the configuration in the Control Hub.
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, "UltraPlanetaryMotor");

        // Set motor direction.  Use REVERSE if the motor spins backwards.
        motor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Allow the motor to coast when zero power is applied.
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Use the built-in encoder for velocity measurement.  RUN_USING_ENCODER
        // enables getVelocity() to return ticks per
        // second【594545095337415†L320-L322】.
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addLine("Press start to run the planetary motor at full " +
                "speed.");
        telemetry.update();
        waitForStart();

        // Spin the motor at full power.  The power value is a percentage of
        // maximum speed, so 1.0 corresponds to 100% power.
        motor.setPower(1.0);

        while (opModeIsActive()) {
            // getVelocity() returns encoder counts per second.
            double ticksPerSecond = motor.getVelocity();

            // Convert ticks per second to revolutions per second and RPM.
            double revsPerSecond = ticksPerSecond / COUNTS_PER_REV;
            double rpm = revsPerSecond * 60.0;

            // Estimate linear speed of the output shaft at its circumference.
            double metersPerSecond = revsPerSecond * SHAFT_CIRCUMFERENCE_M;

            telemetry.addData("Motor power", motor.getPower());
            telemetry.addData("Encoder velocity (ticks/s)", ticksPerSecond);
            telemetry.addData("Motor speed (RPM)", rpm);
            telemetry.addData("Tip speed (m/s)", metersPerSecond);
            telemetry.update();
        }

        // Stop the motor when the opmode ends.
        motor.setPower(0.0);
    }
}