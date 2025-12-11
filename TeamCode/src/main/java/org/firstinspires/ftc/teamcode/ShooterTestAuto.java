package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "ShooterTestAuto", group = "Test")
public class ShooterTestAuto extends LinearOpMode {


    private DcMotorEx launcherMotor;
    private DcMotorEx intakeMotor;


    @Override
    public void runOpMode() {


        // === Map motors exactly like DriverMode ===
        try {
            launcherMotor = hardwareMap.get(DcMotorEx.class, "launcherMotor");
            launcherMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            launcherMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            launcherMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            launcherMotor.setPower(0.0);
            telemetry.addLine("Launcher motor initialised (test)");
        } catch (Exception e) {
            telemetry.addLine("ERROR: launcherMotor NOT FOUND");
        }


        try {
            intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
            intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            intakeMotor.setPower(0.0);
            telemetry.addLine("Intake motor initialised (test)");
        } catch (Exception e) {
            telemetry.addLine("ERROR: intakeMotor NOT FOUND");
        }


        telemetry.update();


        waitForStart();
        if (isStopRequested()) return;


        // If either motor is missing, abort to avoid crashes.
        if (launcherMotor == null || intakeMotor == null) {
            telemetry.addLine("Aborting: shooter or intake not mapped.");
            telemetry.update();
            sleep(3000);
            return;
        }


        // === 1) Spin launcher for whole auto ===
        telemetry.addLine("Spinning launcher at full power");
        telemetry.update();
        launcherMotor.setPower(-1.0);


        // === 2) Feed 3 times with intake ===
        for (int shot = 1; shot <= 3 && opModeIsActive(); shot++) {
            telemetry.addData("Shot", shot);
            telemetry.addLine("Feeding ball with intake");
            telemetry.update();


            // RUN intake forward to feed ball
            intakeMotor.setPower(1.0);
            sleep(3000);   // 2 seconds feed (tune as needed)


            // Stop intake
            intakeMotor.setPower(0.0);
            telemetry.addLine("Stopping intake, pausing");
            telemetry.update();
            if (shot < 3) {
                sleep(2000);   // 2 seconds pause
            }
        }

        // === 3) Shut down ===
        telemetry.addLine("Stopping launcher / intake");
        telemetry.update();
        launcherMotor.setPower(0.0);
        intakeMotor.setPower(0.0);


        sleep(1000);
    }
}
