package org.firstinspires.ftc.teamcode.drive.opmode;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Motor Configuration Wiring Tester", group = "Linear OpMode")
public class MotorConfigurationWiringTester extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightBack = null;
    private DcMotor armExtensionFront = null;
    private DcMotor armExtensionBack = null;
    private DcMotor armHeightMotor = null;
    private DcMotor airplaneMotor = null;
    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() {
        leftFront = hardwareMap.get(DcMotor.class, "frontLeftDriveMotor");
        rightFront = hardwareMap.get(DcMotor.class, "frontRightDriveMotor");
        leftBack = hardwareMap.get(DcMotor.class, "backLeftDriveMotor");
        rightBack = hardwareMap.get(DcMotor.class, "backRightDriveMotor");

        armExtensionFront = hardwareMap.get(DcMotor.class, "frontArmExtensionMotor");
        armExtensionBack = hardwareMap.get(DcMotor.class, "backArmExtensionMotor");
        armHeightMotor = hardwareMap.get(DcMotor.class, "armHeightMotor");

        airplaneMotor = hardwareMap.get(DcMotor.class, "airplaneMotor");

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {

            if (gamepad1.a){
                leftBack.setPower(1.0);
            }

            if (gamepad1.b){
                rightBack.setPower(1.0);
            }
            if (gamepad1.x){
                leftFront.setPower(1.0);
            }
            if (gamepad1.y){
                rightFront.setPower(1.0);
            }
            if (!gamepad1.a && !gamepad1.b && !gamepad1.x && !gamepad1.y){
                leftFront.setPower(0);
                leftBack.setPower(0);
                rightBack.setPower(0);
                rightFront.setPower(0);
            }

            telemetry.addData("Left Front: ",  leftFront.getCurrentPosition());
            telemetry.addData("Right Front: ", rightFront.getCurrentPosition());
            telemetry.addData("Left Back: ", leftBack.getCurrentPosition());
            telemetry.addData("Right Back: ", rightBack.getCurrentPosition());
            telemetry.addData("Arm Front: ", armExtensionFront.getCurrentPosition());
            telemetry.addData("Arm Back: ", armExtensionBack.getCurrentPosition());
            telemetry.addData("Arm Height: ", armHeightMotor.getCurrentPosition());
            telemetry.addData("Airplane: ", airplaneMotor.getCurrentPosition());


            telemetry.update();

        }
    }
}
