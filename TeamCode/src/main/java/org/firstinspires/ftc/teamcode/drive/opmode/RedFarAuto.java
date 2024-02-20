package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Red Far Auto", group = "LinearOpMode")
public class RedFarAuto extends LinearOpMode {
    private final int READ_PERIOD = 1;

    private HuskyLens huskyLens;


    private DcMotor armExtensionFront = null;
    private DcMotor armExtensionBack = null;
    private DcMotor armHeightMotor = null;

    private CRServo leftFeedServo = null;
    private CRServo rightFeedServo = null;

    private Servo angleServo = null;

    private final ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode() {
        huskyLens = hardwareMap.get(HuskyLens.class, "huskyLens");

        armExtensionFront = hardwareMap.get(DcMotor.class, "frontArmExtensionMotor");
        armExtensionBack = hardwareMap.get(DcMotor.class, "backArmExtensionMotor");
        armHeightMotor = hardwareMap.get(DcMotor.class, "armHeightMotor");

        leftFeedServo = hardwareMap.get(CRServo.class, "frontLeftIntakeServo");
        rightFeedServo = hardwareMap.get(CRServo.class, "frontRightIntakeServo");

        angleServo = hardwareMap.get(Servo.class, "angleServo");

        armExtensionFront.setDirection(DcMotor.Direction.FORWARD);
        armExtensionBack.setDirection(DcMotor.Direction.REVERSE);

        double intakeAngle = 0.4;
        double outtakeAngle = 0.535;

        double rightFeedOuttake = 0.3;
        double leftFeedOuttake = -0.3;

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-36, -60, Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        TrajectorySequence centerStart = drive.trajectorySequenceBuilder(startPose)
                .forward(23.85)
                .build();


        TrajectorySequence centerBackAndBoard = drive.trajectorySequenceBuilder(centerStart.end())
                .forward(-5)
                .waitSeconds(0.1)
                .strafeRight(-15)
                .waitSeconds(0.1)
                .lineToConstantHeading(new Vector2d(-51, -10))
                .waitSeconds(0.1)
                .turn(Math.toRadians(-90))
                .waitSeconds(0.1)
                .forward(97)
                .waitSeconds(0.1)
                //assuming standard preload procedure is left side yellow, right side purple
                .lineToConstantHeading(new Vector2d(46, -33))
                .build();

        TrajectorySequence centerPark = drive.trajectorySequenceBuilder(centerBackAndBoard.end())
                .forward(-2)
                .build();


        TrajectorySequence leftStart = drive.trajectorySequenceBuilder(startPose)
                .forward(27)
                .turn(Math.toRadians(90))
                .waitSeconds(0.1)
                .forward(-2)
                .build();

        TrajectorySequence leftBackAndBoard = drive.trajectorySequenceBuilder(leftStart.end())
                .turn(Math.toRadians(-90))
                .waitSeconds(0.1)

                .forward(28)
                .waitSeconds(0.1)
                .turn(Math.toRadians(-90))
                .waitSeconds(0.1)
                .forward(81)
                //assuming standard preload procedure is left side yellow, right side purple
                .lineToConstantHeading(new Vector2d(47, -29))
                .build();

        TrajectorySequence leftPark = drive.trajectorySequenceBuilder(leftBackAndBoard.end())
                .forward(-2)
                .build();

        TrajectorySequence rightStart = drive.trajectorySequenceBuilder(startPose)
                .forward(23.85)
                .waitSeconds(0.1)
                .turn(Math.toRadians(-58))
                .waitSeconds(0.1)
                .forward(-2)
                .build();

        TrajectorySequence rightBackAndBoard = drive.trajectorySequenceBuilder(rightStart.end())
                .forward(-2)
                .waitSeconds(0.1)
                .turn(Math.toRadians(58))
                .waitSeconds(0.1)
                .lineToConstantHeading(new Vector2d(-38, -10))
                .waitSeconds(0.1)
                .turn(Math.toRadians(-90))
                .lineToConstantHeading(new Vector2d(46,-12))
                .waitSeconds(0.2)
                //assuming standard preload procedure is left side yellow, right side purple
                .lineToConstantHeading(new Vector2d(46, -40))
                .build();

        TrajectorySequence rightPark = drive.trajectorySequenceBuilder(rightBackAndBoard.end())
                .forward(-2)
                .build();


        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
        waitForStart();

        int duckPos = 0;
        int duckID = 0;
        int duckHeight = 0;
        HuskyLens.Block[] blocks = huskyLens.blocks();
        telemetry.addData("Block count", blocks.length);
        for (int i = 0; i < blocks.length; i++) {
            telemetry.addData("Block", blocks[i].toString());
            int thisColorID = blocks[i].id;// save the current recognition's Color ID
            duckPos = blocks[i].x;
            duckID = blocks[i].id;
            duckHeight = blocks[i].y;
            telemetry.addData("This Color ID", thisColorID);     // display that Color ID

        }

        if(isStopRequested()) return;


        if (duckPos < 100 && duckPos >0 && duckID == 1){
            angleServo.setPosition(intakeAngle);

            armExtensionFront.setPower(0.5);
            armExtensionBack.setPower(0.5);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 0.3)) {}

            armExtensionFront.setPower(0.0);
            armExtensionBack.setPower(0.0);
            armHeightMotor.setPower(-0.8);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 0.4)) {}

            armHeightMotor.setPower(0.0);
            drive.followTrajectorySequence(leftStart);

            rightFeedServo.setPower(rightFeedOuttake);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 0.5)) {}

            rightFeedServo.setPower(0.0);
            armHeightMotor.setPower(0.8);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 0.4)) {}

            armHeightMotor.setPower(0.0);
            angleServo.setPosition(outtakeAngle);

            drive.followTrajectorySequence(leftBackAndBoard);

            armHeightMotor.setPower(0.8);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 0.3)) {}

            armHeightMotor.setPower(0.0);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 0.5)) {}

            armExtensionBack.setPower(0.8);
            armExtensionFront.setPower(0.8);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 1.3)) {}

            armExtensionFront.setPower(0.0);
            armExtensionBack.setPower(0.0);
            leftFeedServo.setPower(leftFeedOuttake);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 0.5)) {}
            leftFeedServo.setPower(0.0);

            armExtensionBack.setPower(-0.8);
            armExtensionFront.setPower(-0.8);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 1.0)) {}
            armExtensionFront.setPower(0.0);
            armExtensionBack.setPower(0.0);

            drive.followTrajectorySequence(leftPark);
        }
        if (duckPos > 100 && duckPos < 210 && duckID == 1){

            angleServo.setPosition(intakeAngle);

            armExtensionFront.setPower(0.5);
            armExtensionBack.setPower(0.5);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 0.3)) {}

            armExtensionFront.setPower(0.0);
            armExtensionBack.setPower(0.0);
            armHeightMotor.setPower(-0.8);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 0.4)) {}

            armHeightMotor.setPower(0.0);
            drive.followTrajectorySequence(centerStart);

            rightFeedServo.setPower(rightFeedOuttake);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 0.5)) {}

            rightFeedServo.setPower(0.0);
            armHeightMotor.setPower(0.8);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 0.4)) {}

            armHeightMotor.setPower(0.0);
            angleServo.setPosition(outtakeAngle);

            drive.followTrajectorySequence(centerBackAndBoard);

            armHeightMotor.setPower(0.8);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 0.3)) {}

            armHeightMotor.setPower(0.0);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 0.5)) {}

            armExtensionBack.setPower(0.8);
            armExtensionFront.setPower(0.8);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 1.3)) {}

            armExtensionFront.setPower(0.0);
            armExtensionBack.setPower(0.0);
            leftFeedServo.setPower(leftFeedOuttake);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 0.5)) {}
            leftFeedServo.setPower(0.0);

            armExtensionBack.setPower(-0.8);
            armExtensionFront.setPower(-0.8);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 1.0)) {}
            armExtensionFront.setPower(0.0);
            armExtensionBack.setPower(0.0);

            drive.followTrajectorySequence(centerPark);

        }
        if (duckPos == 0 || duckPos > 210){
            angleServo.setPosition(intakeAngle);

            armExtensionFront.setPower(0.5);
            armExtensionBack.setPower(0.5);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 0.3)) {}

            armExtensionFront.setPower(0.0);
            armExtensionBack.setPower(0.0);
            armHeightMotor.setPower(-0.8);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 0.4)) {}

            armHeightMotor.setPower(0.0);
            drive.followTrajectorySequence(rightStart);

            rightFeedServo.setPower(rightFeedOuttake);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 0.5)) {}

            rightFeedServo.setPower(0.0);
            armHeightMotor.setPower(0.8);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 0.4)) {}

            armHeightMotor.setPower(0.0);
            angleServo.setPosition(outtakeAngle);

            drive.followTrajectorySequence(rightBackAndBoard);

            armHeightMotor.setPower(0.8);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 0.3)) {}

            armHeightMotor.setPower(0.0);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 0.5)) {}

            armExtensionBack.setPower(0.8);
            armExtensionFront.setPower(0.8);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 1.3)) {}

            armExtensionFront.setPower(0.0);
            armExtensionBack.setPower(0.0);
            leftFeedServo.setPower(leftFeedOuttake);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 0.5)) {}
            leftFeedServo.setPower(0.0);

            armExtensionBack.setPower(-0.8);
            armExtensionFront.setPower(-0.8);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 1.0)) {}
            armExtensionFront.setPower(0.0);
            armExtensionBack.setPower(0.0);

            drive.followTrajectorySequence(rightPark);
        }
    }
}