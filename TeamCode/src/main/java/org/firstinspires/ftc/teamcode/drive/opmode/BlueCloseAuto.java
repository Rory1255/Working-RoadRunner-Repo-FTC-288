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

@Autonomous(name = "Blue Close Auto", group = "LinearOpMode")
public class BlueCloseAuto extends LinearOpMode {
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

        Pose2d startPose = new Pose2d(12, 60, Math.toRadians(-90));

        drive.setPoseEstimate(startPose);

        TrajectorySequence centerStart = drive.trajectorySequenceBuilder(startPose)
                .forward(23.5)
                .build();

        TrajectorySequence centerBackAndBoard = drive.trajectorySequenceBuilder(centerStart.end())
                .forward(-5)
                .waitSeconds(0.1)
                .turn(Math.toRadians(90))
                .waitSeconds(0.1)
                .splineToConstantHeading(new Vector2d(47.5, 32.8), 0)
                .build();

        TrajectorySequence centerPark = drive.trajectorySequenceBuilder(centerBackAndBoard.end())
                .lineToSplineHeading(new Pose2d(44, 61))
                .build();


        TrajectorySequence leftStart = drive.trajectorySequenceBuilder(startPose)
                .forward(27)
                .turn(Math.toRadians(90))
                .waitSeconds(0.1)
                .forward(-1.3)
                .build();

        TrajectorySequence leftBackAndBoard = drive.trajectorySequenceBuilder(leftStart.end())
                .lineToConstantHeading(new Vector2d(12,55))
                .waitSeconds(0.1)
                .splineToConstantHeading(new Vector2d(47.5, 40), 0)
                .build();

        TrajectorySequence leftPark = drive.trajectorySequenceBuilder(leftBackAndBoard.end())
                .lineToSplineHeading(new Pose2d(44, 61))
                .build();


        TrajectorySequence rightStart = drive.trajectorySequenceBuilder(startPose)
                .forward(23)
                .waitSeconds(0.1)
                .turn(Math.toRadians(-58))
                .build();

        TrajectorySequence rightBackAndBoard = drive.trajectorySequenceBuilder(rightStart.end())
                .lineToConstantHeading(new Vector2d(13,46))
                .waitSeconds(0.2)
                .turn(Math.toRadians(148))
                .waitSeconds(0.1)
                .splineToConstantHeading(new Vector2d(47.5, 27), 0)
                .build();

        TrajectorySequence rightPark = drive.trajectorySequenceBuilder(rightBackAndBoard.end())
                .lineToSplineHeading(new Pose2d(44, 61))
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


        if (duckPos < 100 && duckPos >0 && duckID == 2){
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

            armExtensionBack.setPower(0.5);
            armExtensionFront.setPower(0.5);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 0.6)) {}

            armExtensionFront.setPower(0.0);
            armExtensionBack.setPower(0.0);
            leftFeedServo.setPower(leftFeedOuttake);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 0.5)) {}
            leftFeedServo.setPower(0.0);

            armExtensionBack.setPower(-0.5);
            armExtensionFront.setPower(-0.5);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 0.6)) {}
            armExtensionFront.setPower(0.0);
            armExtensionBack.setPower(0.0);

            drive.followTrajectorySequence(leftPark);
        }
        if (duckPos > 100 && duckPos < 210 && duckID == 2){

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

            armExtensionBack.setPower(0.5);
            armExtensionFront.setPower(0.5);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 0.6)) {}

            armExtensionFront.setPower(0.0);
            armExtensionBack.setPower(0.0);
            leftFeedServo.setPower(leftFeedOuttake);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 0.5)) {}
            leftFeedServo.setPower(0.0);

            armExtensionBack.setPower(-0.5);
            armExtensionFront.setPower(-0.5);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 0.6)) {}
            armExtensionFront.setPower(0.0);
            armExtensionBack.setPower(0.0);

            drive.followTrajectorySequence(centerPark);

        }
        if (duckPos == 0 || duckPos > 210 && duckID != 1){
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

            armExtensionBack.setPower(0.5);
            armExtensionFront.setPower(0.5);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 0.6)) {}

            armExtensionFront.setPower(0.0);
            armExtensionBack.setPower(0.0);
            leftFeedServo.setPower(leftFeedOuttake);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 0.5)) {}
            leftFeedServo.setPower(0.0);

            armExtensionBack.setPower(-0.5);
            armExtensionFront.setPower(-0.5);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 0.6)) {}
            armExtensionFront.setPower(0.0);
            armExtensionBack.setPower(0.0);

            drive.followTrajectorySequence(rightPark);
        }


    }
}
