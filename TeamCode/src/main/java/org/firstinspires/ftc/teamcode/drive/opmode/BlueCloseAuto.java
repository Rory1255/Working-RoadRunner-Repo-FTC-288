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

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(12, 60, Math.toRadians(-90));

        drive.setPoseEstimate(startPose);

        TrajectorySequence center = drive.trajectorySequenceBuilder(startPose)
                .addDisplacementMarker(()->{
                    angleServo.setPosition(intakeAngle);
                })
                .forward(23)
                .addDisplacementMarker(()->{
                    armExtensionFront.setPower(1);
                    armExtensionBack.setPower(1);
                })
                .addDisplacementMarker(()->{
                    armExtensionFront.setPower(0.0);
                    armExtensionBack.setPower(0.0);
                })
                .addDisplacementMarker(()->{
                    armHeightMotor.setPower(-0.5);
                })
                .addDisplacementMarker(()->{
                    armHeightMotor.setPower(0.0);
                })
                .addDisplacementMarker(()->{
                    rightFeedServo.setPower(0.3);
                })
                .waitSeconds(0.5)
                .addDisplacementMarker(()->{
                    rightFeedServo.setPower(0.0);
                })
                .forward(-5)
                .turn(Math.toRadians(90))
                .waitSeconds(0.1)
                .addDisplacementMarker(()->{
                    armHeightMotor.setPower(0.5);
                })
                .waitSeconds(0.2)
                .addDisplacementMarker(()->{
                    armHeightMotor.setPower(0.0);
                })
                .addDisplacementMarker(()->{
                    angleServo.setPosition(outtakeAngle);
                })
                .splineToConstantHeading(new Vector2d(47, 33), 0)
                .waitSeconds(0.5)
                .addDisplacementMarker(()->{
                    leftFeedServo.setPower(-0.3);
                })
                .waitSeconds(0.2)
                .addDisplacementMarker(()->{
                    leftFeedServo.setPower(0.0);
                })
                .lineToSplineHeading(new Pose2d(44, 61))

                .build();

        TrajectorySequence left = drive.trajectorySequenceBuilder(startPose)
                .waitSeconds(0.1)
                .forward(23)
                .turn(Math.toRadians(90))
                .addDisplacementMarker(()->{
                    //Place purple pixel
                    //Run right servo in outtake mode at very low speed
                })
                .waitSeconds(1.0)

                .waitSeconds(0.5)
                .lineToConstantHeading(new Vector2d(12,50))
                .waitSeconds(0.2)
                //assuming standard preload procedure is left side yellow, right side purple
                .splineToConstantHeading(new Vector2d(47, 40), 0)
                .addDisplacementMarker(()->{
                    //score yellow pixel
                    //run left servo in outtake mode at medium speed for 0.5 seconds
                })
                .waitSeconds(0.5)
                .addDisplacementMarker(()->{
                    //raise arm more
                })
                .waitSeconds(0.2)
                .lineToSplineHeading(new Pose2d(44, 61))

                .build();

        TrajectorySequence right = drive.trajectorySequenceBuilder(startPose)
                .waitSeconds(0.1)
                .forward(23)
                .waitSeconds(0.1)
                .turn(Math.toRadians(-58))
                .waitSeconds(1.0)
                .waitSeconds(0.5)
                .lineToConstantHeading(new Vector2d(13,46))
                .waitSeconds(0.2)
                .turn(Math.toRadians(148))

                //assuming standard preload procedure is left side yellow, right side purple
                .splineToConstantHeading(new Vector2d(47, 27), 0)

                .waitSeconds(0.5)
                .addDisplacementMarker(()->{
                    //raise arm more
                })
                .waitSeconds(0.2)
                .lineToSplineHeading(new Pose2d(44, 61))

                .build();


        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
        waitForStart();

        int duckPos = 0;
        HuskyLens.Block[] blocks = huskyLens.blocks();
        telemetry.addData("Block count", blocks.length);
        for (int i = 0; i < blocks.length; i++) {
            telemetry.addData("Block", blocks[i].toString());
            int thisColorID = blocks[i].id;// save the current recognition's Color ID
            duckPos = blocks[i].x;
            telemetry.addData("This Color ID", thisColorID);     // display that Color ID

        }

        if(isStopRequested()) return;

        if (duckPos < 100 && duckPos >0){
            drive.followTrajectorySequence(left);
        }
        if (duckPos > 100 && duckPos < 210){
            drive.followTrajectorySequence(center);
        }
        if (duckPos == 0 || duckPos > 210){
            drive.followTrajectorySequence(right);
        }


    }
}
