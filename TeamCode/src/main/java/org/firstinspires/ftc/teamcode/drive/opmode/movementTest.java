package org.firstinspires.ftc.teamcode.drive.opmode;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;
import static java.lang.Math.max;
import static java.lang.Math.min;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.profile.AccelerationConstraint;
import com.acmerobotics.roadrunner.profile.VelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;


import org.apache.commons.math3.geometry.euclidean.twod.Line;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
@Autonomous(name = "movement Auto", group = "LinearOpMode")
public class movementTest extends LinearOpMode {

    private CRServo leftFeedServo = null;
    private CRServo rightFeedServo = null;

    @Override
    public void runOpMode() {
        leftFeedServo = hardwareMap.get(CRServo.class, "frontLeftIntakeServo");
        rightFeedServo = hardwareMap.get(CRServo.class, "frontRightIntakeServo");
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(12, 60, Math.toRadians(-90));
        TrajectorySequence test = drive.trajectorySequenceBuilder(startPose)
                .addDisplacementMarker(()->{
                    leftFeedServo.setPower(1.0);
                    rightFeedServo.setPower(-1.0);
                })
                .forward(1)
                .addDisplacementMarker(()->{
                    leftFeedServo.setPower(0.0);
                    rightFeedServo.setPower(0.0);
                })
                .forward(-1)
                .waitSeconds(1.0)
                .build();



        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectorySequence(test);
    }

}
