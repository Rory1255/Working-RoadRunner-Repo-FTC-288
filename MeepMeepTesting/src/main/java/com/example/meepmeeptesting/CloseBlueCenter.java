package com.example.meepmeeptesting;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class CloseBlueCenter
{
    public static void main(String[] args) {
        // Declare a MeepMeep instance
        // With a field size of 800 pixels
        MeepMeep meepMeep = new MeepMeep(1200);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Required: Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 40, Math.toRadians(180), Math.toRadians(180), 12)
                // Option: Set theme. Default = ColorSchemeRedDark()
                .setColorScheme(new ColorSchemeBlueDark())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(12, 60, Math.toRadians(-90)))
                                .waitSeconds(0.1)
                                .forward(23)
                                .addDisplacementMarker(()->{
                                    //Place purple pixel
                                    //Run right servo in outtake mode at very low speed
                                })
                                .waitSeconds(1.0)
                                .addDisplacementMarker(()->{
                                    //Lift intake
                                    //Run angle motor to raise arm "x" distance
                                })
                                .addDisplacementMarker(()-> {
                                    //Set intake angle to score position
                                })
                                .waitSeconds(0.5)
                                .forward(-5)
                                .turn(Math.toRadians(90))
                                //assuming standard preload procedure is left side yellow, right side purple
                                .splineToConstantHeading(new Vector2d(47, 33), 0)
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

                                .build()
                );

        // Set field image
        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                // Background opacity from 0-1
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
