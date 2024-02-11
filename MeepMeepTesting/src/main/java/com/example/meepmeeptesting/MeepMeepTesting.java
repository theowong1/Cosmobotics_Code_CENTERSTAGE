package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;

public class MeepMeepTesting {
    public static Pose2d blueLeftBoard;
    public static Pose2d blueCenterBoard;
    public static Pose2d blueRightBoard;
    public static Pose2d redLeftBoard;
    public static Pose2d redCenterBoard;
    public static Pose2d redRightBoard;

    public static void main(String[] args) {
        blueLeftBoard = new Pose2d(45, 42, Math.toRadians(180));
        blueCenterBoard = new Pose2d(45, 35.5, Math.toRadians(180));
        blueRightBoard = new Pose2d(45, 29, Math.toRadians(180));
        redLeftBoard = new Pose2d(45, -29, Math.toRadians(180));
        redCenterBoard = new Pose2d(45, -35.5, Math.toRadians(180));
        redRightBoard = new Pose2d(45, -42, Math.toRadians(180));

        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 14.4)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(15.25, 63.25, Math.toRadians(270)))
                                .lineToLinearHeading(blueLeftBoard)
                                .lineToLinearHeading(blueRightBoard)
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
