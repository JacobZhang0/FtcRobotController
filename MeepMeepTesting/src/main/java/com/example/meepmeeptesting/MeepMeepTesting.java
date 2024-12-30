package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.ColorScheme;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 12.6)
                //.setDimensions()
                .setColorScheme(new ColorSchemeBlueDark())
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(12.5, -62, Math.toRadians(90)))
                .strafeTo(new Vector2d(10, -33.1))
                .strafeTo(new Vector2d(47.9, -39.4))
                .strafeToLinearHeading(new Vector2d(-53, -54), Math.toRadians(225))
                .strafeToLinearHeading(new Vector2d(58, -39), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(-53, -54), Math.toRadians(225))
                .strafeToLinearHeading(new Vector2d(56, -25), Math.toRadians(0))
                .strafeToLinearHeading(new Vector2d(-53, -54), Math.toRadians(225))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}