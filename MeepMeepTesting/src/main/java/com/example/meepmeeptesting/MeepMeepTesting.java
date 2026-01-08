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
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 10.66446239)
                //.setDimensions()
                .setColorScheme(new ColorSchemeBlueDark())
                .build();

        // Red
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-52.2, 47.2, Math.toRadians(125)))
                        .strafeToLinearHeading(new Vector2d(-11.3, 12), Math.toRadians(135))
                        .strafeToLinearHeading(new Vector2d(-11.3, 30), Math.toRadians(90))
                        .strafeToLinearHeading(new Vector2d(-11.3, 12), Math.toRadians(135))
                        .strafeToLinearHeading(new Vector2d(12, 30), Math.toRadians(90))
                        .strafeToLinearHeading(new Vector2d(-11.3, 12), Math.toRadians(135))
                        .strafeToLinearHeading(new Vector2d(36, 30), Math.toRadians(90))
                        .strafeToLinearHeading(new Vector2d(-11.3, 12), Math.toRadians(135))
                        .strafeToLinearHeading(new Vector2d(60, -55), Math.toRadians(90))
                .build());

        /* Blue
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(61.2, -12, Math.toRadians(180)))
                        .strafeToLinearHeading(new Vector2d(-11.3, -12), Math.toRadians(225))
                        .strafeToLinearHeading(new Vector2d(-11.3, -30), Math.toRadians(270))
                        .strafeToLinearHeading(new Vector2d(-11.3, -12), Math.toRadians(225))
                        .strafeToLinearHeading(new Vector2d(12, -30), Math.toRadians(270))
                        .strafeToLinearHeading(new Vector2d(-11.3, -12), Math.toRadians(225))
                        .strafeToLinearHeading(new Vector2d(36, -30), Math.toRadians(270))
                        .strafeToLinearHeading(new Vector2d(-11.3, -12), Math.toRadians(225))
                        .strafeToLinearHeading(new Vector2d(60, 55), Math.toRadians(270))
                .build());
         */

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_BLACK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}