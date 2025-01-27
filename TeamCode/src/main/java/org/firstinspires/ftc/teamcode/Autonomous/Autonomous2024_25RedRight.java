package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name="Autonomous2024_25RedRight", group = "Auto")
public class Autonomous2024_25RedRight extends LinearOpMode {

    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(12.5, -62, Math.toRadians(90)));
        Action trajectory1;

        trajectory1 = drive.actionBuilder(drive.pose)
                .strafeTo(new Vector2d(55, -62))
                /*.strafeTo(new Vector2d(10, -33.1))
                .strafeTo(new Vector2d(47.9, -39.4))
                .strafeToLinearHeading(new Vector2d(-53, -54), Math.toRadians(225))
                .strafeToLinearHeading(new Vector2d(58, -39), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(-53, -54), Math.toRadians(225))
                .strafeToLinearHeading(new Vector2d(56, -25), Math.toRadians(0))
                .strafeToLinearHeading(new Vector2d(-53, -54), Math.toRadians(225))*/
                .build();

        waitForStart();

        if(isStopRequested()) return;

        Action trajectoryActionChosen;
        trajectoryActionChosen = trajectory1;

        Actions.runBlocking(new SequentialAction(
                trajectoryActionChosen
        ));
    }
}
