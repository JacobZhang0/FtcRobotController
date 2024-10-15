package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name = "BlueRight", group = "Autonomous")
public class BlueRight extends LinearOpMode {
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(11.8, 61.7, Math.toRadians(90)));
        Action driveToNet;
        driveToNet = drive.actionBuilder(drive.pose)
                .waitSeconds(1)
                .build();
    }
    public void driveToNet() {

    }
    public void driveToSub() {

    }
    public void hangSpecimen() {

    }
    public void parkObservation() {

    }
    public void parkAscent() {

    }
    /*
    Drive to net zone (avoid alliance partner)
    Pick up alliance-specific sample
    Pick up alliance-specific sample (avoid alliance partner)
    Pick up alliance-neutral sample
    Pick up alliance-neutral sample (avoid alliance partner)
    Park in observation zone
    Park in observation zone (avoid alliance partner)
    Park in ascent zone
    Park in ascent zone (avoid alliance partner)
*/
}