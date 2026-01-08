package org.firstinspires.ftc.teamcode.TeleOP;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "TwoWheelDrive", group = "")
public class Template extends LinearOpMode {


    @Override
    public void runOpMode(){
        waitForStart();

        while (opModeIsActive()) {
            telemetry.update();

        }
    }
}