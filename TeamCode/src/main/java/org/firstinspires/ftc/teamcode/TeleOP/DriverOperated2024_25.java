package org.firstinspires.ftc.teamcode.TeleOP;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="DriverOperated2024_25", group="TeleOP")
public class DriverOperated2024_25 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("Front_Left");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("Back_Left");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("Front_Right");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("Back_Right");

        DcMotor shoulder = hardwareMap.dcMotor.get("Lift");
        //Servo shoulderR = hardwareMap.servo.get("Right_Shoulder");
        Servo elbow = hardwareMap.servo.get("Elbow");
        Servo wrist = hardwareMap.servo.get("Wrist");
        Servo thumb = hardwareMap.servo.get("Thumb");

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        shoulder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        thumb.setPosition(0);

        //elbow.setDirection(Servo.Direction.REVERSE);

        waitForStart();

        if (isStopRequested()) return;

        //shoulderL.setPosition(shoulderL.MAX_POSITION);
        //shoulderR.setPosition(shoulderR.MIN_POSITION);

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            double shoulderMotion = - gamepad2.left_stick_y * 0.25;
            double elbowMotion = - gamepad2.right_stick_y;

            shoulder.setPower(shoulderMotion);
            //elbow.setPower(elbowMotion);

            if(gamepad2.left_bumper) {
                wrist.setPosition(wrist.MIN_POSITION);
            }
            if(gamepad2.right_bumper) {
                wrist.setPosition(wrist.MAX_POSITION);
            }

            if(gamepad2.x) {
                thumb.setPosition(0);
            }
            if(gamepad2.b) {
                thumb.setPosition(1);
            }

            if(gamepad2.y) {
                shoulder.setTargetPosition(420);
                shoulder.setPower(0.25);
                shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sleep(1500);
                shoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            //shoulderPos += (shoulderMotion);
            //double elbowMotion = gamepad2.right_stick_y;

            if(gamepad2.a) {
                elbow.setPosition(elbow.MIN_POSITION + 0.2);
                shoulder.setTargetPosition(690);
                shoulder.setPower(0.25);
                shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sleep(1750);
                shoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            if(gamepad2.b) {
                elbow.setPosition(elbow.MAX_POSITION - 0.1);
                shoulder.setTargetPosition(50);
                shoulder.setPower(0.25);
                shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sleep(1750);
                shoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            if(gamepad2.x) {
                elbow.setPosition(elbow.MIN_POSITION + 0.1);
                shoulder.setTargetPosition(420);
                shoulder.setPower(0.25);
                shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sleep(1750);
                shoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            motorFrontLeft.setPower(frontLeftPower);
            motorBackLeft.setPower(backLeftPower);
            motorFrontRight.setPower(frontRightPower);
            motorBackRight.setPower(backRightPower);

            telemetry.addData("Shoulder postion", shoulder.getCurrentPosition());
            telemetry.addData("Thumb postion", thumb.getPosition());
            telemetry.addData("Elbow Position", elbow.getPosition());
            telemetry.addData("Wrist Position", wrist.getPosition());
            telemetry.update();
        }
    }
}
