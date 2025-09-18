package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp (name="DriverOperated2025_26", group="TeleOp")
public class DriverOperated2025_26 extends LinearOpMode {

    private DcMotor motorFrontLeft;
    private DcMotor motorBackLeft;
    private DcMotor motorFrontRight;
    private DcMotor motorBackRight;

    @Override
    public void runOpMode()
    {
        motorFrontLeft = hardwareMap.dcMotor.get("Front_Left");
        motorBackLeft = hardwareMap.dcMotor.get("Back_Left");
        motorFrontRight = hardwareMap.dcMotor.get("Front_Right");
        motorBackRight = hardwareMap.dcMotor.get("Back_Right");

        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            motorFrontLeft.setPower(frontLeftPower);
            motorBackLeft.setPower(backLeftPower);
            motorFrontRight.setPower(frontRightPower);
            motorBackRight.setPower(backRightPower);

            telemetry.update();
        }
    }
}
