package org.firstinspires.ftc.teamcode.TeleOP;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

@TeleOp(name="LionsRoarMecanum", group="TeleOP")
public class LionsRoarMecanum extends LinearOpMode {
    private static final double MAX_LIFT_POSITION = 4000.0;
    private static final double MIN_LIFT_POSITION = 0.0;
    private IMU imu;
    private DcMotor lift_main;
    private DcMotor lift_reversed;

    @Override
    public void runOpMode() throws InterruptedException {
        double liftPower;
        double liftEncoderPosition;
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("Front_Left");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("Back_Left");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("Front_Right");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("Back_Right");
        lift_main = hardwareMap.dcMotor.get("lift_main");
        lift_reversed = hardwareMap.dcMotor.get("lift_mirrored");

        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        lift_main.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift_main.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift_main.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift_reversed.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lift_main.setDirection(DcMotorSimple.Direction.REVERSE);

        // Retrieve the IMU from the hardware map
        imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = gamepad1.left_stick_y; // Remember, this is reversed!
            double x = -gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;
            liftEncoderPosition = lift_main.getCurrentPosition();
            liftPower = gamepad1.right_trigger - gamepad1.left_trigger;

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.options) {
                imu.resetYaw();
            }
            lift_main.setPower(liftPower);
            lift_reversed.setPower(liftPower);
            /*
            if (liftEncoderPosition <= MIN_LIFT_POSITION) {
                lift_main.setPower(Math.max(liftPower, 0));
                lift_reversed.setPower(Math.max(liftPower, 0));
            } else if (liftEncoderPosition >= MAX_LIFT_POSITION) {
                lift_main.setPower(Math.min(liftPower, 0));
                lift_reversed.setPower(Math.min(liftPower, 0));
            }
            else {
                lift_main.setPower(liftPower);
                lift_reversed.setPower(liftPower);
            }
            */
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            motorFrontLeft.setPower(frontLeftPower);
            motorBackLeft.setPower(backLeftPower);
            motorFrontRight.setPower(frontRightPower);
            motorBackRight.setPower(backRightPower);

            telemetry.addData("Lift Position", liftEncoderPosition);
            telemetry.addData("Lift Power", liftPower);
            telemetry.update();
        }
    }
}
