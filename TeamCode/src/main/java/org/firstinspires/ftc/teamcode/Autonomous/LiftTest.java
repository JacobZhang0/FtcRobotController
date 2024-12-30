package org.firstinspires.ftc.teamcode.Autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
@Autonomous(name="LiftTest", group="Tests")
public class LiftTest extends LinearOpMode {

    public class Lift {
        private DcMotorEx liftMain;
        private DcMotorEx liftSecond;

        public Lift(HardwareMap hardwareMap) {
            liftMain = hardwareMap.get(DcMotorEx.class, "lift_main");
            liftSecond = hardwareMap.get(DcMotorEx.class, "lift_mirrored");
            liftMain.setDirection(DcMotorSimple.Direction.REVERSE);
            liftMain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            liftMain.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftMain.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            liftSecond.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        public class LiftUp implements Action {
            private boolean init = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!init) {
                    liftMain.setPower(0.8);
                    liftSecond.setPower(0.8);
                    init = true;
                }

                double position = liftMain.getCurrentPosition();
                packet.put("lift Position", position);
                if (position < 4000) {
                    return true;
                }
                else {
                    liftMain.setPower(0);
                    liftSecond.setPower(0);
                    return false;
                }
            }
        }
        public Action liftUp() {
            return new LiftUp();
        }

        public class LiftDown implements Action {
            private boolean init = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!init) {
                    liftMain.setPower(-0.8);
                    liftSecond.setPower(-0.8);
                    init = true;
                }

                double position = liftMain.getCurrentPosition();
                packet.put("lift Position", position);
                if (position > 0) {
                    return true;
                }
                else {
                    liftMain.setPower(0);
                    liftSecond.setPower(0);
                    return false;
                }
            }
        }
        public Action liftDown() {
            return new LiftDown();
        }
    }

    public void runOpMode() {
        Lift lift = new Lift(hardwareMap);

        Actions.runBlocking(lift.liftDown());

        waitForStart();


        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        lift.liftUp()
                )
        );
    }
}
