package org.firstinspires.ftc.teamcode.Autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@Autonomous(name="LiftTest", group="AutonomousTests")
public class LiftTest extends LinearOpMode {

    public class Lift {
        private DcMotorEx lift;

        public Lift(HardwareMap hardwareMap) {
            lift = hardwareMap.get(DcMotorEx.class, "lift");
            lift.setDirection(DcMotorSimple.Direction.REVERSE);
            lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        public class LiftUp implements Action {
            private boolean init = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!init) {
                    lift.setPower(0.8);
                    init = true;
                }

                double position = lift.getCurrentPosition();
                packet.put("lift Position", position);
                if (position < 4000) {
                    return true;
                }
                else {
                    lift.setPower(0);
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
                    lift.setPower(-0.8);
                    init = true;
                }

                double position = lift.getCurrentPosition();
                packet.put("lift Position", position);
                if (position > 0) {
                    return true;
                }
                else {
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
