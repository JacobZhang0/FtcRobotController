package org.firstinspires.ftc.teamcode.Autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name="Autonomous2024_25BlueLeft", group = "Auto")
public class Autonomous2024_25BlueLeft extends LinearOpMode {

    public class Shoulder {
        private DcMotorEx shoulder;
        private int targetPosition;

        public Shoulder(HardwareMap hardwareMap) {
            shoulder = hardwareMap.get(DcMotorEx.class, "Lift");
            shoulder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            //shoulder.setTargetPositionTolerance(10);  // Set tolerance for position holding
        }

        public class ShoulderAllIn implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                targetPosition = 50;
                shoulder.setTargetPosition(targetPosition);
                shoulder.setPower(0.2);
                shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sleep(1000);

                packet.put("Shoulder Position", shoulder.getCurrentPosition());
                return false;
            }
        }
        public Action shoulderAllIn() {
            return new ShoulderAllIn();
        }

        public class ShoulderErect implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                targetPosition = 420;
                shoulder.setTargetPosition(targetPosition);
                shoulder.setPower(0.2);
                shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sleep(3000);

                packet.put("Shoulder Position", shoulder.getCurrentPosition());
                return false;
            }
        }
        public Action shoulderErect() {
            return new ShoulderErect();
        }

        public class ShoulderAllOut implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                targetPosition = 690;
                shoulder.setTargetPosition(targetPosition);
                shoulder.setPower(0.2);
                shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sleep(2000);

                packet.put("Shoulder Position", shoulder.getCurrentPosition());
                return false;
            }
        }
        public Action shoulderAllOut() {
            return new ShoulderAllOut();
        }
    }

    public class Elbow {
        private Servo elbow;

        public Elbow(HardwareMap hardwareMap) {
            elbow = hardwareMap.get(Servo.class, "Elbow");
        }

        public class ElbowAllOut implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                elbow.setPosition(0.5);
                return false;
            }
        }
        public Action elbowAllOut() {
            return new ElbowAllOut();
        }

        public class ElbowAllIn implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                elbow.setPosition(1);
                return false;
            }
        }
        public Action elbowAllIn() {
            return new ElbowAllIn();
        }

        public class ElbowErect implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                elbow.setPosition(0.9);
                return false;
            }
        }
        public Action elbowErect() {
            return new ElbowErect();
        }
    }

    public class Wrist {
        private Servo wrist;

        public Wrist(HardwareMap hardwareMap) {
            wrist = hardwareMap.get(Servo.class, "Wrist");
        }

        public class CloseWrist implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                wrist.setPosition(1.0);
                return false;
            }
        }
        public Action closeWrist() {
            return new CloseWrist();
        }

        public class OpenWrist implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                wrist.setPosition(0.0);
                return false;
            }
        }
        public Action openWrist() {
            return new OpenWrist();
        }

        public class MiddleWrist implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                wrist.setPosition(0.5);
                return false;
            }
        }
        public Action middleWrist() {
            return new MiddleWrist();
        }
    }

    public class Thumb {
        private Servo thumb;

        public Thumb(HardwareMap hardwareMap) {
            thumb = hardwareMap.get(Servo.class, "Thumb");
        }

        public class CloseThumb implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                thumb.setPosition(0.8);
                return false;
            }
        }
        public Action closeThumb() {
            return new CloseThumb();
        }

        public class OpenThumb implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                thumb.setPosition(0.5);
                return false;
            }
        }
        public Action openThumb() {
            return new OpenThumb();
        }
    }



    @Override
    public void runOpMode() {
        Pose2d initalPose = new Pose2d(35, 62, Math.toRadians(270));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initalPose);
        Shoulder shoulder = new Shoulder(hardwareMap);
        Wrist wrist = new Wrist(hardwareMap);
        Thumb thumb = new Thumb(hardwareMap);
        Elbow elbow = new Elbow(hardwareMap);



        TrajectoryActionBuilder firstSampleDrive = drive.actionBuilder(initalPose)
                .strafeToLinearHeading(new Vector2d(53, 54), Math.toRadians(45));

        TrajectoryActionBuilder toSecondSample = drive.actionBuilder(new Pose2d(53, 54, Math.toRadians(45)))
                .strafeToLinearHeading(new Vector2d(47.9, 39.4), Math.toRadians(270));
        TrajectoryActionBuilder toSecondSampleBasket = drive.actionBuilder(new Pose2d(47.9, 39.4, Math.toRadians(270)))
                .strafeToLinearHeading(new Vector2d(53, 54), Math.toRadians(45));

        TrajectoryActionBuilder toThirdSample = drive.actionBuilder(new Pose2d(53, 54, Math.toRadians(45)))
                .strafeToLinearHeading(new Vector2d(58, 39), Math.toRadians(270));
        TrajectoryActionBuilder toThirdSampleBasket = drive.actionBuilder(new Pose2d(58, 39, Math.toRadians(270)))
                .strafeToLinearHeading(new Vector2d(53, 54), Math.toRadians(45));

        TrajectoryActionBuilder toFourthSample = drive.actionBuilder(new Pose2d(53, 54, Math.toRadians(45)))
                .strafeToLinearHeading(new Vector2d(56, 25), Math.toRadians(0));
        TrajectoryActionBuilder toFourthSampleBasket = drive.actionBuilder(new Pose2d(56, 25, Math.toRadians(0)))
                .strafeToLinearHeading(new Vector2d(53, 54), Math.toRadians(45));


        Actions.runBlocking(wrist.closeWrist());
        Actions.runBlocking(thumb.closeThumb());

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(new SequentialAction(
                firstSampleDrive.build(),
                elbow.elbowAllOut(),
                shoulder.shoulderErect(),
                wrist.openWrist(),
                thumb.openThumb(),
                elbow.elbowAllIn(),
                shoulder.shoulderAllIn(),
                wrist.closeWrist()
        ));

        Actions.runBlocking(new SequentialAction(
                toSecondSample.build(),
                elbow.elbowErect(),
                shoulder.shoulderAllOut(),
                wrist.middleWrist(),
                thumb.closeThumb(),
                elbow.elbowAllIn(),
                shoulder.shoulderAllIn(),
                toSecondSampleBasket.build(),
                elbow.elbowAllOut(),
                shoulder.shoulderErect(),
                wrist.openWrist(),
                thumb.openThumb(),
                elbow.elbowAllIn(),
                shoulder.shoulderAllIn(),
                wrist.closeWrist()
        ));

        Actions.runBlocking(new SequentialAction(
                toThirdSample.build(),
                elbow.elbowErect(),
                shoulder.shoulderAllOut(),
                wrist.middleWrist(),
                thumb.closeThumb(),
                elbow.elbowAllIn(),
                shoulder.shoulderAllIn(),
                toThirdSampleBasket.build(),
                elbow.elbowAllOut(),
                shoulder.shoulderErect(),
                wrist.openWrist(),
                thumb.openThumb(),
                elbow.elbowAllIn(),
                shoulder.shoulderAllIn()
        ));

        /*Actions.runBlocking(new SequentialAction(
                toFourthSample.build(),
                elbow.elbowErect(),
                shoulder.shoulderAllOut(),
                elbow.elbowAllIn(),
                shoulder.shoulderAllIn(),
                toFourthSampleBasket.build(),
                elbow.elbowAllOut(),
                shoulder.shoulderErect(),
                elbow.elbowAllIn(),
                shoulder.shoulderAllIn()
        ));*/


    }
}
