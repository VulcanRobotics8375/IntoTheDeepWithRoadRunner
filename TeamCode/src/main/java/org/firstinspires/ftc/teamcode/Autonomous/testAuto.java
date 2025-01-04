package org.firstinspires.ftc.teamcode.Autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.HorizontalExtendo;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


@Config
@Autonomous(name = "test auto", group = "Autonomous")
public class testAuto extends LinearOpMode {

    //subsystem classes
    public class Lift {
        private DcMotorEx liftMotorLeft;
        private DcMotorEx liftMotorRight;
        private int liftTarget;
        private int liftPosition;
        private int lastError = 0;
        private double lastTime;
        private double kP = .01;
        private double kD = .0003;

        public Lift(HardwareMap hardwareMap) {
            liftMotorLeft = hardwareMap.get(DcMotorEx.class, "liftMotorLeft");
            liftMotorRight = hardwareMap.get(DcMotorEx.class, "liftMotorLeft");

            liftMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            liftMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            liftMotorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            liftMotorLeft.setDirection(DcMotorSimple.Direction.REVERSE);

            liftPosition = liftMotorLeft.getCurrentPosition();
            liftTarget = liftPosition;
            lastTime = System.nanoTime() * 1e-9;
        }

        public class LiftUpdate implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                liftPosition = liftMotorLeft.getCurrentPosition();

                int error = liftTarget - liftPosition;

                double errorVelocity = (error - lastError) / (System.nanoTime() * 1e-9 - lastTime);
                lastTime = System.nanoTime() * 1e-9;;

                double power = kP * error + kD * errorVelocity;

                liftMotorLeft.setPower(power);
                liftMotorRight.setPower(power);
                lastError = error;
                return true;
            }
        }


        public class LiftGoTo implements Action {
            private int target;
            public LiftGoTo(int target) {
                this.target = target;
            }
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                liftTarget = target;
                return liftPosition >= liftTarget + 20 && liftPosition <= liftTarget - 20;
            }
        }

        public Action goTo(int target) {
            return new LiftGoTo(target);
        }

        public Action update() {
            return new LiftUpdate();
        }
    }

    public class HorizontalExtendo {
        private Servo linkServoLeft;
        private Servo linkServoRight;
        private final double fullRetractLeft = 0.7;
        private final double fullExtendLeft = 0.044864;
        private final double midLeft = 0.327568;

        private final double fullRetractRight = 0.36;
        private final double fullExtendRight = 0.95513608;
        private final double midRight = 0.9756804;

        public HorizontalExtendo(HardwareMap hardwareMap) {
            linkServoLeft = hardwareMap.get(Servo.class, "linkServoLeft");
            linkServoRight = hardwareMap.get(Servo.class, "linkServoRight");

        }

        public class goToFront implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                linkServoLeft.setPosition(fullExtendLeft);
                linkServoRight.setPosition(fullExtendRight);
                return false;
            }
        }
        public Action goToFront() {
            return new goToFront();
        }

        public class goToBack implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                linkServoLeft.setPosition(fullRetractLeft);
                linkServoRight.setPosition(fullRetractRight);
                return false;
            }
        }
        public Action goToBack() {
            return new goToBack();
        }

        public class goToMid implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                linkServoLeft.setPosition(midLeft);
                linkServoRight.setPosition(midRight);
                return false;
            }
        }
        public Action goToMid() {
            return new goToMid();
        }

    }

    public class Arm {
        private Servo armServoLeft;
        private Servo armServoRight;

        private final double LEFT_FRONT_POSITION = 0.2683;
        private final double RIGHT_FRONT_POSITION = 0.765;

        private final double LEFT_FRONT_DEPOSIT = 0.39;
        private final double RIGHT_FRONT_DEPOSIT = 0.6378;

        private final double LEFT_BACK_DEPOSIT_POSITION = 0.7467;
        private final double RIGHT_BACK_DEPOSIT_POSITION = 0.2833;

        private final double LEFT_TRANSFER_POSITION = 0.5339;
        private final double RIGHT_TRANSFER_POSITION = 0.5;



        public Arm(HardwareMap hardwareMap) {
            armServoLeft = hardwareMap.get(Servo.class, "armServoLeft");
            armServoRight = hardwareMap.get(Servo.class, "armServoRight");

        }

        public class ArmIntake implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                armServoRight.setPosition(RIGHT_FRONT_POSITION);
                armServoLeft.setPosition(LEFT_FRONT_POSITION);
                return false;
            }
        }
        public Action armIntake() {
            return new ArmIntake();
        }

        public class ArmDepositBack implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                armServoRight.setPosition(RIGHT_BACK_DEPOSIT_POSITION);
                armServoLeft.setPosition(LEFT_BACK_DEPOSIT_POSITION);
                return false;
            }
        }
        public Action armBackDeposit() {
            return new ArmDepositBack();
        }

        public class ArmDepositFront implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                armServoRight.setPosition(RIGHT_FRONT_DEPOSIT);
                armServoLeft.setPosition(LEFT_FRONT_DEPOSIT);
                return false;
            }
        }
        public Action armDepositFront() {
            return new ArmDepositFront();
        }

        public class ArmTransfer implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                armServoRight.setPosition(RIGHT_TRANSFER_POSITION);
                armServoLeft.setPosition(LEFT_TRANSFER_POSITION);
                return false;
            }
        }
        public Action armTransfer() {
            return new ArmTransfer();
        }


    }

    public class PitchandSpin {
        private Servo clawPitch;
        private Servo clawSpin;

        private static final double CLAW_NORMAL_POS = 0.3458;
        private static final double ROLL_SPEC_DEPO = 0.9027;

        private static final double PITCH_STRAIGHT = 0;
        private static final double PITCH_SAMPLE = 0.687;


        public PitchandSpin(HardwareMap hardwareMap) {
            clawSpin = hardwareMap.get(Servo.class, "clawSpin");
            clawPitch = hardwareMap.get(Servo.class, "clawPitch");

        }

        public class Deposit implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                clawPitch.setPosition(PITCH_STRAIGHT);
                clawSpin.setPosition(ROLL_SPEC_DEPO);
                return false;
            }
        }
        public Action deposit() {
            return new Deposit();
        }

        public class Intake implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                clawPitch.setPosition(PITCH_SAMPLE);
                clawSpin.setPosition(CLAW_NORMAL_POS);
                return false;
            }
        }
        public Action intake() {
            return new Intake();
        }
    }

    public class Claw {
        private Servo claw;

        public Claw(HardwareMap hardwareMap) {
            claw = hardwareMap.get(Servo.class, "clawServo");
        }

        public class CloseClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(0.3);
                return false;
            }
        }
        public Action closeClaw() {
            return new CloseClaw();
        }

        public class OpenClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(0.5);
                return false;
            }
        }
        public Action openClaw() {
            return new OpenClaw();
        }
    }
    //end of subsystem classes


    @Override
    public void runOpMode() {
        //set starting position
        Pose2d initialPose =new Pose2d(-37, -63, Math.toRadians(45));

        //initialize subsystems
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Claw claw = new Claw(hardwareMap);
        HorizontalExtendo horizontalExtendo = new HorizontalExtendo(hardwareMap);
        Lift lift = new Lift(hardwareMap);
        Arm arm = new Arm(hardwareMap);
        PitchandSpin pas = new PitchandSpin(hardwareMap);


        Action action1 = drive.actionBuilder(initialPose)

                //preload sample deposit
                .stopAndAdd(lift.goTo(3000))
                .strafeTo(new Vector2d(-54, -54))
                .stopAndAdd(arm.armBackDeposit())
                .stopAndAdd(pas.deposit())
                .waitSeconds(1)
                .stopAndAdd(claw.openClaw())
                .waitSeconds(0.5)
                .stopAndAdd(horizontalExtendo.goToBack())
                .stopAndAdd(arm.armTransfer()) .build();

/*
                //intake sample 2
                .stopAndAdd(lift.goTo(0))
                .strafeTo(new Vector2d(-60, -46.7))
                .turnTo(Math.toRadians(65))
                .waitSeconds(0.5)
                .stopAndAdd(horizontalExtendo.goToFront())
                .stopAndAdd(arm.armIntake())
                .stopAndAdd(pas.intake())
                .waitSeconds(0.7)
                .stopAndAdd(claw.closeClaw())
                .waitSeconds(0.2)
                .stopAndAdd(arm.armTransfer())


                //deposit sample 2
                .stopAndAdd(lift.goTo(3000))
                .stopAndAdd(horizontalExtendo.goToBack())
                .turnTo(Math.toRadians(45))
                .splineToConstantHeading(new Vector2d(-54, -54), Math.toRadians(45))
                .stopAndAdd(arm.armBackDeposit())
                .stopAndAdd(pas.deposit())
                .waitSeconds(2)
                .stopAndAdd(claw.openClaw())
                .waitSeconds(0.2)
                .stopAndAdd(arm.armTransfer())



                //intake sample 3
                .stopAndAdd(lift.goTo(0))
                .turnTo(Math.toRadians(92))
                .stopAndAdd(horizontalExtendo.goToFront())
                .splineToConstantHeading(new Vector2d(-60, -46.7), Math.toRadians(92))
                .stopAndAdd(arm.armIntake())
                .stopAndAdd(pas.intake())
                .waitSeconds(2)
                .stopAndAdd(claw.closeClaw())
                .waitSeconds(0.2)
                .stopAndAdd(arm.armTransfer())



                //deposit sample 3

                .stopAndAdd(lift.goTo(3000))
                .stopAndAdd(horizontalExtendo.goToBack())
                .turnTo(Math.toRadians(45))
                .splineToConstantHeading(new Vector2d(-54, -54), Math.toRadians(45))
                .stopAndAdd(arm.armBackDeposit())
                .stopAndAdd(pas.deposit())
                .waitSeconds(2)
                .stopAndAdd(claw.openClaw())
                .waitSeconds(0.2)
                .stopAndAdd(arm.armTransfer())

                //intake sample 4
                .stopAndAdd(lift.goTo(0))
                .turnTo(Math.toRadians(110))
                .stopAndAdd(horizontalExtendo.goToFront())
                .splineToConstantHeading(new Vector2d(-60, -46.7), Math.toRadians(110))
                .stopAndAdd(arm.armIntake())
                .stopAndAdd(pas.intake())
                .waitSeconds(2)
                .stopAndAdd(claw.closeClaw())
                .waitSeconds(0.2)
                .stopAndAdd(arm.armTransfer())



                //deposit sample 4
                .stopAndAdd(lift.goTo(3000))
                .stopAndAdd(horizontalExtendo.goToBack())
                .turnTo(Math.toRadians(45))
                .splineToConstantHeading(new Vector2d(-54, -54), Math.toRadians(45))
                .stopAndAdd(arm.armBackDeposit())
                .stopAndAdd(pas.deposit())
                .waitSeconds(2)
                .stopAndAdd(claw.openClaw())
                .waitSeconds(0.2)
                .stopAndAdd(arm.armTransfer())

                //park
                .stopAndAdd(lift.goTo(0))
                .turnTo(0)
                .splineToConstantHeading(new Vector2d(-29, -8.7), Math.toRadians(0))
                .build();
*/

        // actions that need to happen on init; for instance, a claw tightening.
        Actions.runBlocking(claw.closeClaw());
        Actions.runBlocking(arm.armTransfer());
        Actions.runBlocking(pas.deposit());
        Actions.runBlocking(horizontalExtendo.goToBack());




        telemetry.addLine("Starting Position");
        telemetry.update();
        waitForStart();

        //stop program if you press stop
        if (isStopRequested()) return;


        Actions.runBlocking(
                new ParallelAction(
                        action1,
                        lift.update()
                )
        );
    }
}