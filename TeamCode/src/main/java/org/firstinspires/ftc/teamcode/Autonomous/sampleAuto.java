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
@Autonomous(name = "sample auto", group = "Autonomous")
public class sampleAuto extends LinearOpMode {

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

        private final double fullRetractLeft = 0.9208;
        private final double fullExtendLeft = 0.33614;

        private final double fullRetractRight = 0.3359;
        private final double fullExtendRight = 0.9206;


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

    }

    public class Arm {
        private Servo armServoLeft;
        private Servo armServoRight;

        // can try to be used for spec front depo move

        private final double FRONT_INTAKE_POSITION = 0.24;

        // used for spec front depo and sample front depo

        private final double FRONT_DEPO_POSITION = 0.4128;


        private final double BACK_DEPOSIT_POSITION = 0.85;


        private final double TRANSFER_POSITION = 0.5861;




        public Arm(HardwareMap hardwareMap) {
            armServoLeft = hardwareMap.get(Servo.class, "armServoLeft");
            armServoRight = hardwareMap.get(Servo.class, "armServoRight");

        }

        public class ArmIntake implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                armServoRight.setPosition(FRONT_INTAKE_POSITION);
                armServoLeft.setPosition(FRONT_INTAKE_POSITION);
                return false;
            }
        }
        public Action armIntake() {
            return new ArmIntake();
        }

        public class ArmDepositBack implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                armServoRight.setPosition(BACK_DEPOSIT_POSITION);
                armServoLeft.setPosition(BACK_DEPOSIT_POSITION);
                return false;
            }
        }
        public Action armBackDeposit() {
            return new ArmDepositBack();
        }

        public class ArmDepositFront implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                armServoRight.setPosition(FRONT_DEPO_POSITION);
                armServoLeft.setPosition(FRONT_DEPO_POSITION);
                return false;
            }
        }
        public Action armDepositFront() {
            return new ArmDepositFront();
        }

        public class ArmTransfer implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                armServoRight.setPosition(TRANSFER_POSITION);
                armServoLeft.setPosition(TRANSFER_POSITION);
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
                claw.setPosition(0.22);
                return false;
            }
        }
        public Action closeClaw() {
            return new CloseClaw();
        }

        public class OpenClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(0.48 );
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
        Pose2d initialPose =new Pose2d(-41, -63, Math.toRadians(0));

        //initialize subsystems
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Claw claw = new Claw(hardwareMap);
        HorizontalExtendo horizontalExtendo = new HorizontalExtendo(hardwareMap);
        Lift lift = new Lift(hardwareMap);
        Arm arm = new Arm(hardwareMap);
        PitchandSpin pas = new PitchandSpin(hardwareMap);


        Action action1 = drive.actionBuilder(initialPose)


                //preload sample deposit

                .stopAndAdd(lift.goTo(2100))
                .setTangent(Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-50, -62),Math.toRadians(180))
                .stopAndAdd(pas.deposit())
                .waitSeconds(1)
                .stopAndAdd(arm.armBackDeposit())
                .waitSeconds(0.5)
                .stopAndAdd(claw.openClaw())
                .waitSeconds(0.5)
                .stopAndAdd(lift.goTo(0))



                //intake sample 2
                .stopAndAdd(arm.armIntake())
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-50.5,-51.5,Math.toRadians(87)),Math.toRadians(90))
                .stopAndAdd(horizontalExtendo.goToFront())
                .stopAndAdd(pas.intake())
                .waitSeconds(1)
                .stopAndAdd(claw.closeClaw())
                .waitSeconds(0.7)
                .stopAndAdd(arm.armTransfer())
                .stopAndAdd(horizontalExtendo.goToBack())



                //deposit sample 2
                .turnTo(Math.toRadians(45))
                .setTangent(Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-54.5, -53), Math.toRadians(45))
                .stopAndAdd(lift.goTo(2100))
                .stopAndAdd(pas.deposit())
                .waitSeconds(1)
                .stopAndAdd(arm.armBackDeposit())
                .waitSeconds(1)
                .stopAndAdd(claw.openClaw())
                .waitSeconds(0.5)
                .stopAndAdd(arm.armTransfer())
                .stopAndAdd(lift.goTo(0))
                .waitSeconds(1)




                //intake sample 3

                .stopAndAdd(arm.armIntake())
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-60,-51.5,Math.toRadians(90)),Math.toRadians(90))
                .stopAndAdd(horizontalExtendo.goToFront())
                .stopAndAdd(pas.intake())
                .waitSeconds(1)
                .stopAndAdd(claw.closeClaw())
                .waitSeconds(0.7)
                .stopAndAdd(arm.armTransfer())
                .stopAndAdd(horizontalExtendo.goToBack())


                //deposit sample 3
                .turnTo(Math.toRadians(45))
                .setTangent(Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-54.5, -53), Math.toRadians(45))
                .stopAndAdd(lift.goTo(2100))
                .stopAndAdd(pas.deposit())
                .waitSeconds(1)
                .stopAndAdd(arm.armBackDeposit())
                .waitSeconds(1)
                .stopAndAdd(claw.openClaw())
                .waitSeconds(0.5)
                .stopAndAdd(arm.armTransfer())
                .stopAndAdd(lift.goTo(0))
                .waitSeconds(1)


                //intake sample 4


                .stopAndAdd(arm.armIntake())
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-59.4, -49.5,Math.toRadians(122)),Math.toRadians(90))
                .stopAndAdd(horizontalExtendo.goToFront())
                .stopAndAdd(pas.intake())
                .waitSeconds(1)
                .stopAndAdd(claw.closeClaw())
                .waitSeconds(0.7)
                .stopAndAdd(arm.armTransfer())
                .stopAndAdd(horizontalExtendo.goToBack())
                .waitSeconds(1)


                //deposit sample 4
                 .turnTo(Math.toRadians(45))
                .stopAndAdd(pas.deposit())
                .stopAndAdd(lift.goTo(2100))
                .setTangent(Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-54.5, -53), Math.toRadians(45))
                .waitSeconds(1)
                .stopAndAdd(arm.armBackDeposit())
                .waitSeconds(1)
                .stopAndAdd(claw.openClaw())
                .waitSeconds(0.5)
                .stopAndAdd(arm.armTransfer())
                .stopAndAdd(lift.goTo(0))
                .waitSeconds(1)
/*
                //park or go pick up sample
                .splineTo(new Vector2d(-29,-8.7),Math.toRadians(0))

                //deposit fifth sample?
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-53.5,-53.5,Math.toRadians(45)),Math.toRadians(-90))


                //park or go pick up sample
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-29,-8.7,Math.toRadians(0)),Math.toRadians(0))
*/
                .build();


        // actions that need to happen on init; for instance, a claw tightening.
        Actions.runBlocking(claw.closeClaw());
        Actions.runBlocking(arm.armTransfer());
        Actions.runBlocking(pas.intake());
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