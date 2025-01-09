package org.firstinspires.ftc.teamcode.Autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


@Config
@Autonomous(name = "specimen auto", group = "Autonomous")
public class specimenAuto extends LinearOpMode {

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
        private static final double PITCH_SPECIMAN = 0.5;


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

        public class SpecIntake implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                clawPitch.setPosition(PITCH_SPECIMAN);

                clawSpin.setPosition(CLAW_NORMAL_POS);
                return false;
            }
        }
        public Action specintake() {
            return new SpecIntake();
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
                claw.setPosition(0.27);
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

    public class Sweeper {
        private Servo sweeper;

        public Sweeper(HardwareMap hardwareMap) {
            sweeper = hardwareMap.get(Servo.class, "sweeper");
        }

        public class SweeperUp implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                sweeper.setPosition(0.97);
                return false;
            }
        }
        public Action sweeperUp() {
            return new SweeperUp();
        }

        public class SweeperDown implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                sweeper.setPosition(0.359);
                return false;
            }
        }
        public Action sweeperDown() {
            return new SweeperDown();
        }
    }


    //end of subsystem classes


    @Override
    public void runOpMode() {
        //set starting position
        Pose2d initialPose =new Pose2d(5, -63, Math.toRadians(90));

        //initialize subsystems
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Claw claw = new Claw(hardwareMap);
        HorizontalExtendo horizontalExtendo = new HorizontalExtendo(hardwareMap);
        Lift lift = new Lift(hardwareMap);
        Arm arm = new Arm(hardwareMap);
        PitchandSpin pas = new PitchandSpin(hardwareMap);
        Sweeper sweeper = new Sweeper(hardwareMap);


        Action action1 = drive.actionBuilder(initialPose)

                //deposit preload
                .stopAndAdd(lift.goTo(2600))
                .stopAndAdd(horizontalExtendo.goToFront())
                .stopAndAdd(arm.armIntake())
                .stopAndAdd(pas.intake())
                .lineToY(-38)
                .waitSeconds(1)
                .stopAndAdd(claw.openClaw())
                .waitSeconds(0.5)

                //give first sample to human
                .stopAndAdd(lift.goTo(0))
                .stopAndAdd(horizontalExtendo.goToBack())
                .stopAndAdd(arm.armTransfer())
                .stopAndAdd(sweeper.sweeperDown())
                .strafeToLinearHeading(new Vector2d(25.7,-40),Math.toRadians(30))
                .turnTo(Math.toRadians(-45))

                //give second sample to human
                .strafeToLinearHeading(new Vector2d(35.7,-40),Math.toRadians(30))
                .turnTo(Math.toRadians(-45))

                //give third sample to human
                .strafeToLinearHeading(new Vector2d(44.7,-40),Math.toRadians(30))
                .turnTo(Math.toRadians(-45))
                .stopAndAdd(sweeper.sweeperUp())


                //pick up second speciman on wall
                .stopAndAdd(lift.goTo(1000))
                .stopAndAdd(arm.armIntake())
                .stopAndAdd(pas.deposit())
                .strafeToLinearHeading(new Vector2d(35,-50),Math.toRadians(-90))
                .stopAndAdd(horizontalExtendo.goToFront())
                .waitSeconds(1)
                .stopAndAdd(claw.closeClaw())
                .waitSeconds(0.3)
                .stopAndAdd(lift.goTo(2100))
                .stopAndAdd(arm.armTransfer())
                .stopAndAdd(horizontalExtendo.goToBack())



                //deposit second speciman
                .strafeTo(new Vector2d(5,-37))
                .stopAndAdd(arm.armBackDeposit())
                .stopAndAdd(lift.goTo(1800))
                .waitSeconds(0.5)
                .stopAndAdd(claw.openClaw())
                .waitSeconds(0.5)
                .stopAndAdd(lift.goTo(0))

                //pick up third speciman on floor
                .strafeToLinearHeading(new Vector2d(16,-46.5),Math.toRadians(-45))
                .stopAndAdd(horizontalExtendo.goToFront())
                .stopAndAdd(arm.armIntake())
                .stopAndAdd(pas.specintake())
                .waitSeconds(0.5)
                .stopAndAdd(claw.closeClaw())
                .waitSeconds(0.5)
                .stopAndAdd(arm.armBackDeposit())
                .stopAndAdd(pas.deposit())
                .stopAndAdd(lift.goTo(2100))



                //deposit third speciman
                .strafeToLinearHeading(new Vector2d(5,-37),Math.toRadians(-90))
                .stopAndAdd(lift.goTo(1800))
                .waitSeconds(0.5)
                .stopAndAdd(claw.openClaw())
                .waitSeconds(0.5)
                .stopAndAdd(lift.goTo(0))



                //pick up fourth speciman on floor
                .strafeToLinearHeading(new Vector2d(16,-46.5),Math.toRadians(-45))
                .stopAndAdd(horizontalExtendo.goToFront())
                .stopAndAdd(arm.armIntake())
                .stopAndAdd(pas.specintake())
                .waitSeconds(0.5)
                .stopAndAdd(claw.closeClaw())
                .waitSeconds(0.5)
                .stopAndAdd(arm.armBackDeposit())
                .stopAndAdd(pas.deposit())
                .stopAndAdd(lift.goTo(2100))


                //deposit fourth speciman
                .strafeToLinearHeading(new Vector2d(5,-37),Math.toRadians(-90))
                .stopAndAdd(lift.goTo(1800))
                .waitSeconds(0.5)
                .stopAndAdd(claw.openClaw())
                .waitSeconds(0.5)
                .stopAndAdd(lift.goTo(0))

                //pick up fifth speciman
                .strafeToLinearHeading(new Vector2d(16,-46.5),Math.toRadians(-45))
                .stopAndAdd(horizontalExtendo.goToFront())
                .stopAndAdd(arm.armIntake())
                .stopAndAdd(pas.specintake())
                .waitSeconds(0.5)
                .stopAndAdd(claw.closeClaw())
                .waitSeconds(0.5)
                .stopAndAdd(arm.armBackDeposit())
                .stopAndAdd(pas.deposit())
                .stopAndAdd(lift.goTo(2100))

                //deposit fifth speciman
                .strafeToLinearHeading(new Vector2d(5,-37),Math.toRadians(-90))
                .stopAndAdd(lift.goTo(1800))
                .waitSeconds(0.5)
                .stopAndAdd(claw.openClaw())
                .waitSeconds(0.5)
                .stopAndAdd(lift.goTo(0))

                //park
                .stopAndAdd(horizontalExtendo.goToFront())
                .strafeToLinearHeading(new Vector2d(28.2,-54),Math.toRadians(-45))


                .build();


        // actions that need to happen on init; for instance, a claw tightening.
        Actions.runBlocking(claw.closeClaw());
        Actions.runBlocking(arm.armTransfer());
        Actions.runBlocking(pas.intake());
        Actions.runBlocking(horizontalExtendo.goToBack());
        Actions.runBlocking(sweeper.sweeperUp());




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