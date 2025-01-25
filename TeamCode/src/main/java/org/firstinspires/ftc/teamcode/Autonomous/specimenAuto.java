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
import com.qualcomm.robotcore.hardware.CRServo;
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
        private double kP = .02;
        private double kD = .0003;

        public Lift(HardwareMap hardwareMap) {
            liftMotorLeft = hardwareMap.get(DcMotorEx.class, "liftMotorLeft");
            liftMotorRight = hardwareMap.get(DcMotorEx.class, "liftMotorRight");

            liftMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            liftMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            liftMotorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            liftMotorRight.setDirection(DcMotorSimple.Direction.REVERSE);

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

        private final double midPosLeft = 0.704;
        private final double midPosRight = 0.553;


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

        public class goToMid implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                linkServoLeft.setPosition(midPosLeft);
                linkServoRight.setPosition(midPosRight);
                return false;
            }
        }
        public Action mid() {
            return new goToMid();
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

        private final double INTAKE_POSITION = 0.073;


        private final double DEPOSIT_POSITION = 0.3;


        private final double TRANSFER_POSITION = 0.3739;



        public Arm(HardwareMap hardwareMap) {
            armServoLeft = hardwareMap.get(Servo.class, "armServoLeft");
            armServoRight = hardwareMap.get(Servo.class, "armServoRight");

        }

        public class Intake implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                armServoRight.setPosition(INTAKE_POSITION);
                armServoLeft.setPosition(INTAKE_POSITION);
                return false;
            }
        }
        public Action intake() {
            return new Intake();
        }



        public class Deposit implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                armServoRight.setPosition(DEPOSIT_POSITION);
                armServoLeft.setPosition(DEPOSIT_POSITION);
                return false;
            }
        }
        public Action deposit() {
            return new Deposit();
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

    public class Claw {
        private Servo claw;
        private CRServo leftSpinner;
        private CRServo rightSpinner;

        public Claw(HardwareMap hardwareMap) {

            claw = hardwareMap.get(Servo.class, "clawServo");
            leftSpinner = hardwareMap.get(CRServo.class, "leftSpinner");
            rightSpinner = hardwareMap.get(CRServo.class, "rightSpinner");
        }

        public class CloseClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(0.2910);
                leftSpinner.setPower(0);
                rightSpinner.setPower(0);
                return false;
            }
        }

        public Action closeClaw() {
            return new CloseClaw();
        }

        public class OpenClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(0.2);
                leftSpinner.setPower(0);
                rightSpinner.setPower(0);
                return false;
            }
        }

        public Action openClaw() {
            return new OpenClaw();
        }


        public class SampleIntake implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(0.2);
                leftSpinner.setPower(1);
                rightSpinner.setPower(-1);
                return false;
            }
        }

        public Action sampleIntake() {
            return new SampleIntake();
        }

    }
    //end of subsystem classes


    @Override
    public void runOpMode() {
        //set starting position
        Pose2d initialPose =new Pose2d(8, -59, Math.toRadians(90));

        //initialize subsystems
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Claw claw = new Claw(hardwareMap);
        HorizontalExtendo horizontalExtendo = new HorizontalExtendo(hardwareMap);
        Lift lift = new Lift(hardwareMap);
        Arm arm = new Arm(hardwareMap);


        Action action1 = drive.actionBuilder(initialPose)
                // preload depo spec
                .stopAndAdd(lift.goTo(820))
                .stopAndAdd(horizontalExtendo.goToFront())
                .stopAndAdd(arm.deposit())
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(8,-38),Math.toRadians(90))
                .waitSeconds(0.25)
                .stopAndAdd(claw.openClaw())
                .waitSeconds(0.2)
                .stopAndAdd(horizontalExtendo.mid())
                .stopAndAdd(lift.goTo(690))




                //give first sample to human

                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(40,-45,Math.toRadians(68)),Math.toRadians(0))
                .stopAndAdd(lift.goTo(0))
                .stopAndAdd(claw.sampleIntake())
                .stopAndAdd(arm.intake())
                .stopAndAdd(horizontalExtendo.goToFront())
                .waitSeconds(0.7)
                .stopAndAdd(claw.closeClaw())
                .waitSeconds(0.5)
                .turnTo(Math.toRadians(-60))
                .waitSeconds(0.25)
                .stopAndAdd(claw.openClaw())
                .waitSeconds(0.25)
                .stopAndAdd(horizontalExtendo.mid())


                //give second sample to human
                .setTangent(Math.toRadians(20))
                .splineToLinearHeading(new Pose2d(47,-41.5,Math.toRadians(52)),Math.toRadians(0))
                .stopAndAdd(claw.sampleIntake())

                .stopAndAdd(arm.intake())
                .stopAndAdd(horizontalExtendo.goToFront())
                .waitSeconds(0.7)
                .stopAndAdd(claw.closeClaw())
                .waitSeconds(0.5)
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(47,-42.5,Math.toRadians(-90)),Math.toRadians(-90))
                .waitSeconds(0.5)
                .stopAndAdd(claw.openClaw())
                .waitSeconds(0.5)



                //pick up second spec
                .stopAndAdd(lift.goTo(630))
                .waitSeconds(1)
                .stopAndAdd(claw.closeClaw())
                .waitSeconds(0.3)
                .stopAndAdd(horizontalExtendo.mid())


                //deposit second spec
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(6,-47,Math.toRadians(90)),Math.toRadians(180))
                .stopAndAdd(lift.goTo(820))
                .stopAndAdd(horizontalExtendo.goToFront())
                .stopAndAdd(arm.deposit())
                .waitSeconds(0.5)
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(6,-38),Math.toRadians(90))
                .waitSeconds(0.5)
                .stopAndAdd(claw.openClaw())
                .waitSeconds(0.2)
                .stopAndAdd(horizontalExtendo.mid())
                .stopAndAdd(lift.goTo(630))



                //pick up third spec
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(35,-50,Math.toRadians(-90)),Math.toRadians(-90))
                .stopAndAdd(arm.intake())

                .splineToConstantHeading(new Vector2d(35,-54),Math.toRadians(180))

                .waitSeconds(1)
                .stopAndAdd(claw.closeClaw())
                .waitSeconds(0.3)
                .stopAndAdd(lift.goTo(820))
                .stopAndAdd(arm.deposit())




                //deposit third spec

                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(4,-47,Math.toRadians(90)),Math.toRadians(180))
                .stopAndAdd(horizontalExtendo.goToFront())
                .waitSeconds(0.5)
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(4,-38),Math.toRadians(90))
                .waitSeconds(0.5)
                .stopAndAdd(claw.openClaw())
                .waitSeconds(0.2)
                .stopAndAdd(horizontalExtendo.mid())
                .stopAndAdd(arm.intake())
                .stopAndAdd(lift.goTo(630))


                /*
                                          //pick up fourth spec

                                          .setTangent(Math.toRadians(-50))
                                          .splineToLinearHeading(new Pose2d(35,-53,Math.toRadians(-90)),Math.toRadians(-90))



                                          //deposit fourth spec

                                          .setTangent(Math.toRadians(180))
                                          .splineToLinearHeading(new Pose2d(10,-40,Math.toRadians(90)),Math.toRadians(180))
                                          .setTangent(Math.toRadians(90))
                                          .splineToConstantHeading(new Vector2d(10,-36),Math.toRadians(90))
                                          */



                .build();


        // actions that need to happen on init; for instance, a claw tightening.
        Actions.runBlocking(claw.closeClaw());
        Actions.runBlocking(arm.armTransfer());
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