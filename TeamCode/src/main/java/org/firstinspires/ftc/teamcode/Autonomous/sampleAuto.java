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

        private final double FRONT_INTAKE_POSITION = 0.20;


        private final double BACK_DEPOSIT_POSITION = 0.6472;


        private final double TRANSFER_POSITION = 0.45;





        public Arm(HardwareMap hardwareMap) {
            armServoLeft = hardwareMap.get(Servo.class, "armServoLeft");
            armServoRight = hardwareMap.get(Servo.class, "armServoRight");

        }

        public class ArmIntakeHover implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                armServoRight.setPosition(FRONT_INTAKE_POSITION+0.05);
                armServoLeft.setPosition(FRONT_INTAKE_POSITION+0.05);
                return false;
            }
        }
        public Action armIntakeHover() {
            return new ArmIntakeHover();
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



        private static final double PITCH_STRAIGHT = 1;
        private static final double PITCH_SAMPLE = 0.6249;



        public PitchandSpin(HardwareMap hardwareMap) {
            clawSpin = hardwareMap.get(Servo.class, "clawSpin");
            clawPitch = hardwareMap.get(Servo.class, "clawPitch");

        }

        public class Deposit implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                clawPitch.setPosition(PITCH_STRAIGHT);
                clawSpin.setPosition(CLAW_NORMAL_POS);
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
                claw.setPosition(0.1545);
                return false;
            }
        }
        public Action closeClaw() {
            return new CloseClaw();
        }

        public class OpenClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(0.448 );
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
        Pose2d initialPose =new Pose2d(-41, -63, Math.toRadians(90));

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
                .stopAndAdd(arm.armBackDeposit())
                .setTangent(Math.toRadians(135))
                .splineToLinearHeading(new Pose2d(-58,-56.5,Math.toRadians(45)),Math.toRadians(225))
                .waitSeconds(0.7)
                .stopAndAdd(pas.deposit())
                .waitSeconds(1)
                .stopAndAdd(claw.openClaw())
                .waitSeconds(1)
                .stopAndAdd(lift.goTo(0))
                .stopAndAdd(pas.intake())
                .stopAndAdd(arm.armIntakeHover()).waitSeconds(0.5)


                //intake sample 2
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-53,-53,Math.toRadians(90)),Math.toRadians(90))
                .waitSeconds(0.2)
                .stopAndAdd(horizontalExtendo.goToFront())
                .waitSeconds(1)
                .stopAndAdd(arm.armIntake())
                .waitSeconds(0.2)
                .stopAndAdd(claw.closeClaw())
                .waitSeconds(0.5)
                .stopAndAdd(horizontalExtendo.goToBack())


                //deposit sample 2
                .stopAndAdd(lift.goTo(2100))
                .stopAndAdd(arm.armBackDeposit())

                .setTangent(Math.toRadians(225))
                .splineToLinearHeading(new Pose2d(-58,-56.5,Math.toRadians(45)),Math.toRadians(90))
                .waitSeconds(0.5)
                .stopAndAdd(pas.deposit())

                .waitSeconds(1)
                .stopAndAdd(claw.openClaw())
                .waitSeconds(0.5)
                .stopAndAdd(lift.goTo(0))
                .stopAndAdd(pas.intake())
                .stopAndAdd(arm.armIntakeHover())
                .waitSeconds(1)

                //intake sample 3
                .setTangent(Math.toRadians(90)).splineToLinearHeading(new Pose2d(-62.5,-53,Math.toRadians(90)),Math.toRadians(90))
                .waitSeconds(0.5)
                .stopAndAdd(horizontalExtendo.goToFront())
                .waitSeconds(1)
                .stopAndAdd(arm.armIntake())
                .waitSeconds(0.3)
                .stopAndAdd(claw.closeClaw())
                .waitSeconds(0.5)
                .stopAndAdd(horizontalExtendo.goToBack())

                //deposit sample 3
                .stopAndAdd(lift.goTo(2100))
                .stopAndAdd(arm.armBackDeposit())
                .setTangent(Math.toRadians(260))
                .splineToLinearHeading(new Pose2d(-58,-56.5,Math.toRadians(45)),Math.toRadians(90))
                .waitSeconds(0.5)
                .stopAndAdd(pas.deposit())
                .waitSeconds(0.3)
                .stopAndAdd(claw.openClaw())
                .waitSeconds(0.5)
                .stopAndAdd(lift.goTo(0))
                .stopAndAdd(pas.intake())
                .stopAndAdd(arm.armIntakeHover())
                .waitSeconds(1)



                //intake sample 4


                .setTangent(Math.toRadians(120))
                .splineToLinearHeading(new Pose2d(-64.4, -51.5,Math.toRadians(110)),Math.toRadians(90))
                .waitSeconds(1)
                .stopAndAdd(horizontalExtendo.goToFront())
                .waitSeconds(1)
                .stopAndAdd(arm.armIntake())
                .waitSeconds(0.2)
                .stopAndAdd(claw.closeClaw())
                .waitSeconds(0.5)
                .stopAndAdd(horizontalExtendo.goToBack())

                .waitSeconds(0.5)


                //deposit sample 4

                .stopAndAdd(lift.goTo(2100))
                .stopAndAdd(arm.armBackDeposit())
                .setTangent(Math.toRadians(300))
                .splineToLinearHeading(new Pose2d(-58,-56.5,Math.toRadians(45)),Math.toRadians(90))

                .waitSeconds(0.5)
                .stopAndAdd(pas.deposit())
                .waitSeconds(0.3)
                .stopAndAdd(claw.openClaw())
                .waitSeconds(0.5)
                .waitSeconds(1)
                .stopAndAdd(lift.goTo(0))
                .stopAndAdd(arm.armTransfer())
                .waitSeconds(1)





                //park or go pick up sample
                .splineToLinearHeading(new Pose2d(-29,-8.7,Math.toRadians(0)),Math.toRadians(0))

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