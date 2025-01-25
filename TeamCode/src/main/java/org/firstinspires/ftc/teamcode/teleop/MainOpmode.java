package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.MainConfig;
import org.firstinspires.ftc.teamcode.robotcorelib.opmode.OpModePipeline;
import org.firstinspires.ftc.teamcode.robotcorelib.robot.Robot;
import org.firstinspires.ftc.teamcode.robotcorelib.util.RobotRunMode;


@TeleOp(name = "MainOpmode")
public class MainOpmode extends OpModePipeline {
    MainConfig subsystems = new MainConfig();

    private RobotState robotState = RobotState.TRANSFERPOS;  //set first state as transfer

    //set lift positions
    private final int highBucket = 2200; //set location of the high bucket
    private final int highBar = 780; //set location as high bar

    //define buttons for claw
    private boolean clawPrev = false;
    private boolean claw = false; //claw boolean: gamepad1.right bumper
    private double horizBack = 0.0; //horizontal extendo back manual: gamepad1.left trigger
    private double horizForward = 0.0; //horizontal extendo forward manual: gamepad1.right trigger


    private boolean aPrev = false;
    private boolean aClick = false;
    private boolean bPrev = false;
    private boolean bClick = false;
    private boolean xPrev = false;
    private boolean xClick = false;
    private boolean yPrev = false;
    private boolean yClick = false;

    private boolean gp1aPrev = false;
    private boolean gp1aClick = false;


    private boolean gp2hangPrev = false;
    private boolean gp2hangClick = false;
    private boolean exitIntake = false;
    private boolean exitIntakePrev = false;


    private boolean gp2RB = false;
    private boolean gp2RBPrev = false;
    private boolean gp2LB = false;
    private boolean gp2LBPrev = false;

    private boolean holdingLow = false;
    private boolean holdingHigh = false;
    @Override
    public void init() {
        super.subsystems = subsystems; //first 4 lines needed always
        runMode = RobotRunMode.TELEOP; //initialize robot for teleop
        telemetry.update(); //initialize telemetry
        super.init();//begin opmode

        // Start in transfer position
        subsystems.transferPos();
    }

    @Override
    public void loop() {
        Robot.update(); //reset all the clutter in robot


        // Run drivetrain using the controller input
        subsystems.drivetrain.mecanumDrive(
                -gamepad1.left_stick_y,  // Forward/backward
                gamepad1.left_stick_x,   // Strafe
                gamepad1.right_stick_x   // Rotation
        );


        subsystems.lift2.update();

        horizBack = gamepad1.left_trigger;
        horizForward = gamepad1.right_trigger;

        subsystems.horizontalExtendo.run(horizBack,horizForward);

        aClick = gamepad2.a && !aPrev;
        aPrev = gamepad2.a;
        bClick = gamepad2.b && !bPrev;
        bPrev = gamepad2.b;
        xClick = gamepad2.x && !xPrev;
        xPrev = gamepad2.x;
        yClick = gamepad2.y && !yPrev;
        yPrev = gamepad2.y;

        gp1aClick = gamepad1.a && gp1aPrev;
        gp1aPrev = gamepad1.a;

        gp2hangClick = gamepad2.dpad_down && gp2hangPrev;
        gp2hangPrev = gamepad2.dpad_down;

        exitIntake = gamepad1.dpad_right && exitIntakePrev;
        exitIntakePrev = gamepad1.dpad_right;

        gp2LB = gamepad2.left_bumper && gp2LBPrev;
        gp2LBPrev = gamepad2.left_bumper;

        gp2RB = gamepad2.right_bumper && gp2RBPrev;
        gp2RBPrev = gamepad2.right_bumper;

        //assign buttons for claw
        claw = gamepad1.right_bumper && !clawPrev; //set claw status
        clawPrev = gamepad1.right_bumper;


        switch (robotState) {

            case SPECIMANINTAKE:
                subsystems.specimenIntake(claw); //manualControl code that intakes specimen, see MainConfig to view all methods
                telemetry.addData("state", "SPECIMAN INTAKE");

                if(aClick){
                    robotState = RobotState.SAMPLEINTAKEREADY;
                }
                else if(bClick){
                    robotState = RobotState.SAMPLEDEPOSIT;
                }
                else if(xClick || gp1aClick){
                    robotState = RobotState.TRANSFERPOS;
                }
                else if(yClick){
                    robotState = RobotState.SPECDEPOSIT;
                }
                break;


            case SAMPLEINTAKEREADY:

                subsystems.sampleIntakeReady();

                telemetry.addData("state", "SAMPLE INTAKE HOVER");

                if(claw){
                    robotState = RobotState.SAMPLEINTAKE;
                }
                if(aClick){
                    robotState = RobotState.TRANSFERPOS;
                }
                else if(bClick){
                    robotState = RobotState.SAMPLEDEPOSIT;
                }
                else if(xClick){
                    robotState = RobotState.SPECIMANINTAKE;
                }
                else if(yClick){
                    robotState = RobotState.SPECDEPOSIT;
                }

                break;

            case SAMPLEINTAKE:
                subsystems.sampleIntake(claw);
                if(exitIntake){
                    robotState = RobotState.SAMPLEINTAKEREADY;
                }
                else if(aClick || gp1aClick){
                    robotState = RobotState.TRANSFERPOS;
                }

                telemetry.addData("state", "SAMPLE INTAKE");

                break;

            case SAMPLEDEPOSIT:

                if(gp2LB || holdingLow){
                    subsystems.depositFront(0, claw);
                    holdingLow = true;
                    holdingHigh = false;
                }
                if(gp2RB || holdingHigh){
                    subsystems.depositFront(highBucket, claw);
                    holdingLow = false;
                    holdingHigh = true;
                }



                telemetry.addData("state", "SAMPLE DEPOSIT");

                if(aClick){
                    robotState = RobotState.SAMPLEINTAKEREADY;
                    holdingLow = false;
                    holdingHigh = false;
                }
                else if(bClick){
                    robotState = RobotState.TRANSFERPOS;
                    holdingLow = false;
                    holdingHigh = false;
                }
                else if(xClick){
                    robotState = RobotState.SPECIMANINTAKE;
                    holdingLow = false;
                    holdingHigh = false;
                }
                else if(yClick){
                    robotState = RobotState.SPECDEPOSIT;
                    holdingLow = false;
                    holdingHigh = false;

                }

                break;


            case SPECDEPOSIT:

                    subsystems.specDepositFront(highBar,claw);
                    telemetry.addData("Deposit", "SPECIMEN");


                if(aClick){
                    robotState = RobotState.SAMPLEINTAKEREADY;
                }
                else if(bClick){
                    robotState = RobotState.SAMPLEDEPOSIT;
                }
                else if(xClick){
                    robotState = RobotState.SPECIMANINTAKE;
                }
                else if(yClick){
                    robotState = RobotState.TRANSFERPOS;
                }

                break;

            case TRANSFERPOS:
                telemetry.addData("state", "TRANSFER POSITION");
                subsystems.transferPos();

                if(aClick){
                    robotState = RobotState.SAMPLEINTAKEREADY;
                }
                else if(bClick){
                    robotState = RobotState.SAMPLEDEPOSIT;
                }
                else if(xClick){
                    robotState = RobotState.SPECIMANINTAKE;
                }
                else if(yClick){
                    robotState = RobotState.SPECDEPOSIT;
                }
                else if(gp2hangClick){
                    robotState = RobotState.HANG;
                }
                break;

            case HANG:
                telemetry.addData("state", "HANG");
                subsystems.hang();

                if(gp2hangClick){
                    robotState = RobotState.TRANSFERPOS;
                }
                break;

        }
        telemetry.addData("Claw Open", claw);
        telemetry.addData("Previous Click", clawPrev);

        telemetry.update();
    }

    public enum RobotState {
        TRANSFERPOS,
        SAMPLEINTAKE,
        SAMPLEINTAKEREADY,
        SPECIMANINTAKE,
        SAMPLEDEPOSIT,
        SPECDEPOSIT,
        HANG,
    }


}
