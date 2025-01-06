package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.MainConfig;
import org.firstinspires.ftc.teamcode.robotcorelib.opmode.OpModePipeline;
import org.firstinspires.ftc.teamcode.robotcorelib.robot.Robot;
import org.firstinspires.ftc.teamcode.robotcorelib.util.RobotRunMode;
import org.firstinspires.ftc.teamcode.robotcorelib.util.Switch;


/*

map of button presses

gamepad1 left joystick -> drivetrain translation
gamepad1 right joystick -> drivetrain rotation

gamepad1 right bumper -> claw
gamepad1 left bumper -> claw Spin to 90 degrees
gamepad1 left trigger -> spin claw clockwise
gamepad1 right trigger -> spin claw counterclockwise

gamepad2 a button -> sample intake position
gamepad2 x button -> specimen intake position
gamepad2 y button -> deposit back
                                  if gamepad1 x-> sample depo pos
                                  if gamepad1  circle -> spec depo pos
gamepad2 b button -> deposit front
                                  if gamepad1 x-> sample depo pos
                                  if gamepad1  circle -> spec depo pos

 */



@TeleOp(name = "MainOpmode")
public class MainOpmode extends OpModePipeline {
    MainConfig subsystems = new MainConfig();

    private RobotState robotState = RobotState.TRANSFERPOS;  //set first state as transfer
    private Switch stateSwitch = new Switch();

    //set lift positions
    private final int highBucket = 3000; //set location of the high bucket
    private final int highBar = 1900; //set location as high bar

    //define buttons for claw
    private boolean clawPrev = false;
    private boolean claw = false; //claw boolean: gamepad1.right bumper
    private double clawRollIncCCW = 0.0; //counterclockwise incrementer: gamepad1.left trigger
    private double clawRollIncCW = 0.0; //clockwise incrementer: gamepad1.right trigger
    private boolean claw90degTurnPrev = false;
    private boolean claw90degTurn = false; //claw 90 degree turn: gamepad1.left bumper

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
    private boolean gp1bPrev = false;
    private boolean gp1bClick = false;
    private boolean gp1hangPrev = false;
    private boolean gp1hangClick = false;

      boolean gp1bholding = false;
    boolean gp1aholding = false;


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
        subsystems.lift2.manualControl(-gamepad2.left_stick_y); //initialize gamepad2 left joystick as manual input for lift
        subsystems.horizontalExtendo.run(-gamepad2.right_stick_y); //initialize gamepad2 right joystick as manual input for horizextendo

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
        gp1bClick = gamepad1.b && gp1bPrev;
        gp1bPrev = gamepad1.b;

        gp1hangClick = gamepad1.dpad_down && gp1hangPrev;
        gp1hangPrev = gamepad1.dpad_down;



        //assign buttons for claw on init
        claw = gamepad1.right_bumper && !clawPrev; //set claw status
        clawPrev = gamepad1.right_bumper;
        clawRollIncCCW = gamepad1.left_trigger; //same
        clawRollIncCW = gamepad1.right_trigger; //same
        claw90degTurn = gamepad1.left_bumper && !claw90degTurnPrev;
        claw90degTurnPrev = gamepad1.left_bumper;

        switch (robotState) {

            case SPECIMANINTAKE: //what do do when we are intaking a speciman
                subsystems.specimenIntake(claw); //manualControl code that intakes specimen, see MainConfig to view all methods
                telemetry.addData("state", "SPECIMAN INTAKE");

                if(aClick){
                    robotState = RobotState.SAMPLEINTAKE;
                }
                else if(bClick){
                    robotState = RobotState.DEPOSITFRONT;
                }
                else if(xClick){
                    robotState = RobotState.TRANSFERPOS;
                }
                else if(yClick){
                    robotState = RobotState.DEPOSITBACK;
                }
                break;

            case SAMPLEINTAKE: 
                subsystems.sampleIntake(clawRollIncCCW, clawRollIncCW,  claw90degTurn, claw); //manualControl code for intaking samples
                telemetry.addData("state", "SAMPLE INTAKE");

                if(aClick){
                    robotState = RobotState.TRANSFERPOS;
                }
                else if(bClick){
                    robotState = RobotState.DEPOSITFRONT;
                }
                else if(xClick){
                    robotState = RobotState.SPECIMANINTAKE;
                }
                else if(yClick){
                    robotState = RobotState.DEPOSITBACK;
                }
                break;

            case DEPOSITFRONT:
               if(gp1aClick || gp1aholding){
                    gp1aholding = true;
                    gp1bholding = false;

                    subsystems.depositFront(highBucket, claw);
                    telemetry.addData("Deposit", "FRONT SAMPLE");
               }
               if(gp1bClick || gp1bholding) { //specimen deposit front
                    gp1bholding = true;
                    gp1aholding = false;
                    subsystems.depositFront(highBar, claw);
                    telemetry.addData("Deposit", "FRONT SPECIMEN");
                }
                telemetry.addData("state", "DEPOSITFRONT");

                if(aClick){
                    robotState = RobotState.SAMPLEINTAKE;
                    gp1bholding = false;
                    gp1aholding = false;
                }
                else if(bClick){
                    robotState = RobotState.TRANSFERPOS;
                     gp1bholding = false;
                    gp1aholding = false;
                }
                else if(xClick){
                    robotState = RobotState.SPECIMANINTAKE;
                     gp1bholding = false;
                    gp1aholding = false;
                }
                else if(yClick){
                    robotState = RobotState.DEPOSITBACK;
                     gp1bholding = false;
                    gp1aholding = false;
                }

                break;


            case DEPOSITBACK:
                if(gp1aClick || gp1aholding) { //specimen deposit back
                    gp1aholding=true;
                    gp1bholding=false;
                    subsystems.depositBack(highBucket, claw);
                    telemetry.addData("Deposit", "BACK SPECIMEN");
                }
                if(gp1bClick || gp1bholding){ //sample deposit back
                    gp1bholding=true;
                    gp1aholding=false;
                    subsystems.depositBack(highBar, claw);
                    telemetry.addData("Deposit", "BACK SAMPLE");

                }
                telemetry.addData("state", "DEPOSITBACK");

                if(aClick){
                    robotState = RobotState.SAMPLEINTAKE;
                    gp1bholding = false;
                    gp1aholding = false;
                }
                else if(bClick){
                    robotState = RobotState.DEPOSITFRONT;
                    gp1bholding = false;
                    gp1aholding = false;
                }
                else if(xClick){
                    robotState = RobotState.SPECIMANINTAKE;
                    gp1bholding = false;
                    gp1aholding = false;
                }
                else if(yClick){
                    robotState = RobotState.TRANSFERPOS;
                    gp1bholding = false;
                    gp1aholding = false;
                }

                break;

            case TRANSFERPOS:
                telemetry.addData("state", "TRANSFER POSITION");
                subsystems.transferPos();

                if(aClick){
                    robotState = RobotState.SAMPLEINTAKE;
                }
                else if(bClick){
                    robotState = RobotState.DEPOSITFRONT;
                }
                else if(xClick){
                    robotState = RobotState.SPECIMANINTAKE;
                }
                else if(yClick){
                    robotState = RobotState.DEPOSITBACK;
                }
                else if(gp1hangClick){
                    robotState = RobotState.HANG;
                }
                break;

            case HANG:
                telemetry.addData("state", "HANG");
                subsystems.hang();
                if(gp1hangClick){
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
        SPECIMANINTAKE,
        DEPOSITFRONT,
        DEPOSITBACK,
        HANG,
    }


}
