package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.MainConfig;
import org.firstinspires.ftc.teamcode.robotcorelib.opmode.OpModePipeline;
import org.firstinspires.ftc.teamcode.robotcorelib.robot.Robot;
import org.firstinspires.ftc.teamcode.robotcorelib.util.RobotRunMode;
@TeleOp (name = "IntakeTest")
public class sampleIntakeTest extends OpModePipeline {

    boolean clawOpen = false;
    MainConfig subsystems = new MainConfig();

    //define buttons for claw
    private double clawRollIncCCW = 0.01; //counterclockwise incrementer: gamepad1.left trigger
    private double clawRollIncCW = 0.01; //clockwise incrementer: gamepad1.right trigger
    private boolean claw90degTurnPrev = false;
    private boolean claw90degTurn = false; //claw 90 degree turn: gamepad1.left bumper

    public void init() {
        runMode = RobotRunMode.TELEOP;
        super.subsystems = subsystems; //first 4 lines needed always
        super.init();
    }

    private boolean prevClickrB;


    @Override
    public void loop() {
        Robot.update();
        subsystems.lift2.update();


        clawRollIncCCW = gamepad1.left_trigger; //same
        clawRollIncCW = gamepad1.right_trigger; //same

        claw90degTurn = gamepad1.left_bumper && !claw90degTurnPrev;
        claw90degTurnPrev = gamepad1.left_bumper;

        subsystems.sampleIntake(clawRollIncCCW, clawRollIncCW,  claw90degTurn, clawOpen); //manualControl code for intaking samples


        clawOpen = gamepad1.right_bumper && !prevClickrB;

        // Update previous click states
        prevClickrB = gamepad1.right_bumper;
        telemetry.update();

        }


}