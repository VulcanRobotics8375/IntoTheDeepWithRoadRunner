package org.firstinspires.ftc.teamcode.teleop;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.MainConfig;
import org.firstinspires.ftc.teamcode.robotcorelib.opmode.OpModePipeline;
import org.firstinspires.ftc.teamcode.robotcorelib.robot.Robot;
import org.firstinspires.ftc.teamcode.robotcorelib.util.RobotRunMode;


@TeleOp(name = "LiftTest")

public class LiftTest extends OpModePipeline{
    MainConfig subsystems = new MainConfig(); // Initialize MainConfig

    private boolean gp1aPrev = false;
    private boolean gp1aClick = false;


    private boolean gp1bPrev = false;
    private boolean gp1bClick = false;


    private boolean gp1xPrev = false;
    private boolean gp1xClick = false;

    @Override
    public void init() {
        super.subsystems = subsystems;
        runMode = RobotRunMode.TELEOP;
        telemetry.update();
        super.init();

    }

    private boolean liftHoldingA =false;
    private boolean liftHoldingB =false;

    @Override
    public void loop() {
        Robot.update();

        // Run lift using the controller input
        double manualLift = -gamepad1.left_stick_y;      // Manual lift control

        if(gp1aClick || liftHoldingA){
            subsystems.lift2.setPos(2000);
            liftHoldingA = true;
            liftHoldingB = false;

        }
        if(gp1bClick || liftHoldingB){
            subsystems.lift2.setPos(1000);
            liftHoldingB = true;
            liftHoldingA = false;

        }


        gp1aClick = gamepad1.a && gp1aPrev;
        gp1aPrev = gamepad1.a;

        gp1bClick = gamepad1.b && gp1bPrev;
        gp1bPrev = gamepad1.b;

        subsystems.lift2.manualControl(manualLift); // Update lift with gamepad inputs

        subsystems.lift2.update();

        telemetry.addData("Joystick Input", manualLift);

        if(gp1aClick){
            subsystems.lift2.setPos(1000);
        }

        subsystems.lift2.telemetry(telemetry);
        telemetry.update();
    }
}
