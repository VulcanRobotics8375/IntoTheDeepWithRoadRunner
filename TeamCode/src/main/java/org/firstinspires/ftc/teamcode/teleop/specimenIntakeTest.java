package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.MainConfig;
import org.firstinspires.ftc.teamcode.robotcorelib.opmode.OpModePipeline;
import org.firstinspires.ftc.teamcode.robotcorelib.robot.Robot;
import org.firstinspires.ftc.teamcode.robotcorelib.util.RobotRunMode;
@TeleOp (name = "specimenIntakeTest")
public class specimenIntakeTest extends OpModePipeline {

    boolean clawOpen = false;
    MainConfig subsystems = new MainConfig();


    public void init() {
        runMode = RobotRunMode.TELEOP;
        super.subsystems = subsystems; //first 4 lines needed always
        super.init();
    }

    private boolean prevClickrB;


    @Override
    public void loop() {
        Robot.update();

        subsystems.specimenIntake(clawOpen); //manualControl code for intaking samples


        clawOpen = gamepad1.right_bumper && !prevClickrB;

        // Update previous click states
        prevClickrB = gamepad1.right_bumper;
        telemetry.update();

    }


}