package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.MainConfig;
import org.firstinspires.ftc.teamcode.robotcorelib.opmode.OpModePipeline;
import org.firstinspires.ftc.teamcode.robotcorelib.robot.Robot;
import org.firstinspires.ftc.teamcode.robotcorelib.util.RobotRunMode;

@TeleOp(name = "SpecBackTest")
public class SpecBackTest extends OpModePipeline {
    MainConfig subsystems = new MainConfig(); // Initialize MainConfig

    @Override
    public void init() {   //this stuff is needed for all teleop functions so dont question it
        super.subsystems = subsystems;
        runMode = RobotRunMode.TELEOP;
        telemetry.update();
        super.init();
    }

    boolean clawOpen = false;
    private boolean prevClickrB;


    @Override
    public void loop() {
        Robot.update();
        subsystems.lift2.update();

        subsystems.depositBack(1500, clawOpen);

        clawOpen = gamepad1.right_bumper && !prevClickrB;

        // Update previous click states
        prevClickrB = gamepad1.right_bumper;



        telemetry.update();
    }
}
