package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.MainConfig;
import org.firstinspires.ftc.teamcode.robotcorelib.opmode.OpModePipeline;
import org.firstinspires.ftc.teamcode.robotcorelib.util.RobotRunMode;

@TeleOp(name = "SamBackTest")
public class SamBackTest extends OpModePipeline {
    MainConfig subsystems = new MainConfig(); // Initialize MainConfig
    boolean clawOpen = false;
    private boolean prevClickrB;



    @Override
    public void init() {   //this stuff is needed for all teleop functions so dont question it
        super.subsystems = subsystems;
        runMode = RobotRunMode.TELEOP;
        telemetry.update();
        super.init();
    }

    @Override
    public void loop() {
        Robot.update();
        subsystems.depositBack(2800, clawOpen);


        clawOpen = gamepad1.right_bumper && !prevClickrB;

        // Update previous click states
        prevClickrB = gamepad1.right_bumper;

        telemetry.update();
    }
}
