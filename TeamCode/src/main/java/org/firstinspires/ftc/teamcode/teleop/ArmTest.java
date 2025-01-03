package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.MainConfig;
import org.firstinspires.ftc.teamcode.robotcorelib.opmode.OpModePipeline;
import org.firstinspires.ftc.teamcode.robotcorelib.util.RobotRunMode;

@TeleOp(name = "ArmTest")
public class ArmTest extends OpModePipeline {
    MainConfig subsystems = new MainConfig(); // Initialize MainConfig

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
        subsystems.arm.manualArmCode(gamepad1.right_stick_y);
        telemetry.addData("left servo position", subsystems.arm.getCurrentLeftPos());
        telemetry.addData("right servo position", subsystems.arm.getCurrentRightPos());

        telemetry.update();
    }
}
