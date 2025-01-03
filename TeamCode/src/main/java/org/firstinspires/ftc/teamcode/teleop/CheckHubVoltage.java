package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.MainConfig;
import org.firstinspires.ftc.teamcode.robotcorelib.opmode.OpModePipeline;
import org.firstinspires.ftc.teamcode.robotcorelib.util.RobotRunMode;

@TeleOp(name = "Check Hub Voltage")

public class CheckHubVoltage extends OpModePipeline {

    public void init() {
        runMode = RobotRunMode.TELEOP;
        subsystems = new MainConfig();
        super.init();
    }
    @Override
    public void loop() {
            double hubVoltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
            telemetry.addData("Hub Voltage", hubVoltage);
            telemetry.update();

    }
}
