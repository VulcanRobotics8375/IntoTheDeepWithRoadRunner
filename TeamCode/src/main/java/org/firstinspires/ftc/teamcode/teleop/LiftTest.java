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


    @Override
    public void init() {
        super.subsystems = subsystems;
        runMode = RobotRunMode.TELEOP;
        telemetry.update();
        super.init();

    }

    @Override
    public void loop() {
        Robot.update();

        // Run lift using the controller input
        double manualLift = -gamepad2.left_stick_y;      // Manual lift control

        gp1aClick = gamepad1.a && gp1aPrev;
        gp1aPrev = gamepad1.a;
        subsystems.lift2.update();
        subsystems.lift2.run(manualLift); // Update lift with gamepad inputs

        telemetry.addData("Joystick Input", manualLift);

        if(gp1aClick){
            subsystems.lift2.setPos(1000);
        }

        subsystems.lift2.telemetry(telemetry);
        telemetry.update();
    }
}
