package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.MainConfig;
import org.firstinspires.ftc.teamcode.robotcorelib.opmode.OpModePipeline;
import org.firstinspires.ftc.teamcode.robotcorelib.robot.Robot;
import org.firstinspires.ftc.teamcode.robotcorelib.util.RobotRunMode;

@TeleOp(name = "SamFrontTest")
public class SamFrontTest extends OpModePipeline {
    MainConfig subsystems = new MainConfig(); // Initialize MainConfig
    boolean clawOpen = false;
    private boolean prevClickrB = false;



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
        subsystems.lift2.update();
        subsystems.depositFront(2800, clawOpen);

        // Toggle clawOpen on button press
        if (gamepad1.right_bumper && !prevClickrB) {
            clawOpen = !clawOpen;
        }
        prevClickrB = gamepad1.right_bumper;


        // Add telemetry for debugging
        telemetry.addData("Claw Open", clawOpen);
        telemetry.addData("Previous Click", prevClickrB);
        telemetry.update();
    }

}
