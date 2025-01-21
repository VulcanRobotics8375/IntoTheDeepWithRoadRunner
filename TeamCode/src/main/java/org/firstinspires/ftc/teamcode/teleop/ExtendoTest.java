package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.MainConfig;
import org.firstinspires.ftc.teamcode.robotcorelib.opmode.OpModePipeline;
import org.firstinspires.ftc.teamcode.robotcorelib.robot.Robot;
import org.firstinspires.ftc.teamcode.robotcorelib.util.RobotRunMode;



@TeleOp(name = "ExtendoTest")
public class ExtendoTest extends OpModePipeline{
    MainConfig subsystems = new MainConfig(); // Initialize MainConfig
    private boolean prevClickA;
    private boolean prevClickB;


    @Override
    public void init() {   //this stuff is needed for all teleop functions so dont question it
        super.subsystems = subsystems;
        runMode = RobotRunMode.TELEOP;
        telemetry.update();
        super.init();
    }

    private boolean isMovingToFront = false; // Tracks the desired position

    @Override
    public void loop() {
        Robot.update();

        // Check if manual input is active
        double forward = gamepad1.right_trigger;
        double back = gamepad1.left_trigger;


        if (Math.abs(forward-back) > 0.1) { // Manual input takes priority
            subsystems.horizontalExtendo.run(back,forward);
            isMovingToFront = false; // Disable automated movement when manual input is used

        }
        // Display telemetry
        telemetry.addData("Left Servo Position", subsystems.horizontalExtendo.getCurrentLinkagePos());
        telemetry.update();
    }


}
