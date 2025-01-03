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
        double manualInput = gamepad1.right_stick_y;

        if (Math.abs(manualInput) > 0.1) { // Manual input takes priority
            subsystems.horizontalExtendo.run(manualInput);
            isMovingToFront = false; // Disable automated movement when manual input is used
        } else {
            // Automated control based on toggle state
            if (isMovingToFront) {
                subsystems.horizontalExtendo.goToFront();
            } else {
                subsystems.horizontalExtendo.goToBack();
            }
        }

        // Check button A for "go to front"
        if (gamepad1.a && !prevClickA) {
            isMovingToFront = true;
        }

        // Check button B for "go to back"
        if (gamepad1.b && !prevClickB) {
            isMovingToFront = false;
        }

        // Update previous click states
        prevClickA = gamepad1.a;
        prevClickB = gamepad1.b;

        // Display telemetry
        telemetry.addData("Left Servo Position", subsystems.horizontalExtendo.getCurrentLinkagePos());
        telemetry.update();
    }


}
