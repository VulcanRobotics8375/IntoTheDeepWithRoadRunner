package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.MainConfig;
import org.firstinspires.ftc.teamcode.robotcorelib.opmode.OpModePipeline;
import org.firstinspires.ftc.teamcode.robotcorelib.robot.Robot;
import org.firstinspires.ftc.teamcode.robotcorelib.util.RobotRunMode;

@TeleOp(name = "DrivetrainTest")
public class DrivetrainTest extends OpModePipeline {
    MainConfig subsystems = new MainConfig(); // Initialize MainConfig

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

        // Run drivetrain using the controller input
        subsystems.drivetrain.mecanumDrive(
                -gamepad1.left_stick_y,  // Forward/backward
                gamepad1.left_stick_x,   // Strafe
                gamepad1.right_stick_x   // Rotation
        );

        telemetry.addLine("Drivetrain test in progress...");
        telemetry.addData("Left Stick Y", -gamepad1.left_stick_y);
        telemetry.addData("Left Stick X", gamepad1.left_stick_x);
        telemetry.addData("Right Stick X", gamepad1.right_stick_x);
        telemetry.update();
    }
}
