package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robot.MainConfig;
import org.firstinspires.ftc.teamcode.robotcorelib.opmode.OpModePipeline;
import org.firstinspires.ftc.teamcode.robotcorelib.robot.Robot;
import org.firstinspires.ftc.teamcode.robotcorelib.util.RobotRunMode;
@TeleOp (name = "CR Servo Test")
public class continuousServoTest extends OpModePipeline {
    private CRServo spinner;

    public void init() {

        spinner = hardwareMap.get(CRServo.class, "leftSpinner");

    }

    @Override
    public void loop() {
            spinner.setPower(1);
        telemetry.update();
    }
}