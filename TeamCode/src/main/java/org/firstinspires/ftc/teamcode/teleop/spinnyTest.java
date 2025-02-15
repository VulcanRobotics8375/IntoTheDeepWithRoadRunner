package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp
public class spinnyTest extends LinearOpMode {
    public CRServo spinner1 = null;
    public CRServo spinner2 = null;

    @Override
    public void runOpMode() throws InterruptedException {
        spinner1 = hardwareMap.get(CRServo.class, "leftSpinner");
        spinner2 = hardwareMap.get(CRServo.class, "rightSpinner");

        waitForStart();

        while (!isStopRequested()) {
            spinner1.setPower(1.0);
            spinner2.setPower(-1.0);

        }
    }
}
