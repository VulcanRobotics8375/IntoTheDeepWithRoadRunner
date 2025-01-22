package org.firstinspires.ftc.teamcode.subsystems;
import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robotcorelib.util.Subsystem;

public class HorizontalExtendo extends Subsystem {

    private Servo linkServoLeft;
    private Servo linkServoRight;

    private final double fullRetractLeft = 0.9208;
    private final double midPosLeft = 0.641;
    private final double fullExtendLeft = 0.33614;

    private final double fullRetractRight = 0.3359;
    private final double midPosRight = 0.61564;
    private final double fullExtendRight = 0.9206;

    private double leftServoPos = fullRetractLeft;
    private double rightServoPos = fullRetractRight;

    @Override
    public void init() {
        linkServoLeft = hardwareMap.servo.get("linkServoLeft");
        linkServoRight = hardwareMap.servo.get("linkServoRight");

        linkServoLeft.setPosition(leftServoPos);
        linkServoRight.setPosition(rightServoPos);
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    public void run(double leftTrigger, double rightTrigger) {
        double adjustment = rightTrigger * 0.005 - leftTrigger * 0.005;
        leftServoPos = clamp(leftServoPos - adjustment, fullExtendLeft, fullRetractLeft);
        rightServoPos = clamp(rightServoPos + adjustment, fullRetractRight, fullExtendRight);

        linkServoLeft.setPosition(leftServoPos);
        linkServoRight.setPosition(rightServoPos);
    }

    public void goToFront() {
        leftServoPos = fullExtendLeft;
        rightServoPos = fullExtendRight;
        moveToPosition(leftServoPos, rightServoPos);
    }

    public void goToBack() {
        leftServoPos = fullRetractLeft;
        rightServoPos = fullRetractRight;
        moveToPosition(leftServoPos, rightServoPos);
    }

    public void goToMid() {
        leftServoPos = midPosLeft;
        rightServoPos = midPosRight;
        moveToPosition(leftServoPos, rightServoPos);
    }

    private void moveToPosition(double leftPosition, double rightPosition) {
        linkServoLeft.setPosition(leftPosition);
        linkServoRight.setPosition(rightPosition);
    }

    public String getCurrentLinkagePos() {
        return "Left servo pos: " + leftServoPos + "\nRight servo pos: " + rightServoPos;
    }
}
