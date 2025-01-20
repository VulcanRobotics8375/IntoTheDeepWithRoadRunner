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



    // Define servo limits
    private final double fullRetractLeft = 0.9208;
    private final double midPosLeft = 0.791;
    private final double fullExtendLeft = 0.33614;

    private final double fullRetractRight = 0.3359;
    private final double midPosRight = 0.46564;
    private final double fullExtendRight = 0.9206;


    // Initialize current positions
    private double leftServoPos = fullRetractLeft;
    private double rightServoPos = fullRetractRight;

    @Override
    public void init() {
        linkServoLeft = hardwareMap.servo.get("linkServoLeft");
        linkServoRight = hardwareMap.servo.get("linkServoRight");

        // Set initial positions
        linkServoLeft.setPosition(leftServoPos);
        linkServoRight.setPosition(rightServoPos);
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }


    // GamePad1 right trigger to extend, left trigger to retract
    // gamePad1 right bumper to full extend, left bumper to fully retract
    public void run(double leftTrigger, double rightTrigger) {


        // Roll Adjustment Logic
        double currentLeftPosition = linkServoLeft.getPosition();
        double currentRightPosition = linkServoRight.getPosition();
        double Adjustment = rightTrigger * 0.01 - leftTrigger * 0.01;
        linkServoLeft.setPosition(clamp(currentLeftPosition - Adjustment, fullExtendLeft, fullRetractLeft));
        linkServoRight.setPosition(clamp(currentRightPosition + Adjustment, fullRetractRight, fullExtendRight));

        moveToPosition(leftServoPos, rightServoPos);
    }


    // Method to set servo positions
    private void moveToPosition(double leftPosition, double rightPosition) {
        linkServoLeft.setPosition(leftPosition);
        linkServoRight.setPosition(rightPosition);
    }



    public void goToFront() {
        moveToPosition(fullExtendLeft, fullExtendRight);
    }

    public void goToBack() {
        moveToPosition(fullRetractLeft, fullRetractRight);
    }

    public  void goToMid(){
        moveToPosition(midPosLeft,midPosRight);
    }



    public String getCurrentLinkagePos(){
        return "left servo pos: " + leftServoPos + "\nright servo pos: " + rightServoPos;
    }
}
