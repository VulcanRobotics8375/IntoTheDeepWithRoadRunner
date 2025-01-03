package org.firstinspires.ftc.teamcode.subsystems;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robotcorelib.util.Subsystem;

public class HorizontalExtendo extends Subsystem {

    private Servo linkServoLeft;
    private Servo linkServoRight;

    // Define servo limits
    private final double fullRetractLeft = 0.7;
    private final double fullExtendLeft = 0.044864;

    private final double fullRetractRight = 0.36;
    private final double fullExtendRight = 0.95513608;

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

    // GamePad2 right trigger to extend, left trigger to retract
    // gamePad2 right bumper to full extend, left bumper to fully retract
    public void run(double manualInput) {
        // Ensure manualInput is non-zero
        if (Math.abs(manualInput) > 0.1) { // Add a dead zone to prevent small input noise
            // Adjust servo positions based on input
            leftServoPos = Range.clip(leftServoPos - 0.005 * Math.signum(manualInput), fullExtendLeft, fullRetractLeft);
            rightServoPos = Range.clip(rightServoPos + 0.005 * Math.signum(manualInput), fullRetractRight, fullExtendRight);

            // Apply updated positions
            moveToPosition(leftServoPos, rightServoPos);
        }
    }


    // Method to set servo positions
    private void moveToPosition(double leftPosition, double rightPosition) {
        linkServoLeft.setPosition(leftPosition);
        linkServoRight.setPosition(rightPosition);
    }

    public void moveGradually(double targetLeft, double targetRight) {
        while (Math.abs(leftServoPos - targetLeft) > 0.01 || Math.abs(rightServoPos - targetRight) > 0.01) {
            leftServoPos += (targetLeft - leftServoPos) * 0.1;
            rightServoPos += (targetRight - rightServoPos) * 0.1;
            moveToPosition(leftServoPos, rightServoPos);
        }
    }



    public void goToFront() {
        moveGradually(fullExtendLeft, fullExtendRight);
    }

    public void goToBack() {
        moveGradually(fullRetractLeft, fullRetractRight);
    }


    public String getCurrentLinkagePos(){
        return "left servo pos: " + leftServoPos + "\nright servo pos: " + rightServoPos;
    }
}
