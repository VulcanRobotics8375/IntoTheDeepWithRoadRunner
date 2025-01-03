package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robotcorelib.util.Subsystem;

public class Arm extends Subsystem implements Action {

    private Servo armServoLeft;
    private Servo armServoRight;

    private ArmPos armPos = ArmPos.TRANSFER;


    private final double LEFT_FRONT_POSITION = 0.2683;
    private final double RIGHT_FRONT_POSITION = 0.765;

    private final double LEFT_BACK_DEPOSIT_POSITION = 0.7467;
    private final double RIGHT_BACK_DEPOSIT_POSITION = 0.2833;

    private final double LEFT_TRANSFER_POSITION = 0.5339;
    private final double RIGHT_TRANSFER_POSITION = 0.5;


    @Override
    public void init() {
        armServoLeft = hardwareMap.servo.get("armServoLeft");
        armServoRight = hardwareMap.servo.get("armServoRight");
        transfer(); // Start in middle position
    }


    public void front(){
        armPos = ArmPos.FRONT_DEP;
        setPos();
    }

    public void back(){
        armPos = ArmPos.BACK_DEP;
        setPos();
    }

    public void transfer(){
        armPos = ArmPos.TRANSFER;
        setPos();
    }

    
    public double getCurrentLeftPos(){
        return armServoLeft.getPosition();
    }
    public double getCurrentRightPos(){
        return armServoRight.getPosition();
    }

    public void setPos() {
        switch (armPos) {
            case TRANSFER:
                //set servos to 90 degrees up
                armServoLeft.setPosition(LEFT_TRANSFER_POSITION);
                armServoRight.setPosition(RIGHT_TRANSFER_POSITION);
                break;

            case FRONT_DEP:
                //set servos to 0 degrees
                armServoLeft.setPosition(LEFT_FRONT_POSITION);
                armServoRight.setPosition(RIGHT_FRONT_POSITION);
                break;
            case BACK_DEP:
                //set servos to  back
                armServoLeft.setPosition(LEFT_BACK_DEPOSIT_POSITION);
                armServoRight.setPosition(RIGHT_BACK_DEPOSIT_POSITION);
                break;
        }
    }


    // back --> front: right goes up, left goes down
    public void manualArmCode(double manualInput) {
        // Incrementally change


        if (Math.abs(manualInput) > 0.1){
            armServoRight.setPosition(Range.clip(armServoRight.getPosition() + 0.01 * manualInput, RIGHT_BACK_DEPOSIT_POSITION, RIGHT_FRONT_POSITION));
            armServoLeft.setPosition(Range.clip(armServoLeft.getPosition() - 0.01 * manualInput, LEFT_FRONT_POSITION, LEFT_BACK_DEPOSIT_POSITION));


        }
    }


    //for autonomous
    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        return false;
    }

    enum ArmPos {
        SAMPLE,
        FRONT_DEP,
        BACK_DEP,
        TRANSFER
    }

    //for autonomous
    public Action sampleDepoAuto() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (armServoLeft.getPosition() >= 0.7467) {
                    return true; // Action is complete
                } else {
                    back();
                    return false; // Action is still in progress
                }
            }
        };
    }



}
