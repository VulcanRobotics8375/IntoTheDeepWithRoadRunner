package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robotcorelib.util.Subsystem;

public class Arm extends Subsystem {

    private Servo armServoLeft;
    private Servo armServoRight;

    private ArmPos armPos = ArmPos.TRANSFER;


    private final double LEFT_FRONT_POSITION = 0.2683;
    private final double RIGHT_FRONT_POSITION = 0.765;

    private final double LEFT_FRONT_DEPO_POSITION = 0.39;
    private final double RIGHT_FRONT_DEPO_POSITION = 0.6378;

    private final double LEFT_BACK_DEPOSIT_POSITION = 0.7067;
    private final double RIGHT_BACK_DEPOSIT_POSITION = 0.3233;

    private final double LEFT_TRANSFER_POSITION = 0.5339;
    private final double RIGHT_TRANSFER_POSITION = 0.5;

    private final double SAMPLEINTAKEHOVERLEFT = 0.2889;
    private final double SAMPLEINTAKEHOVERRIGHT = 0.7439;

    private final double LEFTSPECDEPO = 0.8167;
    private final double RIGHTSPECDEPO = 0.2156;

    @Override
    public void init() {
        armServoLeft = hardwareMap.servo.get("armServoLeft");
        armServoRight = hardwareMap.servo.get("armServoRight");
        transfer(); // Start in middle position
    }

    public void specDeo(){
        armPos = ArmPos.SPEC_DEPO;
        setPos();
    }



    public void frontIntake(){
        armPos = ArmPos.FRONT_INTAKE;
        setPos();
    }

    public void frontDeposit() {
        armPos = ArmPos.FRONT_DEPOSIT;
        setPos();
    }

    public void sampleIntakeHover(){
        armPos = ArmPos.SAMPLE_INTAKE_HOVER;
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

            case FRONT_DEPOSIT:
                //set servos to 0 degrees
                armServoLeft.setPosition(LEFT_FRONT_DEPO_POSITION);
                armServoRight.setPosition(RIGHT_FRONT_DEPO_POSITION);
                break;
            case FRONT_INTAKE:
                armServoLeft.setPosition(LEFT_FRONT_POSITION);
                armServoRight.setPosition(RIGHT_FRONT_POSITION);
                break;
            case BACK_DEP:
                //set servos to  back
                armServoLeft.setPosition(LEFT_BACK_DEPOSIT_POSITION);
                armServoRight.setPosition(RIGHT_BACK_DEPOSIT_POSITION);
                break;

            case SPEC_DEPO:
                //set servos to  back
                armServoLeft.setPosition(LEFTSPECDEPO);
                armServoRight.setPosition(RIGHTSPECDEPO);
                break;
            case SAMPLE_INTAKE_HOVER:
                //set servos to 0 degrees
                armServoLeft.setPosition(SAMPLEINTAKEHOVERLEFT);
                armServoRight.setPosition(SAMPLEINTAKEHOVERRIGHT);
                break;
        }
    }


    // back --> front: right goes up, left goes down
    public void manualArmCode(double manualInput) {
        // Incrementally change


        if (Math.abs(manualInput) > 0.1){
            armServoRight.setPosition(armServoRight.getPosition() + 0.01 * manualInput);
            armServoLeft.setPosition(armServoLeft.getPosition()- 0.01 * manualInput);


        }
    }

    enum ArmPos {
        FRONT_INTAKE,
        BACK_DEP,
        TRANSFER,
        FRONT_DEPOSIT,
        SPEC_DEPO,
        SAMPLE_INTAKE_HOVER
    }

}
