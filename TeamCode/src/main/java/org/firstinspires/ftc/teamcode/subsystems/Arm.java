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

    // can try to be used for spec front depo move

    private final double FRONT_INTAKE_POSITION = 0.3;

    // used for spec front depo and sample front depo

    private final double FRONT_DEPO_POSITION = 0.4328;


    private final double BACK_DEPOSIT_POSITION = 0.7511;


    private final double TRANSFER_POSITION = 0.5861;


    // can try to be used for spec front depo move

    private final double SAMPLEINTAKEHOVER = 0.37;


    @Override
    public void init() {
        armServoLeft = hardwareMap.servo.get("armServoLeft");
        armServoRight = hardwareMap.servo.get("armServoRight");
        transfer(); // Start in middle position
    }

    // sets position for spec depo
    public void specDeo(){
        armPos = ArmPos.SPEC_DEPO;
        setPos();
    }

    // could set position for spec depo move
    public void frontIntake(){
        armPos = ArmPos.FRONT_INTAKE;
        setPos();
    }

    public void frontDeposit() {
        armPos = ArmPos.FRONT_DEPOSIT;
        setPos();
    }

    // could set position for spec depo move
    public void sampleIntakeHover(){
        armPos = ArmPos.SAMPLE_INTAKE_HOVER;
        setPos();
    }

    public void back(){
        armPos = ArmPos.BACK_DEP;
        setPos();
    }

    public void specDepoMove(){
        armPos = ArmPos.SPEC_DEPO_MOVE;
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
                armServoLeft.setPosition(TRANSFER_POSITION);
                armServoRight.setPosition(TRANSFER_POSITION);
                break;
            case FRONT_DEPOSIT:
                armServoLeft.setPosition(FRONT_DEPO_POSITION);
                armServoRight.setPosition(FRONT_DEPO_POSITION);
                break;
            case FRONT_INTAKE:
                armServoLeft.setPosition(FRONT_INTAKE_POSITION);
                armServoRight.setPosition(FRONT_INTAKE_POSITION);
                break;

            case SPEC_DEPO_MOVE:
                armServoLeft.setPosition(SAMPLEINTAKEHOVER+0.75);
                armServoRight.setPosition(SAMPLEINTAKEHOVER+0.75);

            case BACK_DEP:
                //set servos to  back
                armServoLeft.setPosition(BACK_DEPOSIT_POSITION);
                armServoRight.setPosition(BACK_DEPOSIT_POSITION);
                break;
            case SPEC_DEPO:
                //set servos to back
                armServoLeft.setPosition(FRONT_DEPO_POSITION);
                armServoRight.setPosition(FRONT_DEPO_POSITION);
                break;
            case SAMPLE_INTAKE_HOVER:
                //set servos to 0 degrees
                armServoLeft.setPosition(SAMPLEINTAKEHOVER);
                armServoRight.setPosition(SAMPLEINTAKEHOVER);
                break;
        }
    }


    // back --> front: right goes up, left goes down
    public void manualArmCode(double manualInput) {
        // Incrementally change


        if (Math.abs(manualInput) > 0.1){
            armServoRight.setPosition(armServoRight.getPosition() + 0.01 * manualInput);
            armServoLeft.setPosition(armServoLeft.getPosition()+ 0.01 * manualInput);


        }
    }

    enum ArmPos {
        FRONT_INTAKE,
        BACK_DEP,
        TRANSFER,
        FRONT_DEPOSIT,
        SPEC_DEPO,
        SAMPLE_INTAKE_HOVER,
        SPEC_DEPO_MOVE
    }

}
