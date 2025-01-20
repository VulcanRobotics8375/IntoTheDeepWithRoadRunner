package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.teamcode.robotcorelib.util.Subsystem;

public class Intake extends Subsystem {

    private Servo claw;
    private CRServo leftSpinner;
    private CRServo rightSpinner;

    private boolean clawOpen = false;

    private static final double CLAW_OPEN_POSITION = 0;
    private static final double CLAW_CLOSED_POSITION = 1;


    @Override
    public void init() {
        claw = hardwareMap.get(Servo.class, "clawServo");
        leftSpinner = hardwareMap.get(CRServo.class, "leftSpinner");
        rightSpinner = hardwareMap.get(CRServo.class, "rightSpinner");
        leftSpinner.setPower(0);
        rightSpinner.setPower(0);
        claw.setPosition(CLAW_CLOSED_POSITION);

    }

    public void sampleIntake(boolean rightBumper) {


       rightSpinner.setPower(1);
       leftSpinner.setPower(-1);

        // Claw Open/Close Logic
        if(rightBumper) {
            toggleClaw();
            rightSpinner.setPower(0);
            leftSpinner.setPower(0);
        }

    }


    public void sampleIntakeReady() {

        leftSpinner.setPower(0);
        rightSpinner.setPower(0);
        claw.setPosition(CLAW_OPEN_POSITION);
        clawOpen = true;

    }


public void specDepo(boolean claw){
    leftSpinner.setPower(0);
    rightSpinner.setPower(0);
    if(claw) {
        toggleClaw();
    }
}


    public void specimenIntake(boolean claw){

        leftSpinner.setPower(0);
        rightSpinner.setPower(0);

        if(claw) {
            toggleClaw();
        }
    }

    public void transferPos() {
        claw.setPosition(CLAW_CLOSED_POSITION);
        leftSpinner.setPower(0);
        rightSpinner.setPower(0);
    }

    public void toggleClaw() {
        clawOpen = !clawOpen; // Toggle the state
        double targetPosition = clawOpen ? CLAW_OPEN_POSITION : CLAW_CLOSED_POSITION;
        claw.setPosition(targetPosition);
    }


    public void sampleDeposit(boolean claw) {
        leftSpinner.setPower(0);
        rightSpinner.setPower(0);
        if(claw) {
            leftSpinner.setPower(-1);
            rightSpinner.setPower(1);
            toggleClaw();
        }
    }

}
