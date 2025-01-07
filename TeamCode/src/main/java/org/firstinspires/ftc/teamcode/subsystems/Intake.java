package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robotcorelib.util.Subsystem;

public class Intake extends Subsystem {

    private Servo claw;
    private Servo clawRoll;
    private Servo clawPitch;

    private boolean clawOpen = false;

    private static final double CLAW_OPEN_POSITION = 0.5;
    private static final double CLAW_CLOSED_POSITION = 0.32;

    private static final double ROLL_90_POSITION = 0.6173;
    private static final double CLAW_NORMAL_POS = 0.3458;
    private static final double ROLL_SPEC_DEPO = 0.9027;
    private static final double ROLL_MAX_POS = 0;

    private static final double PITCH_STRAIGHT = 0;
    private static final double PITCH_SPECIMAN = 0.5;
    private static final double PITCH_SAMPLE = 0.687;

    @Override
    public void init() {
        claw = hardwareMap.get(Servo.class, "clawServo");
        clawRoll = hardwareMap.get(Servo.class, "clawSpin");
        clawPitch = hardwareMap.get(Servo.class, "clawPitch");

        claw.setPosition(CLAW_CLOSED_POSITION);
        clawRoll.setPosition(CLAW_NORMAL_POS);
        clawPitch.setPosition(PITCH_STRAIGHT);
    }

    public void sampleIntake(double leftTrigger, double rightTrigger, boolean leftBumper, boolean rightBumper) {

        // Roll Adjustment Logic
        double currentRollPosition = clawRoll.getPosition(); // Get the current roll position
        double rollAdjustment = -rightTrigger * 0.01 + leftTrigger * 0.01;
        clawRoll.setPosition(clamp(currentRollPosition + rollAdjustment, ROLL_MAX_POS, ROLL_SPEC_DEPO)); // Adjust claw roll position

        if(leftBumper && currentRollPosition<0.4){
            clawRoll.setPosition(ROLL_90_POSITION);
        }
        else if(leftBumper && currentRollPosition >= 0.4){
            clawRoll.setPosition(CLAW_NORMAL_POS);
        }
        // Claw Open/Close Logic
        if(rightBumper) {
            toggleClaw();
        }
        // Set Claw Pitch for Sample Intake
        clawPitch.setPosition(PITCH_SAMPLE);
    }


    public void specimenIntake(boolean claw){

        clawRoll.setPosition(CLAW_NORMAL_POS);
        clawPitch.setPosition(PITCH_SPECIMAN);

        if(claw) {
            toggleClaw();
        }
    }

    public void transferPos() {
        claw.setPosition(CLAW_CLOSED_POSITION);
        clawRoll.setPosition(CLAW_NORMAL_POS);
        clawPitch.setPosition(PITCH_SAMPLE);
    }

    public void toggleClaw() {
        clawOpen = !clawOpen; // Toggle the state
        double targetPosition = clawOpen ? CLAW_OPEN_POSITION : CLAW_CLOSED_POSITION;
        claw.setPosition(targetPosition);
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    public void depositBack(boolean claw) {
        clawRoll.setPosition(ROLL_SPEC_DEPO);
        clawPitch.setPosition(PITCH_STRAIGHT);
        if(claw) {
            toggleClaw();
        }
    }

    public void depositFront(boolean claw) {
        clawRoll.setPosition(CLAW_NORMAL_POS);
        clawPitch.setPosition(PITCH_STRAIGHT);
        if(claw) {
            toggleClaw();
        }
    }

    public void setPitch(double x){
        clawPitch.setPosition(x);
    }

}
