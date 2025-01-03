package org.firstinspires.ftc.teamcode.subsystems;

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

    private static final double PITCH_STRAIGHT = 0;
    private static final double PITCH_SPECIMAN = 0.5;
    private static final double PITCH_SAMPLE = 0.687;


    private boolean clawPrevPos = false; //true open, false close
    ;

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
        if(!clawPrevPos){
            toggleClaw(true);
            clawPrevPos = true;
        }


        // Roll Adjustment Logic
        double currentRollPosition = clawRoll.getPosition(); // Get the current roll position
        double rollAdjustment = -rightTrigger * 0.01 + leftTrigger * 0.01;
        clawRoll.setPosition(clamp(currentRollPosition + rollAdjustment, CLAW_NORMAL_POS, ROLL_SPEC_DEPO)); // Adjust claw roll position


        if(leftBumper && currentRollPosition<0.4){
            clawRoll.setPosition(ROLL_90_POSITION);
        }
        else if(leftBumper && currentRollPosition >= 0.4){
            clawRoll.setPosition(CLAW_NORMAL_POS);
        }
        // Claw Open/Close Logic
       toggleClaw(rightBumper);

        // Set Claw Pitch for Sample Intake
        clawPitch.setPosition(PITCH_SAMPLE);
    }


    public void specimenIntake(boolean claw){

        if(!clawPrevPos){
            toggleClaw(true);
            clawPrevPos = true;
        }
        clawRoll.setPosition(CLAW_NORMAL_POS);
        clawPitch.setPosition(PITCH_SPECIMAN);

        toggleClaw(claw);
    }

    public void transferPos(){
        claw.setPosition(CLAW_CLOSED_POSITION);
        clawRoll.setPosition(CLAW_NORMAL_POS);
        clawPitch.setPosition(PITCH_STRAIGHT);
    }

    public void toggleClaw(boolean rightBumperPressed) {
        if (rightBumperPressed) {
            clawOpen = !clawOpen; // Toggle the state
            double targetPosition = clawOpen ? CLAW_OPEN_POSITION : CLAW_CLOSED_POSITION;
            claw.setPosition(targetPosition);
        }
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    public void deposit(boolean claw)
    {

        clawRoll.setPosition(ROLL_SPEC_DEPO);
        clawPitch.setPosition(PITCH_STRAIGHT);
        toggleClaw(claw);

    }

}
