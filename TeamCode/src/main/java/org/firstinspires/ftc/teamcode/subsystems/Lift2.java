package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robotcorelib.math.control.PID;
import org.firstinspires.ftc.teamcode.robotcorelib.util.Subsystem;

public class Lift2 extends Subsystem{
    private DcMotorEx liftMotorLeft;
    private DcMotorEx liftMotorRight;

    private PID liftPID = new PID(0.013, 0, 0.13,0.005); // Tuned PID coefficients

    private final int LIFT_TOP_POSITION = 2210; // Adjust based on lift's top position
    private int LIFT_BOTTOM_POSITION; // Bottom position (assume 0 for base)

    private double targetPosition = 0.0;
    private boolean liftHolding = false;
    private double lastManualInput = 0.0;

    @Override
    public void init() {
        liftMotorLeft = hardwareMap.get(DcMotorEx.class, "liftMotorLeft"); // Motor with encoder
        liftMotorLeft.setDirection(DcMotor.Direction.REVERSE);

        liftMotorRight = hardwareMap.get(DcMotorEx.class, "liftMotorRight"); // Motor without encoder
        liftMotorRight.setDirection(DcMotor.Direction.REVERSE);

        // Initialize liftMotorLeft
        liftMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotorLeft.setPower(0);

        // Initialize liftMotorRight
        liftMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotorRight.setPower(0);

        LIFT_BOTTOM_POSITION = liftMotorLeft.getCurrentPosition();
        targetPosition = LIFT_BOTTOM_POSITION; // Start at the bottom

        liftPID.setOutputLimits(-1.0, 1.0);
    }

    public void setPos(int height) {
        // Set the target position within bounds
        targetPosition = Range.clip(height, LIFT_BOTTOM_POSITION, LIFT_TOP_POSITION);
        liftPID.reset(); // Reset the PID controller
        liftHolding = true; // Enable automated movement
    }

    public void update() {
        if (liftHolding) {
            // Ensure the target position is within limits
            targetPosition = Range.clip(targetPosition, LIFT_BOTTOM_POSITION, LIFT_TOP_POSITION);

            // Calculate the PID output
            double currentPosition = liftMotorLeft.getCurrentPosition();
            double pidOutput = liftPID.getOutput(currentPosition, targetPosition);

            // Apply the PID output to the motors
            liftMotorLeft.setPower(pidOutput);
            liftMotorRight.setPower(pidOutput); // Sync the right motor

            // Allow the PID to hold the position
            if (Math.abs(currentPosition - targetPosition) <= 10) {
                // Keep liftHolding active for PID-based holding
                // The PID controller adjusts motor power dynamically
                applyPIDControl();

            }
        }
    }


    public void manualControl(double manualInput) {
        if (Math.abs(manualInput) > 0.1) {
            // Manual control
            liftHolding = false;
            targetPosition = liftMotorLeft.getCurrentPosition(); // Update target for hold mode

            double power = Range.clip(manualInput, -1.0, 1.0);
            liftMotorLeft.setPower(power);
            liftMotorRight.setPower(liftMotorLeft.getPower()); // Sync power
            lastManualInput = manualInput;
        } else {
            // Hold position with PID control
            if (!liftHolding) {
                liftHolding = true;
                liftPID.reset();
                targetPosition = liftMotorLeft.getCurrentPosition();
            }

            applyPIDControl();
        }
    }

    private void applyPIDControl() {
        targetPosition = Range.clip(targetPosition, LIFT_BOTTOM_POSITION, LIFT_TOP_POSITION);

        double currentPosition = liftMotorLeft.getCurrentPosition();
        double pidOutput = liftPID.getOutput(currentPosition, targetPosition);

        // Apply power to motors
        liftMotorLeft.setPower(pidOutput);
        liftMotorRight.setPower(liftMotorLeft.getPower()); // Sync right motor
    }

    public void stop() {
        liftMotorLeft.setPower(0);
        liftMotorRight.setPower(0); // Stop the follower motor
        liftHolding = false;
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Lift Current Position", liftMotorLeft.getCurrentPosition());
        telemetry.addData("Lift Target Position", targetPosition);
        telemetry.addData("Lift Motor Power", liftMotorLeft.getPower());
        telemetry.addData("Last Manual Input", lastManualInput);
        telemetry.addData("Lift Stalling?", isStalling());
    }

    private boolean isStalling() {
        // Adjust threshold based on your motor and load
        return Math.abs(liftMotorLeft.getCurrentPosition() - targetPosition) > 50 &&
                liftMotorLeft.getPower() > 0.8;
    }

    private void clampTargetPosition() {
        targetPosition = Range.clip(targetPosition, LIFT_BOTTOM_POSITION, LIFT_TOP_POSITION);
    }


}
