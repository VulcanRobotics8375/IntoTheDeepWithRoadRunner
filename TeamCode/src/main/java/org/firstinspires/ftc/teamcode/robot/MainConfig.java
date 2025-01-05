package org.firstinspires.ftc.teamcode.robot;

import org.firstinspires.ftc.teamcode.robotcorelib.robot.RobotConfig;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.HorizontalExtendo;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift2;

public class MainConfig extends RobotConfig {

    public Drivetrain drivetrain;
    public Intake intake;
    public Arm arm;
    public HorizontalExtendo horizontalExtendo;
    public Lift2 lift2;

    @Override
    public void init() {
        subsystems.clear();

        try {
            drivetrain = new Drivetrain();
            intake = new Intake();
            lift2 = new Lift2();
            arm = new Arm();
            horizontalExtendo = new HorizontalExtendo();
        } catch (Exception e) {
            throw new RuntimeException("Subsystem initialization failed: " + e.getMessage());
        }
    }

    public void sampleIntake(double clawRollIncCCW, double clawRollIncCW, boolean claw90degTurn, boolean claw) {
        lift2.setPos(0);//down
        horizontalExtendo.goToFront();
        arm.front();
        intake.sampleIntake(clawRollIncCCW, clawRollIncCW,  claw90degTurn, claw);
    }

    public void specimenIntake(boolean claw) {
        intake.specimenIntake(claw);
        lift2.setPos(0);//down
        arm.front();
        horizontalExtendo.goToFront();
    }



    public void depositFront(int height, boolean claw) {
        intake.deposit(claw);
        lift2.setPos(height);
        arm.front();
        horizontalExtendo.goToBack();
    }

    public void depositBack(int height, boolean claw) {
        intake.deposit(claw);
        lift2.setPos(height);
        arm.back();
        horizontalExtendo.goToBack();
    }

    public void transferPos() {
        intake.transferPos();
        arm.transfer();
        lift2.setPos(0);//go down
        horizontalExtendo.goToBack();
    }

}

