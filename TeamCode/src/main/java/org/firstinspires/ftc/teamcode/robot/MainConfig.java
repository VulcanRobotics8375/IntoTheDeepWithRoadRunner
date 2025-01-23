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

    public void sampleIntake(boolean claw) {
        lift2.setPos(0);//down
        arm.frontIntake();
        intake.sampleIntake(claw);
    }

    public void sampleIntakeReady() {
        lift2.setPos(0);//down
        horizontalExtendo.goToMid();
        arm.sampleIntakeHover();
        intake.sampleIntakeReady();
    }

    public void specimenIntake(boolean claw) {
        intake.specimenIntake(claw);
        lift2.setPos(0);//down
        arm.frontIntake();
    }
    
    public void depositFront(int height, boolean claw) {
        intake.sampleDeposit(claw);
        lift2.setPos(height);
        arm.frontDeposit();
        horizontalExtendo.goToMid();
    }

    public void depositBack(int height, boolean claw) {
        intake.sampleDeposit(claw);
        lift2.setPos(height);
        arm.back();
        horizontalExtendo.goToBack();
    }


    public void specDepositFront(int height, boolean claw) {
        intake.specDepo(claw);
        lift2.setPos(height);
        arm.specDeo();
        horizontalExtendo.goToMid();
    }

    public void specDepositFrontMove(int height, boolean claw) {
        horizontalExtendo.goToFront();
        intake.specDepo(claw);
        lift2.setPos(height);
        arm.specDepoMove();
    }
    
    public void transferPos() {
        intake.transferPos();
        arm.transfer();
        lift2.setPos(0);//go down
        horizontalExtendo.goToBack();
    }

    public void hang(){
        intake.transferPos();
        lift2.setPos(2200);
        arm.transfer();
        horizontalExtendo.goToBack();
    }

}

