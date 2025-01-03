package org.firstinspires.ftc.teamcode.robot;

import org.firstinspires.ftc.teamcode.robotcorelib.robot.RobotConfig;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.HorizontalExtendo;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift2;
import org.firstinspires.ftc.teamcode.subsystems.LiftHover;
import org.firstinspires.ftc.teamcode.subsystems.OdoLift;

public class AutoConfig extends RobotConfig {

    public Intake intake;
    public Arm arm;
    public HorizontalExtendo horizontalExtendo;
    public Lift2 lift2;

    @Override
    public void init() {
        subsystems.clear();
        intake = new Intake();
        lift2 = new Lift2();
        arm = new Arm();
        horizontalExtendo = new HorizontalExtendo();

        subsystems.add(intake);
        subsystems.add(lift2);
        subsystems.add(arm);
        subsystems.add(horizontalExtendo);
    }
}