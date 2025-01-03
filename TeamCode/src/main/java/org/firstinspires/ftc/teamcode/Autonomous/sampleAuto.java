package org.firstinspires.ftc.teamcode.Autonomous;

import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.ParallelAction;


// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.HorizontalExtendo;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift2;


@Config
@Autonomous(name = "SAMPLEAUTO", group = "Autonomous")
public class sampleAuto extends  LinearOpMode{

    public Intake intake;
    public Arm arm;
    public HorizontalExtendo horizontalExtendo;
    public Lift2 lift2;


    @Override
    public void runOpMode() {

            intake = new Intake();
            intake.init();
            lift2 = new Lift2();
            lift2.init();
            arm = new Arm();
            arm.init();
            horizontalExtendo = new HorizontalExtendo();
            horizontalExtendo.init();

        Pose2d initialPose = new Pose2d(11.8, 61.7, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        TrajectoryActionBuilder sampleDepoPos = drive.actionBuilder(initialPose)
                .lineToYSplineHeading(33, Math.toRadians(0))
                .waitSeconds(2);


        TrajectoryActionBuilder IntakeSample2 = drive.actionBuilder(initialPose)
                .lineToYSplineHeading(33, Math.toRadians(0))
                .waitSeconds(2);

        Action trajectoryActionCloseOut = sampleDepoPos.endTrajectory().fresh()
                .strafeTo(new Vector2d(48, 12))
                .build();

        telemetry.addLine("Starting Position");
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return; //code to stop program when stop is pressed


        Actions.runBlocking(
                new SequentialAction(
                    new ParallelAction(
                            lift2.sampleDepoAuto(),
                            horizontalExtendo.sampleDepoAuto(),
                            arm.sampleDepoAuto(),
                            intake.sampleDepoAuto(),
                        new SequentialAction(
                            sampleDepoPos.build(),
                            trajectoryActionCloseOut
                        )
                    ),
                    intake.autoToggleClaw(),
                    new ParallelAction(
                            IntakeSample2.build(),
                            lift2.sampleIntakeAuto()
                            trajectoryActionCloseOut


                    )

                )
        );
    }
}
