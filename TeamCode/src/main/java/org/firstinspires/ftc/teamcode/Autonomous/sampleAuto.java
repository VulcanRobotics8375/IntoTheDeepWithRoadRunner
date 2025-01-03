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
import org.firstinspires.ftc.teamcode.robotcorelib.opmode.AutoPipeline;
import org.firstinspires.ftc.teamcode.robotcorelib.robot.AutoConfig;
import org.firstinspires.ftc.teamcode.robotcorelib.util.RobotRunMode;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.HorizontalExtendo;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift2;


@Config
@Autonomous(name = "SAMPLEAUTO", group = "Autonomous")
public class sampleAuto extends AutoPipeline {



    @Override
    public void runOpMode() throws InterruptedException {
        runMode = RobotRunMode.AUTONOMOUS;
        AutoConfig subsystems = new AutoConfig();

        super.subsystems = subsystems;
        runMode = RobotRunMode.AUTONOMOUS;
        robotInit();
        sleep(500);


        Pose2d initialPose = new Pose2d(-37, -64.5, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        TrajectoryActionBuilder sample1DepoPos = drive.actionBuilder(initialPose)
                .splineToConstantHeading(new Vector2d(-37.0, -64.5),0);
          //      .waitSeconds(2);


        TrajectoryActionBuilder IntakeSample2 = drive.actionBuilder(initialPose)
               // .lineToYSplineHeading(33, Math.toRadians(0))
                .waitSeconds(2);

        TrajectoryActionBuilder sample2DepoPos = drive.actionBuilder(initialPose)
              //  .lineToYSplineHeading(33, Math.toRadians(0))
                .waitSeconds(2);


        TrajectoryActionBuilder IntakeSample3 = drive.actionBuilder(initialPose)
              //  .lineToYSplineHeading(33, Math.toRadians(0))
                .waitSeconds(2);

        TrajectoryActionBuilder IntakeSample4 = drive.actionBuilder(initialPose)
            //    .lineToYSplineHeading(33, Math.toRadians(0))
                .waitSeconds(2);



        telemetry.addLine("Starting Position");
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return; //code to stop program when stop is pressed


        Actions.runBlocking(
                new SequentialAction(

                        //deposit preload sample
                    new ParallelAction(
//                            subsystems.lift2.sampleDepoAuto(),
//                            subsystems.horizontalExtendo.sampleDepoAuto(),
//                            subsystems.arm.sampleDepoAuto(),
//                            subsystems.intake.sampleDepoAuto(),
                            sample1DepoPos.build() //path
                    )//,
                        //subsystems.intake.autoToggleClaw()

//                    //intake second sample
//                    new ParallelAction(
//                            IntakeSample2.build(), //path
//                            subsystems.lift2.sampleIntakeAuto(),
//                            subsystems.horizontalExtendo.sampleIntakeAuto(),
//                            subsystems.arm.sampleIntakeAuto(),
//                            subsystems.intake.sampleIntakeAuto()
//                    ),
//                        intake.autoToggleClaw(),
//
//                        //deposit second sample
//                        new ParallelAction(
//                                subsystems.lift2.sampleDepoAuto(),
//                                subsystems.horizontalExtendo.sampleDepoAuto(),
//                                subsystems.arm.sampleDepoAuto(),
//                                subsystems.intake.sampleDepoAuto(),
//                                sample2DepoPos.build() //path
//                        ),
//                        intake.autoToggleClaw(),
//
//                        //intake third sample
//                        new ParallelAction(
//                                subsystems.IntakeSample3.build(), //path
//                                subsystems.lift2.sampleIntakeAuto(),
//                                subsystems.horizontalExtendo.sampleIntakeAuto(),
//                                subsystems.arm.sampleIntakeAuto(),
//                                subsystems.intake.sampleIntakeAuto()
//                        )
//
//
                        )
        );
    }
}
