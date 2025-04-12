package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.FollowPathChain;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.IntakeClaw;
import org.firstinspires.ftc.teamcode.subsystems.IntakeExt;

@Autonomous
public class FiveAndOne extends CommandOpMode {
    public Drivetrain drivetrain;
    public IntakeExt intakeExt;
    public IntakeClaw intakeClaw;
    public Arm arm;
    public Claw claw;

    public static PathBuilder builder = new PathBuilder();

    public static PathChain line1 = builder
            .addPath(
                    new BezierLine(
                            new Point(10, 72.32, Point.CARTESIAN),
                            new Point(38.5, 72.32, Point.CARTESIAN)
                    )
            )
            .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
            .build();

    public static PathBuilder builder2 = new PathBuilder();
    public static PathChain line2 = builder2
            .addPath(
                    new BezierCurve(
                            new Point(38.5, 72.32, Point.CARTESIAN),
                            new Point(18.347, 71.467, Point.CARTESIAN),
                            new Point(8.533, 12.587, Point.CARTESIAN),
                            new Point(76.800, 30.293, Point.CARTESIAN),
                            new Point(58.027, 45.440, Point.CARTESIAN),
                            new Point(60.587, 28, Point.CARTESIAN)
                    )
            )
            .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
            //line 3

            .addPath(
                    new BezierLine(
                            new Point(60.587, 28, Point.CARTESIAN),
                            new Point(25, 28, Point.CARTESIAN)
                    )
            )
            //line 4
            .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
            .addPath(
                    new BezierCurve(
                            new Point(25, 28, Point.CARTESIAN),
                            new Point(74.667, 34.987, Point.CARTESIAN),
                            new Point(59, 19, Point.CARTESIAN)
                    )
            )
            .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

            //line 5
            .addPath(
                    new BezierLine(
                            new Point(59, 19, Point.CARTESIAN),
                            new Point(25, 17, Point.CARTESIAN)
                    )
            )
            .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
            //line 6
            .addPath(
                    new BezierCurve(
                            new Point(25, 13.867, Point.CARTESIAN),
                            new Point(65.067, 28.373, Point.CARTESIAN),
                            new Point(56.747, 12, Point.CARTESIAN)
                    )
            )
            .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

            .addPath(new BezierLine(
                    new Point(56.987, 12, Point.CARTESIAN),
                    new Point(15, 12, Point.CARTESIAN)
            ))
            .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
            .build();



//
//    public static PathBuilder builder3 = new PathBuilder();
//    public static PathChain line3 = builder3
//            .addPath(
//                    new BezierLine(
//                            new Point(60.587, 28, Point.CARTESIAN),
//                            new Point(13.653, 28, Point.CARTESIAN)
//                    )
//            )
//            .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
//            .build();
//    public static PathBuilder builder4 = new PathBuilder();
//    public static PathChain line4 = builder4
//            .addPath(
//                    new BezierCurve(
//                            new Point(13.653, 28, Point.CARTESIAN),
//                            new Point(74.667, 34.987, Point.CARTESIAN),
//                            new Point(59, 19, Point.CARTESIAN)
//                    )
//            )
//            .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
//            .build();



//    public static PathBuilder builder5 = new PathBuilder();
//    public static PathChain line5 = builder5
//            .addPath(
//                    new BezierLine(
//                            new Point(59, 19, Point.CARTESIAN),
//                            new Point(13.013, 17, Point.CARTESIAN)
//                    )
//            )
//            .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
//            .build();
//    public static PathBuilder builder6 = new PathBuilder();
//    public static PathChain line6 = builder6
//            .addPath(
//                    new BezierCurve(
//                            new Point(13.013, 13.867, Point.CARTESIAN),
//                            new Point(65.067, 28.373, Point.CARTESIAN),
//                            new Point(56.747, 10, Point.CARTESIAN)
//                    )
//            )
//            .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
//            .build();

    public static PathBuilder builder7 = new PathBuilder();
    public static PathChain line7 = builder7
            .addPath(new BezierLine(
                    new Point(56.987, 10, Point.CARTESIAN),
                    new Point(16.640, 10, Point.CARTESIAN)
            ))
            .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
            .build();
    public static PathBuilder builder8 = new PathBuilder();
    public static PathChain line8 = builder8
            .addPath(
                    new BezierCurve(
                            new Point(16.640, 8.107, Point.CARTESIAN),
                            new Point(33.067, 31.360, Point.CARTESIAN),
                            new Point(10, 26.240, Point.CARTESIAN)
                    )
            )
            .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
            .build();
    public static  PathBuilder builder9 = new PathBuilder();
    public static PathChain line9 = builder9
            .addPath(
                    new BezierLine(
                            new Point(10.5, 26.240, Point.CARTESIAN),
                            new Point(40.533, 74.000, Point.CARTESIAN)
                    )
            )
            .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
            .build();
    public static  PathBuilder builder10 = new PathBuilder();

    public static PathChain line10 = builder10
            .addPath(
                    new BezierLine(
                            new Point(40.533, 74.000, Point.CARTESIAN),
                            new Point(10.5, 26.240, Point.CARTESIAN)
                    )
            )
            .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
            .build();

    public static  PathBuilder builder11 = new PathBuilder();
    public static PathChain line11 = builder11
            .addPath(
                    new BezierLine(
                            new Point(10.5, 26.240, Point.CARTESIAN),
                            new Point(40.533, 74.000, Point.CARTESIAN)
                    )
            )
            .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
            .build();
    public static  PathBuilder builder12 = new PathBuilder();
    public static PathChain line12 = builder12
            .addPath(
                    new BezierLine(
                            new Point(40.533, 74.000, Point.CARTESIAN),
                            new Point(10.5, 26.240, Point.CARTESIAN)
                    )
            )
            .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
            .build();
    public static  PathBuilder builder13 = new PathBuilder();
    public static PathChain line13 = builder13
            .addPath(
                    new BezierLine(
                            new Point(10.5, 26.240, Point.CARTESIAN),
                            new Point(40.533, 74.000, Point.CARTESIAN)
                    )
            )
            .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
            .build();
    public static  PathBuilder builder14 = new PathBuilder();
    public static PathChain line14 = builder14
            .addPath(
                    new BezierLine(
                            new Point(40.533, 74.000, Point.CARTESIAN),
                            new Point(10.5, 26.240, Point.CARTESIAN)
                    )
            )
            .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
            .build();
    public static  PathBuilder builder15 = new PathBuilder();
    public static PathChain line15 = builder15
            .addPath(
                    new BezierLine(
                            new Point(10.5, 26.240, Point.CARTESIAN),
                            new Point(40.533, 74.000, Point.CARTESIAN)
                    )
            )
            .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
            .build();
    public static  PathBuilder builder16 = new PathBuilder();
    public static PathChain line16 = builder16
            .addPath(
                    new BezierLine(
                            new Point(40.533, 74.000, Point.CARTESIAN),
                            new Point(10.5, 26.240, Point.CARTESIAN)
                    )
            )
            .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
            .build();




    @Override
    public void initialize() {
        drivetrain = new Drivetrain(this.hardwareMap, new Pose(10, 72.32, Math.toRadians(180)), telemetry);
        intakeExt = new IntakeExt(this.hardwareMap);
        intakeClaw = new IntakeClaw(this.hardwareMap);

        register(drivetrain, intakeClaw, intakeExt);
        waitForStart();
        schedule(new RunCommand(telemetry::update));

        schedule(new SequentialCommandGroup(
                new InstantCommand(drivetrain::resetFollower),
                intakeExt.retractIntakeCmd(),
                intakeClaw.openClawCmd(),
                new FollowPathChain(drivetrain, line1),
                new FollowPathChain(drivetrain, line2, true),
                new FollowPathChain(drivetrain, line8),
                new FollowPathChain(drivetrain, line9),
                new FollowPathChain(drivetrain, line10),
                new FollowPathChain(drivetrain, line11),
                new FollowPathChain(drivetrain, line12),
                new FollowPathChain(drivetrain, line13),
                new FollowPathChain(drivetrain, line14),
                new FollowPathChain(drivetrain, line15),
                new FollowPathChain(drivetrain, line16)










        ));
    }
}
