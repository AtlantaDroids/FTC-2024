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
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.IntakeClaw;
import org.firstinspires.ftc.teamcode.subsystems.IntakeExt;

@Autonomous
public class SixSpecimenAuto extends CommandOpMode {
    public Drivetrain drivetrain;
    public IntakeExt intakeExt;
    public IntakeClaw intakeClaw;

    public static PathBuilder builder = new PathBuilder();

    public static PathChain line1 = builder
            .addPath(
                    new BezierLine(
                            new Point(8.625, 72.000, Point.CARTESIAN),
                            new Point(38.500, 72.000, Point.CARTESIAN)
                    )
            )
            .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
            .build();

    public static PathBuilder builder2 = new PathBuilder();
    public static PathChain line2 = builder2
            .addPath(
                    new BezierCurve(
                            new Point(38.500, 72.000, Point.CARTESIAN),
                            new Point(25, 61.440, Point.CARTESIAN),
                            new Point(41, 44, Point.CARTESIAN)
                    )
            )
            .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(290))
            .build();


    public static PathBuilder builder3 = new PathBuilder();
    public static PathChain line3 = builder3
            .addPath(
                    new BezierLine(
                            new Point(35.200, 45.653, Point.CARTESIAN),
                            new Point(25.600, 36.053, Point.CARTESIAN)
                    )
            )
            .setLinearHeadingInterpolation(Math.toRadians(290), Math.toRadians(250))
            .build();
    public static PathBuilder builder4 = new PathBuilder();
    public static PathChain line4 = builder4
            .addPath(
                    new BezierLine(
                            new Point(25.600, 36.053, Point.CARTESIAN),
                            new Point(26.5, 32.5, Point.CARTESIAN)
                    )
            )
            .setLinearHeadingInterpolation(Math.toRadians(250), Math.toRadians(315))
            .build();



    public static PathBuilder builder5 = new PathBuilder();
    public static PathChain line5 = builder5
            .addPath(
                    new BezierLine(
                            new Point(26.5, 33.5, Point.CARTESIAN),
                            new Point(45.440, 22.827, Point.CARTESIAN)
                    )
            )
            .setLinearHeadingInterpolation(Math.toRadians(220), Math.toRadians(270))
            .build();
    public static PathBuilder builder6 = new PathBuilder();
    public static PathChain line6 = builder6
            .addPath(
                    new BezierLine(
                            new Point(45.440, 22.827, Point.CARTESIAN),
                            new Point(16.853, 23.040, Point.CARTESIAN)
                    )
            )
            .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(180))
            .build();

    @Override
    public void initialize() {
        drivetrain = new Drivetrain(this.hardwareMap, new Pose(8.625, 72.000, Math.toRadians(0)), telemetry);
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
                new ParallelCommandGroup(
                        intakeExt.extendIntakeCmd(),
                        intakeClaw.rotateTo90(),
                        intakeClaw.pivotClawCmd(IntakeClaw.IntakePosition.READY)
                        ),
                new WaitCommand(500),
                intakeClaw.pivotClawCmd(IntakeClaw.IntakePosition.COLLECT),
                intakeClaw.closeClawCmdBlocking(),
                new FollowPathChain(drivetrain, line3),
                new WaitCommand(500),
                intakeExt.extendIntakeCmd(),
                new ParallelCommandGroup(
                        intakeClaw.openClawCmd(),
                        intakeClaw.pivotClawCmdBlocking(IntakeClaw.IntakePosition.READY)
                        ),
                new ParallelCommandGroup(
                        new FollowPathChain(drivetrain, line4),
                        intakeClaw.rotateTo0()
                ),
                new FollowPathChain(drivetrain, line5)
//                new FollowPathChain(drivetrain, line6)
        ));
    }
}
