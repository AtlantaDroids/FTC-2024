package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
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

import org.firstinspires.ftc.teamcode.commands.ElevatorGoTo;
import org.firstinspires.ftc.teamcode.commands.FollowPathChain;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.IntakeClaw;
import org.firstinspires.ftc.teamcode.subsystems.IntakeExt;

@Autonomous
public class FiveAndOne extends CommandOpMode {
    public Drivetrain drivetrain;
    public IntakeExt intakeExt;
    public IntakeClaw intakeClaw;
    public Arm arm;
    public Claw claw;
    public Elevator elevator;

    public static PathBuilder builder = new PathBuilder();

    public static PathChain line1 = builder
            .addPath(
                    new BezierLine(
                            new Point(10, 72.32, Point.CARTESIAN),
                            new Point(37, 72.32, Point.CARTESIAN)
                    )
            )
            .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
            .build();


    public static PathBuilder builder2 = new PathBuilder();
    public static PathChain line2 = builder2
            .addPath(
                    new BezierCurve(
                            new Point(37, 72.32, Point.CARTESIAN),
                            new Point(18.347, 71.467, Point.CARTESIAN),
                            new Point(8.533, 12.587, Point.CARTESIAN),
                            new Point(76.800, 30.293, Point.CARTESIAN),
                            new Point(58.027, 45.440, Point.CARTESIAN),
                            new Point(57, 28, Point.CARTESIAN)
                    )
            )
            .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
            //line 3

            .addPath(
                    new BezierLine(
                            new Point(57, 28, Point.CARTESIAN),
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
                            new Point(25, 17, Point.CARTESIAN),
                            new Point(65.067, 28.373, Point.CARTESIAN),
                            new Point(56.747, 12, Point.CARTESIAN)
                    )
            )
            .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

            .addPath(new BezierLine(
                    new Point(56.987, 12, Point.CARTESIAN),
                    new Point(16.640, 8.107, Point.CARTESIAN)
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

    //    public static PathBuilder builder7 = new PathBuilder();
//    public static PathChain line7 = builder7
//            .addPath(new BezierLine(
//                    new Point(56.987, 10, Point.CARTESIAN),
//                    new Point(16.640, 10, Point.CARTESIAN)
//            ))
//            .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
//            .build();
    public static PathBuilder builder8 = new PathBuilder();
    public static PathChain line8 = builder8
            .addPath(
                    new BezierCurve(
                            new Point(16.640, 8.107, Point.CARTESIAN),
                            new Point(33.067, 31.360, Point.CARTESIAN),
                            new Point(15.5, 38, Point.CARTESIAN)
                    )
            )
            .setZeroPowerAccelerationMultiplier(6)
            .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
            .build();
    public static  PathBuilder builder9 = new PathBuilder();
    public static PathChain line9 = builder9
            .addPath(
                    new BezierLine(
                            new Point(15.5, 38, Point.CARTESIAN),
                            new Point(37, 70.000, Point.CARTESIAN)
                    )
            )
            .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
            .build();
    public static  PathBuilder builder10 = new PathBuilder();

    public static PathChain line10 = builder10
            .addPath(
                    new BezierLine(
                            new Point(37, 70.000, Point.CARTESIAN),
                            new Point(12, 41, Point.CARTESIAN)
                    )
            )
            .setZeroPowerAccelerationMultiplier(6)
            .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
            .build();

    public static  PathBuilder builder11 = new PathBuilder();
    public static PathChain line11 = builder11
            .addPath(
                    new BezierLine(
                            new Point(12, 41, Point.CARTESIAN),
                            new Point(37, 68.000, Point.CARTESIAN)
                    )
            )
            .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
            .build();
    public static  PathBuilder builder12 = new PathBuilder();
    public static PathChain line12 = builder12
            .addPath(
                    new BezierLine(
                            new Point(37, 68, Point.CARTESIAN),
                            new Point(12, 41, Point.CARTESIAN)
                    )
            )
            .setZeroPowerAccelerationMultiplier(6)
            .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
            .build();
    public static  PathBuilder builder13 = new PathBuilder();
    public static PathChain line13 = builder13
            .addPath(
                    new BezierLine(
                            new Point(12, 41, Point.CARTESIAN),
                            new Point(39, 66, Point.CARTESIAN)
                    )
            )
            .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
            .build();
    public static  PathBuilder builder14 = new PathBuilder();
    public static PathChain line14 = builder14
            .addPath(
                    new BezierLine(
                            new Point(39, 66, Point.CARTESIAN),
                            new Point(12, 41, Point.CARTESIAN)
                    )
            )
            .setZeroPowerAccelerationMultiplier(6)
            .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
            .build();
    public static  PathBuilder builder15 = new PathBuilder();
    public static PathChain line15 = builder15
            .addPath(
                    new BezierLine(
                            new Point(12, 41, Point.CARTESIAN),
                            new Point(39, 64, Point.CARTESIAN)
                    )
            )
            .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
            .build();
    public static  PathBuilder builder16 = new PathBuilder();
    public static PathChain line16 = builder16
            .addPath(
                    new BezierLine(
                            new Point(39, 64, Point.CARTESIAN),
                            new Point(12.5, 41, Point.CARTESIAN)
                    )
            )
            .setZeroPowerAccelerationMultiplier(6)
            .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
            .build();




    @Override
    public void initialize() {
        drivetrain = new Drivetrain(this.hardwareMap, new Pose(10, 72.32, Math.toRadians(180)), telemetry);
        intakeExt = new IntakeExt(this.hardwareMap);
        intakeClaw = new IntakeClaw(this.hardwareMap);
        elevator = new Elevator(this.hardwareMap, telemetry);
        claw = new Claw(this.hardwareMap);
        arm = new Arm(this.hardwareMap);
        register(drivetrain, intakeClaw, intakeExt, elevator, claw, arm);
        waitForStart();
        schedule(new RunCommand(telemetry::update));

        schedule(new SequentialCommandGroup(
                new InstantCommand(drivetrain::resetFollower),
                new ParallelCommandGroup(
                        claw.closeClawCommand(),
//                        intakeExt.retractIntakeCmd(),
//                        intakeClaw.pivotClawCmdBlocking(IntakeClaw.IntakePosition.STORE),
                        arm.goToPosCmd(Arm.ArmState.SCORE)
//
                ),
                // Score preload
                new ParallelCommandGroup(
                        new FollowPathChain(drivetrain, line1),
                        new ElevatorGoTo(elevator, Elevator.PREPARE),
                        arm.elbowGoToPosCmd(Arm.ArmState.WAIT).andThen(
                                arm.elbowGoToPosCmd(Arm.ArmState.SCORE))
                ),
                new ElevatorGoTo(elevator, Elevator.SCORE+20),
                // Push samples
                new ParallelCommandGroup(
                        claw.openClawCommand().andThen(
                                arm.goToPosCmd(Arm.ArmState.COLLECT),
                                arm.elbowGoToPosCmd(Arm.ArmState.COLLECT)
                        ),
                        new ElevatorGoTo(elevator, Elevator.DOWN),
                        new FollowPathChain(drivetrain, line2, true)
                ),
                new FollowPathChain(drivetrain, line8),
                new ParallelCommandGroup(
                        new FollowPathChain(drivetrain, line8),
                        new ElevatorGoTo(elevator, Elevator.DOWN),
                        arm.goToPosCmd(Arm.ArmState.COLLECT),
                        claw.openClawCommand()
                ),
                // Pick up first
                claw.closeClawCommand(),

                new ParallelCommandGroup(
                        new FollowPathChain(drivetrain, line9),
                        arm.goToPosCmd(Arm.ArmState.SCORE).andThen(

                                arm.elbowGoToPosCmd(Arm.ArmState.SCORE)),
                        new ElevatorGoTo(elevator, Elevator.PREPARE)
                ),
                // Score First
                new ElevatorGoTo(elevator, Elevator.SCORE),
                new ParallelCommandGroup(
                        claw.openClawCommand().andThen(
                                arm.goToPosCmd(Arm.ArmState.COLLECT),
                                arm.elbowGoToPosCmd(Arm.ArmState.COLLECT)
                        ),
                        new FollowPathChain(drivetrain, line10),
                        new ElevatorGoTo(elevator, Elevator.DOWN)
                ),
                // Pick up second
                claw.closeClawCommand(),
                new ParallelCommandGroup(
                        new FollowPathChain(drivetrain, line11),
                        arm.goToPosCmd(Arm.ArmState.SCORE).andThen(
                                arm.elbowGoToPosCmd(Arm.ArmState.SCORE)),
                        new ElevatorGoTo(elevator, Elevator.PREPARE)
                ),
                // Score second
                new ElevatorGoTo(elevator, Elevator.SCORE),
                new ParallelCommandGroup(
                        claw.openClawCommand().andThen(
                                arm.elbowGoToPosCmd(Arm.ArmState.COLLECT),
                                arm.goToPosCmd(Arm.ArmState.COLLECT)
                        ),
                        new FollowPathChain(drivetrain, line12),
                        new ElevatorGoTo(elevator, Elevator.DOWN)
                ),
                // Pick up third
                claw.closeClawCommand(),
                new ParallelCommandGroup(
                        new FollowPathChain(drivetrain, line13),
                        arm.goToPosCmd(Arm.ArmState.SCORE).andThen(
                                arm.elbowGoToPosCmd(Arm.ArmState.SCORE)),
                        new ElevatorGoTo(elevator, Elevator.PREPARE)
                ),
                // Score third
                new ElevatorGoTo(elevator, Elevator.SCORE),
                new ParallelCommandGroup(
                        claw.openClawCommand().andThen(
                                arm.goToPosCmd(Arm.ArmState.COLLECT),
                                arm.elbowGoToPosCmd(Arm.ArmState.COLLECT)
                        ),
                        new FollowPathChain(drivetrain, line14),
                        new ElevatorGoTo(elevator, Elevator.DOWN)
                ),
                // Pick up 4
                claw.closeClawCommand(),
                new ParallelCommandGroup(
                        new FollowPathChain(drivetrain, line15),
                        arm.goToPosCmd(Arm.ArmState.SCORE).andThen(
                                arm.elbowGoToPosCmd(Arm.ArmState.SCORE)),
                        new ElevatorGoTo(elevator, Elevator.PREPARE)
                ),
                // Score 4
                new ElevatorGoTo(elevator, Elevator.SCORE),
                new ParallelCommandGroup(
                        claw.openClawCommand().andThen(
                                arm.elbowGoToPosCmd(Arm.ArmState.COLLECT),
                                arm.goToPosCmd(Arm.ArmState.COLLECT)
                        ),
                        //park
                        new ParallelCommandGroup(
                                new FollowPathChain(drivetrain, line16),
                                new ElevatorGoTo(elevator, Elevator.DOWN)

                        )


                )











        ));
    }
}
