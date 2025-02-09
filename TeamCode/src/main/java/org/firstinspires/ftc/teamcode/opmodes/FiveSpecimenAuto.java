package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.CloseClaw;
import org.firstinspires.ftc.teamcode.commands.ElevatorGoTo;
import org.firstinspires.ftc.teamcode.commands.OpenClaw;
import org.firstinspires.ftc.teamcode.commands.PivotIntake;
import org.firstinspires.ftc.teamcode.commands.RetractIntake;
import org.firstinspires.ftc.teamcode.commands.TrajectoryCommand;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.IntakeClaw;
import org.firstinspires.ftc.teamcode.subsystems.IntakeExt;

@Autonomous
public class FiveSpecimenAuto extends CommandOpMode {
    private static int HIGH_CHAMBER_HEIGHT = 1300;
    private static int SCORE_HEIGHT = 900;
    private Drivetrain drivetrain;
    private Claw claw;
    private Elevator elevator;
    private IntakeClaw intakeClaw;
    private IntakeExt intakeExt;

    @Override
    public void initialize() {
        drivetrain = new Drivetrain(hardwareMap, new Pose2d(0, -61, Math.toRadians(180)), telemetry);
        claw = new Claw(hardwareMap);
        elevator = new Elevator(hardwareMap, telemetry);
        intakeClaw = new IntakeClaw(hardwareMap);
        intakeExt = new IntakeExt(hardwareMap);
        Vector2d homePosition = new Vector2d(-59, -56);

        Action driveToFirstScore = drivetrain.getTrajectoryBuilder(new Pose2d(0, -61, Math.toRadians(180)))
                .strafeTo(new Vector2d(0, -27))
                .build();

        Action driveToPrimePush = drivetrain.getTrajectoryBuilder(new Pose2d(0, -27, Math.toRadians(180)))
                .splineToConstantHeading(new Vector2d(0,-39), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(26.5,-38.7, Math.toRadians(30)), Math.toRadians(30))
                .build();
        Action pushFirstSample = drivetrain.getTrajectoryBuilder(new Pose2d(26.5, -38.3, Math.toRadians(30)))
                .strafeToLinearHeading(new Vector2d(26,-50), Math.toRadians(330))
                .build();
        Action goToSecondSample = drivetrain.getTrajectoryBuilder(new Pose2d(26,-50, Math.toRadians(330)))
                .strafeToLinearHeading(new Vector2d(37, -38), Math.toRadians(30))
                .build();
        Action pushSecondSample = drivetrain.getTrajectoryBuilder(new Pose2d(37, -38, Math.toRadians(30)))
                .strafeToLinearHeading(new Vector2d(36.5,-50), Math.toRadians(330))
                .build();
        Action goToThirdSample = drivetrain.getTrajectoryBuilder(new Pose2d(36.5,-50, Math.toRadians(330)))
                .strafeToLinearHeading(new Vector2d(44.5, -24.5), Math.toRadians(0))
                .build();
        Action pushThirdSample = drivetrain.getTrajectoryBuilder(new Pose2d(44.5, -24.5, Math.toRadians(0)))
                .strafeToLinearHeading(new Vector2d(44.5,-65), Math.toRadians(0))
                .build();
        Action scoreSecondSample = drivetrain.getTrajectoryBuilder(new Pose2d(44.5, -64, Math.toRadians(0)))
                .strafeToSplineHeading(new Vector2d(3, -29), Math.toRadians(180))
                .build();

        Action pickUpThirdSample = drivetrain.getTrajectoryBuilder(new Pose2d(3, -29, Math.toRadians(180)))
                .strafeToSplineHeading(new Vector2d(44, -64), Math.toRadians(0))
                .build();

        Action scoreThirdSample = drivetrain.getTrajectoryBuilder(new Pose2d(44, -64, Math.toRadians(0)))
                .strafeToSplineHeading(new Vector2d(-2, -29), Math.toRadians(180))
                .build();

        Action pickUpFourthSample = drivetrain.getTrajectoryBuilder(new Pose2d(-2, -29, Math.toRadians(180)))
                .strafeToSplineHeading(new Vector2d(44, -64), Math.toRadians(0))
                .build();
        Action scoreFourthSample = drivetrain.getTrajectoryBuilder(new Pose2d(44, -64, Math.toRadians(0)))
                .strafeToSplineHeading(new Vector2d(-6, -29), Math.toRadians(180))
                .build();
        Action pickUpFifthSample = drivetrain.getTrajectoryBuilder(new Pose2d(-6, -29, Math.toRadians(180)))
                .strafeToSplineHeading(new Vector2d(44, -64), Math.toRadians(0))
                .build();
        Action scoreFifthSample = drivetrain.getTrajectoryBuilder(new Pose2d(44, -64, Math.toRadians(0)))
                .strafeToSplineHeading(new Vector2d(2, -29), Math.toRadians(180))
                .build();


//        Action scoreFirstSample = drivetrain.getTrajectoryBuilder(new Pose2d());
//
//        Action pushSecondSample = drivetrain.getTrajectoryBuilder(new Pose2d());
//
//        Action scoreSecondSample = drivetrain.getTrajectoryBuilder(new Pose2d());
//
//        Action pushThirdSample = drivetrain.getTrajectoryBuilder(new Pose2d());
//
//        Action scoreThirdSample = drivetrain.getTrajectoryBuilder(new Pose2d());
//
//        Action pushFourthSample = drivetrain.getTrajectoryBuilder(new Pose2d());
//
//        Action scoreFourthSample = drivetrain.getTrajectoryBuilder(new Pose2d());




        schedule(new RunCommand(() -> telemetry.update()));
        register(drivetrain, claw, elevator);
        waitForStart();

        schedule(new SequentialCommandGroup(
                new InstantCommand(() -> {
                    intakeClaw.closeIntakeClaw();
                    intakeExt.extendTo(0);
                    intakeClaw.rotateClawTo(0);
                    intakeClaw.pivotTo(IntakeClaw.IntakePosition.STORE);
                }),
                new ParallelCommandGroup(
                        claw.closeClawCommand(),
                        new TrajectoryCommand(driveToFirstScore, drivetrain),
                        new ElevatorGoTo(elevator, HIGH_CHAMBER_HEIGHT)
                ),
                new ElevatorGoTo(elevator, SCORE_HEIGHT),
                claw.openClawCommand(),
                new ParallelCommandGroup(
                        new TrajectoryCommand(driveToPrimePush, drivetrain),
                        new ElevatorGoTo(elevator, 0)
                ),
                new ParallelCommandGroup(
                        intakeExt.extendIntakeCmd(),
                        intakeClaw.waitFor(550, intakeClaw.rotateClawToCmd(0.25).andThen(
                                intakeClaw.pivotClawCmd(IntakeClaw.IntakePosition.READY),
                                intakeClaw.openClawCmd())
                        )
                ),
                intakeClaw.pivotClawCmdBlocking(IntakeClaw.IntakePosition.COLLECT),
                new WaitCommand(100),
                intakeClaw.closeClawCmdBlocking(),
                new TrajectoryCommand(pushFirstSample, drivetrain),
                intakeClaw.openClawCmdBlocking(),
                intakeClaw.pivotClawCmd(IntakeClaw.IntakePosition.READY),
                new TrajectoryCommand(goToSecondSample, drivetrain),
                intakeClaw.pivotClawCmdBlocking(IntakeClaw.IntakePosition.COLLECT),
                intakeClaw.closeClawCmdBlocking(),
                new TrajectoryCommand(pushSecondSample, drivetrain),
                intakeClaw.openClawCmdBlocking(),
                intakeClaw.pivotClawCmdBlocking(IntakeClaw.IntakePosition.READY),
                new ParallelCommandGroup(
                        new TrajectoryCommand(goToThirdSample, drivetrain),
                        intakeClaw.rotateTo90()
                        ),
                intakeClaw.pivotClawCmdBlocking(IntakeClaw.IntakePosition.COLLECT),
                new WaitCommand(100),
                intakeClaw.closeClawCmd(),
                new TrajectoryCommand(pushThirdSample, drivetrain),
                intakeClaw.openClawCmdBlocking(),
                new ParallelCommandGroup(
                        intakeClaw.pivotClawCmd(IntakeClaw.IntakePosition.HOME),
                        intakeExt.retractIntakeCmd(),
                        claw.closeClawCommand()
                ),
                new ParallelCommandGroup(
                        new ElevatorGoTo(elevator, HIGH_CHAMBER_HEIGHT),
                        new TrajectoryCommand(scoreSecondSample, drivetrain)
                ),
                new ElevatorGoTo(elevator, SCORE_HEIGHT),
                claw.openClawCommand(),
                new ParallelCommandGroup(
                       new ElevatorGoTo(elevator, 0),
                       new TrajectoryCommand(pickUpThirdSample, drivetrain)
                ),
                claw.closeClawCommand(),
                new ParallelCommandGroup(
                        new ElevatorGoTo(elevator, HIGH_CHAMBER_HEIGHT),
                        new TrajectoryCommand(scoreThirdSample, drivetrain)
                ),
                new ElevatorGoTo(elevator, SCORE_HEIGHT),
                claw.openClawCommand(),
                new ParallelCommandGroup(
                        new ElevatorGoTo(elevator, 0),
                        new TrajectoryCommand(pickUpFourthSample, drivetrain)
                ),
                claw.closeClawCommand(),
                new ParallelCommandGroup(
                    new ElevatorGoTo(elevator, HIGH_CHAMBER_HEIGHT),
                    new TrajectoryCommand(scoreFourthSample, drivetrain)
                ),
                new ElevatorGoTo(elevator, SCORE_HEIGHT),
                claw.openClawCommand(),
                new ParallelCommandGroup(
                        new TrajectoryCommand(pickUpFifthSample, drivetrain),
                        new ElevatorGoTo(elevator, 0)
                ),
                claw.closeClawCommand(),
                new ParallelCommandGroup(
                    new TrajectoryCommand(scoreFifthSample, drivetrain),
                    new ElevatorGoTo(elevator, HIGH_CHAMBER_HEIGHT)
                ),
                new ElevatorGoTo(elevator, SCORE_HEIGHT),
                claw.openClawCommand(),
                new ElevatorGoTo(elevator, 0)
        ));







    }

}
