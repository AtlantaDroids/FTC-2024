package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.CreepIntake;
import org.firstinspires.ftc.teamcode.commands.ElevatorGoTo;
import org.firstinspires.ftc.teamcode.commands.ExtendIntake;
import org.firstinspires.ftc.teamcode.commands.ExtendIntakeVariable;
import org.firstinspires.ftc.teamcode.commands.PivotIntake;
import org.firstinspires.ftc.teamcode.commands.RetractIntake;
import org.firstinspires.ftc.teamcode.commands.SetArmPosition;
import org.firstinspires.ftc.teamcode.commands.SetRollerState;
import org.firstinspires.ftc.teamcode.commands.TrajectoryCommand;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.IntakeClaw;
import org.firstinspires.ftc.teamcode.subsystems.IntakeExt;
import org.firstinspires.ftc.teamcode.subsystems.IntakeRoller;

import java.security.spec.PSSParameterSpec;

@Autonomous(name = "Four Sample Auto")
public class FourSampleAuto extends CommandOpMode {

    private Drivetrain drivetrain;
    private Claw claw;
    private Elevator elevator;
    private IntakeClaw intakeClaw;
    private Arm arm;
    private IntakeExt intakeExt;

    @Override
    public void initialize() {

        drivetrain = new Drivetrain(hardwareMap, new Pose2d(-38, -61, Math.toRadians(90)), telemetry);
        claw = new Claw(hardwareMap);
        elevator = new Elevator(hardwareMap, telemetry);
        arm = new Arm(hardwareMap);
        intakeClaw = new IntakeClaw(hardwareMap);
        intakeExt = new IntakeExt(hardwareMap);

        Vector2d homePosition = new Vector2d(-59, -56);

        Action depositLoadSample = drivetrain.getTrajectoryBuilder(new Pose2d(-38, -61, Math.toRadians(90)))
            .strafeToLinearHeading(homePosition, Math.toRadians(45))
            .build();
        Action pickUpFirstSample = drivetrain.getTrajectoryBuilder(new Pose2d(-59, -56, Math.toRadians(45)))
                .strafeToLinearHeading(new Vector2d(-51, -49), Math.toRadians(90))
                .build();
        Action depositFirstSample = drivetrain.getTrajectoryBuilder(new Pose2d(-52, -49, Math.toRadians(90)))
            .strafeToLinearHeading(homePosition, Math.toRadians(45))
            .build();


        Action depositSecondSample = drivetrain.getTrajectoryBuilder(new Pose2d(-40, -35, Math.toRadians(150)))
            .strafeToLinearHeading(homePosition, Math.toRadians(45))
            .build();

        Action depositThirdSample = drivetrain.getTrajectoryBuilder(new Pose2d(-48, -25, Math.toRadians(180)))
            .strafeToLinearHeading(homePosition, Math.toRadians(45))
            .build();




        schedule(new RunCommand(() -> telemetry.update()));
        register(drivetrain, claw, elevator, intakeClaw);
        waitForStart();

        schedule(new SequentialCommandGroup(
                new ParallelCommandGroup(
                    new TrajectoryCommand(depositLoadSample, drivetrain),
                    new ElevatorGoTo(elevator, 1850)
                ),
                new SetArmPosition(arm, Arm.ArmState.SCORE).withTimeout(1000),
                claw.openClawCommand(),
                new WaitCommand(500),
                new SetArmPosition(arm, Arm.ArmState.INTAKE).withTimeout(1000),
                new ParallelCommandGroup(
                        new ElevatorGoTo(elevator, 300),
                        new TrajectoryCommand(pickUpFirstSample, drivetrain),
                        intakeExt.extendIntakeCmd(),
                        intakeClaw.pivotClawCmd(IntakeClaw.IntakePosition.READY)
                ),
                intakeClaw.pivotClawCmd(IntakeClaw.IntakePosition.COLLECT),
                new WaitCommand(400),
                intakeClaw.closeClawCmdBlocking(),
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                intakeClaw.pivotClawCmd(IntakeClaw.IntakePosition.STORE),
                                intakeExt.retractIntakeCmd(),
                                new ElevatorGoTo(elevator, 0),
                                claw.closeClawCommand(),
                                intakeClaw.openClawCmdBlocking(),
                                new ElevatorGoTo(elevator, 1850)
                        ),
                        new TrajectoryCommand(depositFirstSample,drivetrain)
                ),
            new SetArmPosition(arm, Arm.ArmState.SCORE).withTimeout(1000),
            claw.openClawCommand(),
            new WaitCommand(500),
            new SetArmPosition(arm, Arm.ArmState.INTAKE).withTimeout(1000),
            new ElevatorGoTo(elevator, 300)
        ));
    }
}
