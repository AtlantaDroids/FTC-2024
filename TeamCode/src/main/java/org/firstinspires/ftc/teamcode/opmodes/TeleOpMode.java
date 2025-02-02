package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.PerpetualCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.CloseClaw;
import org.firstinspires.ftc.teamcode.commands.DefaultDrive;
import org.firstinspires.ftc.teamcode.commands.ElevatorGoTo;
import org.firstinspires.ftc.teamcode.commands.ExtendIntake;
import org.firstinspires.ftc.teamcode.commands.ExtendIntakeVariable;
import org.firstinspires.ftc.teamcode.commands.ManualElevatorCommand;
import org.firstinspires.ftc.teamcode.commands.OpenClaw;
import org.firstinspires.ftc.teamcode.commands.PivotIntake;
import org.firstinspires.ftc.teamcode.commands.RetractIntake;
import org.firstinspires.ftc.teamcode.commands.ScoreAtBucket;
import org.firstinspires.ftc.teamcode.commands.SetKickerPosition;
import org.firstinspires.ftc.teamcode.commands.SetRollerState;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.IntakeClaw;
import org.firstinspires.ftc.teamcode.subsystems.IntakeExt;
import org.firstinspires.ftc.teamcode.subsystems.IntakeRoller;
@TeleOp
public class TeleOpMode extends CommandOpMode {

    private GamepadEx driver;
    private GamepadEx operator;
    private Arm arm;
    private Elevator elevator;
    private Drivetrain drivetrain;
    private Claw claw;
    private IntakeClaw intakeClaw;
    private IntakeExt intakeExt;


    @Override
    public void initialize() {
        driver   = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);

        arm = new Arm(this.hardwareMap);
        elevator = new Elevator(this.hardwareMap, telemetry);
        drivetrain = new Drivetrain(this.hardwareMap, new Pose2d(-58.923881554, -55.0502525317, Math.toRadians(180)), telemetry);
        claw = new Claw(hardwareMap);
        intakeClaw = new IntakeClaw(hardwareMap);
        intakeExt = new IntakeExt(hardwareMap);

        GamepadButton armButton = new GamepadButton(
            operator, GamepadKeys.Button.A
        );
        GamepadButton clawButton = new GamepadButton(
            operator, GamepadKeys.Button.X
        );

        GamepadButton intakeButton = new GamepadButton(
            driver, GamepadKeys.Button.RIGHT_BUMPER
        );

        GamepadButton rotateClawButton = new GamepadButton(
            driver, GamepadKeys.Button.LEFT_BUMPER
        );
        GamepadButton transferClawButton = new GamepadButton(
            driver, GamepadKeys.Button.X
        );

//        GamepadButton openIntakeClaw = new GamepadButton(
//                driver, GamepadKeys.Button.RIGHT_BUMPER
//        );
//        GamepadButton intakeExt = new GamepadButton(
//                driver, GamepadKeys.Button.RIGHT_BUMPER
//        );


        GamepadButton elevatorUpButton = new GamepadButton(
            operator, GamepadKeys.Button.LEFT_BUMPER
        );

        GamepadButton elevatorDownButton = new GamepadButton(
            operator, GamepadKeys.Button.RIGHT_BUMPER
        );

//        GamepadButton kickerButton = new GamepadButton(
//                driver, GamepadKeys.Button.A
//        );
        // You can compose triggers to bind multiple buttons to one action
        // if the trigger is held, move the intake based on the trigger


//        kickerButton.whenPressed(new SetKickerPosition(false, intake))
//                .whenReleased(new SetKickerPosition(true, intake));



        intakeButton.whenPressed(
            new ParallelCommandGroup(
                intakeExt.extendIntakeCmd(),
                intakeClaw.pivotClawCmd(IntakeClaw.IntakePosition.COLLECT)
            ).andThen(
                intakeClaw.openClawCmd()
            )
        ).whenReleased(
            intakeClaw.waitFor(500, intakeClaw.closeClawCmd()).andThen(
                intakeClaw.rotateTo0(),
                intakeClaw.pivotClawCmd(IntakeClaw.IntakePosition.HOME).alongWith(intakeExt.retractIntakeCmd())

            )
        );

        rotateClawButton.whenPressed(intakeClaw.rotateTo90()).whenReleased(intakeClaw.rotateTo0());

        transferClawButton.whenPressed(intakeClaw.openClawCmd()).whenReleased(intakeClaw.closeClawCmd());

        armButton.whenHeld(new InstantCommand(() -> arm.goToPos(Arm.ArmState.SCORE)))
            .whenReleased(new InstantCommand(() -> arm.goToPos(Arm.ArmState.INTAKE)));

        clawButton.whenPressed(new OpenClaw(claw)).whenReleased(new CloseClaw(claw));

        elevatorUpButton.whenPressed(new ElevatorGoTo(elevator, 35));

        elevatorDownButton.whenPressed(new ElevatorGoTo(elevator, 0));

        CommandScheduler.getInstance().setDefaultCommand(elevator, new ManualElevatorCommand(elevator,
            () -> (operator.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) - operator.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)), telemetry));

        drivetrain.setDefaultCommand(new DefaultDrive(drivetrain,
            () -> driver.getLeftX(),
            () -> driver.getLeftY(),
            () -> driver.getRightX()));

        register(arm, intakeClaw, intakeExt);
        schedule(new RunCommand(telemetry::update));

        waitForStart();
        schedule(new InstantCommand(() -> {
            intakeClaw.closeIntakeClaw();
            intakeExt.extendTo(0);
            intakeClaw.rotateClawTo(0);
        }));
    }
}
