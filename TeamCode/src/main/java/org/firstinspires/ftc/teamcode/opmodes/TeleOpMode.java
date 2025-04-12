package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.commands.CloseClaw;
import org.firstinspires.ftc.teamcode.commands.DefaultDrive;
import org.firstinspires.ftc.teamcode.commands.ElevatorGoTo;
import org.firstinspires.ftc.teamcode.commands.ExtendIntake;
import org.firstinspires.ftc.teamcode.commands.ManualElevatorCommand;
import org.firstinspires.ftc.teamcode.commands.OpenClaw;
import org.firstinspires.ftc.teamcode.commands.SetArmPosition;
import org.firstinspires.ftc.teamcode.commands.SetClawPosition;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.IntakeClaw;
import org.firstinspires.ftc.teamcode.subsystems.IntakeExt;

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
        drivetrain = new Drivetrain(this.hardwareMap, new Pose(0, -0, Math.toRadians(180)), telemetry);
        claw = new Claw(hardwareMap);
        intakeClaw = new IntakeClaw(hardwareMap);
        intakeExt = new IntakeExt(hardwareMap);



        GamepadButton goToScoreButton = new GamepadButton(
                operator, GamepadKeys.Button.Y
        );
        GamepadButton clawButton = new GamepadButton(
                operator, GamepadKeys.Button.X
        );
        GamepadButton pickUpButton = new GamepadButton(
                operator, GamepadKeys.Button.A
        );

        GamepadButton intakeButton = new GamepadButton(
            driver, GamepadKeys.Button.RIGHT_BUMPER
        );

        GamepadButton rotateClawButton = new GamepadButton(
            driver, GamepadKeys.Button.LEFT_BUMPER
        );
//        GamepadButton transferClawButton = new GamepadButton(
//            driver, GamepadKeys.Button.X
//        ); add back later

//        GamepadButton openIntakeClaw = new GamepadButton(
//                driver, GamepadKeys.Button.RIGHT_BUMPER
//        );
//        GamepadButton intakeExt = new GamepadButton(
//                driver, GamepadKeys.Button.RIGHT_BUMPER
//        );


//        GamepadButton clawFlipButton = new GamepadButton(
//                operator, GamepadKeys.Button.B
//        );
//        GamepadButton collectButton = new GamepadButton(
//                operator, GamepadKeys.Button.Y
//        );
        GamepadButton elevatorUpButton = new GamepadButton(
            operator, GamepadKeys.Button.LEFT_BUMPER
        );

        GamepadButton elevatorDownButton = new GamepadButton(
            operator, GamepadKeys.Button.RIGHT_BUMPER
        );
        GamepadButton zeroButton = new GamepadButton(
            driver, GamepadKeys.Button.Y
        );

//        GamepadButton kickerButton = new GamepadButton(
//                driver, GamepadKeys.Button.A
//        );
        // You can compose triggers to bind multiple buttons to one action
        // if the trigger is held, move the intake based on the trigger


//        kickerButton.whenPressed(new SetKickerPosition(false, intake))
//                .whenReleased(new SetKickerPosition(true, intake));


        zeroButton.whenPressed(drivetrain::reset);

//        collectButton.whenPressed(arm.goToPosCmd(Arm.ArmState.COLLECT));

//        clawFlipButton.whenPressed(
//                claw.clawTo180()
//        ).whenReleased(
//                claw.clawTo0());

        intakeButton.whenPressed(
            new ParallelCommandGroup(
                intakeExt.extendIntakeCmd(),
                intakeClaw.pivotClawCmd(IntakeClaw.IntakePosition.READY)
            ).andThen(
                new WaitCommand(100),
                intakeClaw.openClawCmd()
            )
        ).whenReleased(
            new SequentialCommandGroup(
                intakeClaw.waitFor(250, intakeClaw.pivotClawCmd(IntakeClaw.IntakePosition.COLLECT)),
                intakeClaw.waitFor(250, intakeClaw.closeClawCmd()),
                intakeClaw.rotateTo0(),
                intakeClaw.pivotClawCmd(IntakeClaw.IntakePosition.HOME).alongWith(intakeExt.retractIntakeCmd())

            )
        );

        rotateClawButton.whenPressed(intakeClaw.rotateTo90()).whenReleased(intakeClaw.rotateTo0());
//        transferClawButton.whenPressed(new SequentialCommandGroup(
//            new ElevatorGoTo(elevator, 300).alongWith(new SetArmPosition(arm, Arm.ArmState.COLLECT).withTimeout(200), new OpenClaw(claw)),
//            intakeClaw.waitFor(500, intakeClaw.pivotClawCmd(IntakeClaw.IntakePosition.STORE)),
//            new ElevatorGoTo(elevator, 0),
//            new CloseClaw(claw),
//            new WaitCommand(500),
//            intakeClaw.openClawCmd(),
//            new ElevatorGoTo(elevator, 300)
//        ));



        goToScoreButton.whenPressed(arm.goToPosCmd(Arm.ArmState.SCORE).andThen(
                                new ParallelCommandGroup(
                                        claw.closeClawCommand(),
                                        arm.elbowGoToPosCmd(Arm.ArmState.SCORE),
                                        intakeClaw.pivotClawCmdBlocking(IntakeClaw.IntakePosition.MOVE)
                                )

                ));
//  .whenReleased(new InstantCommand(()-> arm.goToPos(Arm.ArmState.COLLECT)))


//       armButton.whenHeld(new InstantCommand(() -> arm.goToPos(Arm.ArmState.SCORE)).andThen(new SequentialCommandGroup(
//                        claw.clawTo180())))
//                        claw.closeClawCommand()
//                ))
//        .whenReleased(
//                new SequentialCommandGroup(
//                        arm.goToPosCmd(Arm.ArmState.FURTHER),
//                        new WaitCommand(1000),
//                        claw.openClawCommand(),
//                        arm.goToPosCmd(Arm.ArmState.COLLECT),
//                        claw.clawTo0()
//                )
//        );
        pickUpButton.whenPressed(new SequentialCommandGroup(
                new ParallelCommandGroup(
                 claw.closeClawCommand(),
                 new SetArmPosition(arm, Arm.ArmState.COLLECT))

        ));

        clawButton.whenPressed(new OpenClaw(claw))
                .whenReleased(new CloseClaw(claw))

//        elevatorUpButton.whenHeld(
//                new SequentialCommandGroup(
////                        new ElevatorGoTo(elevator, 1900),
////                        new WaitCommand(500),
////                        new SetArmPosition(arm, Arm.ArmState.SCORE)
//                )
//        )


               ;

        elevatorDownButton.whenPressed(new ElevatorGoTo(elevator, 0));

        CommandScheduler.getInstance().setDefaultCommand(elevator, new ManualElevatorCommand(elevator,
            () -> (operator.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) - operator.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)), telemetry));

        drivetrain.setDefaultCommand(new DefaultDrive(drivetrain,
            () -> driver.getLeftX(),
            () -> driver.getLeftY(),
            () -> driver.getRightX()));

        register(arm, intakeClaw, intakeExt, claw);
        schedule(new RunCommand(telemetry::update));

        waitForStart();
        schedule(new InstantCommand(() -> {
            new SetClawPosition(claw, Claw.ClawState.COLLECT);
            intakeClaw.closeIntakeClaw();
            intakeExt.extendTo(0);
            intakeClaw.rotateClawTo(0);
            intakeClaw.pivotTo(IntakeClaw.IntakePosition.HOME);
            drivetrain.setBrakeMode();

        }), new SetArmPosition(arm, Arm.ArmState.COLLECT).withTimeout(10));
    }
}
