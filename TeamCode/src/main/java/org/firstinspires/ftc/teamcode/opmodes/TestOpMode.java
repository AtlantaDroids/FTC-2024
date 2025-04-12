package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.IntakeClaw;
import org.firstinspires.ftc.teamcode.subsystems.IntakeExt;
@TeleOp
public class TestOpMode extends CommandOpMode {

    public Servo armRight;
    public Servo armLeft;
    public Servo clawFlip;
    public double shoulderPos = 0.2;
    public double elbowPos = 0.2;
    @Override
    public void initialize() {
//        driver   = new GamepadEx(gamepad1);
        GamepadEx operator = new GamepadEx(gamepad2);

        armRight = hardwareMap.get(Servo.class, "ArmRight");
        armLeft = hardwareMap.get(Servo.class, "ArmLeft");
        clawFlip = hardwareMap.get(Servo.class, "ClawFlip");
        operator.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(()->shoulderPos = shoulderPos+0.01);
        operator.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(()->shoulderPos = shoulderPos-0.01);
        operator.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(()->elbowPos = elbowPos+0.01);
        operator.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(()->elbowPos = elbowPos-0.01);

        waitForStart();
        schedule(new RunCommand(()-> {
            telemetry.addData("shoulderPos", shoulderPos);
            telemetry.addData("elbowPos", elbowPos);
            telemetry.addData("clawFlip", clawFlip.getPosition());
            armLeft.setPosition(shoulderPos);
            armRight.setPosition(shoulderPos);
            clawFlip.setPosition(elbowPos);
                })
        );
        schedule(new RunCommand(telemetry::update));


    }

}
