package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Arm extends SubsystemBase {
    private final Servo armServoLeft;
    private final Servo armServoRight;

    public enum ArmState {
        INTAKE,
        SCORE,
        COLLECT,
        FURTHER
    }

    public Arm(HardwareMap hMap) {
        this.armServoLeft = hMap.get(Servo.class, "ArmLeft");
        this.armServoRight = hMap.get(Servo.class, "ArmRight");
    }

    public void goToPos(ArmState state) {
        switch(state) {
            case SCORE:
                this.armServoLeft.setPosition(0.7);
                this.armServoRight.setPosition(0.71);
                break;
            case INTAKE:
                this.armServoLeft.setPosition(0);
                this.armServoRight.setPosition(0.01);
                break;
            case COLLECT:
                this.armServoLeft.setPosition(0.27);
                this.armServoRight.setPosition(0.28);
                break;
            case FURTHER:
                this.armServoLeft.setPosition(0.975);
                this.armServoRight.setPosition(0.985);
                break;
        }
    }

    public Command goToPosCmd(ArmState state) {
        return new InstantCommand(() -> goToPos(state));
    }

}
