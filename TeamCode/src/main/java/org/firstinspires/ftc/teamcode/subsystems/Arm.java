package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Arm extends SubsystemBase {
    private final Servo armServoLeft;
    private final Servo armServoRight;
    private final Servo elbow;

    public enum ArmState {
        TRANSITION,
        SCORE,
        COLLECT,
//        FURTHER
    }

    public Arm(HardwareMap hMap) {
        this.armServoLeft = hMap.get(Servo.class, "ArmLeft");
        this.elbow = hMap.get(Servo.class, "ClawFlip");
        this.armServoRight = hMap.get(Servo.class, "ArmRight");
    }



    public void goToPos(ArmState state) {
        switch(state) {
            case SCORE:
                this.armServoLeft.setPosition(0.37);
                this.armServoRight.setPosition(0.37);
                this.elbow.setPosition(0.7);
                break;
            case TRANSITION:
                this.armServoLeft.setPosition(0.38);
                this.armServoRight.setPosition(0.38);
                this.elbow.setPosition(0.05);
                break;
            case COLLECT:
                this.armServoLeft.setPosition(0.14);
                this.armServoRight.setPosition(0.14);
                this.elbow.setPosition(0.48);
                break;
        }
    }
    public void clawGoToPos(ArmState state) {
        switch(state) {
            case SCORE:
                this.elbow.setPosition(0.77);
                break;
            case TRANSITION:
                this.elbow.setPosition(0.05);
                break;
            case COLLECT:
                this.elbow.setPosition(0.48);
                break;
        }
    }

    public Command goToPosCmd(ArmState state) {
        return new RunCommand(() -> goToPos(state)).raceWith(new WaitCommand(500));
    }
    public Command elbowGoToPosCmd(ArmState state) {
        return new RunCommand(() -> goToPos(state)).raceWith(new WaitCommand(500));
    }

}
