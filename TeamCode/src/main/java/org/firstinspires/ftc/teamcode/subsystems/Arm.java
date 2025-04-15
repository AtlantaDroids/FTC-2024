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
        WAIT
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
                this.armServoLeft.setPosition(0.39);
                this.armServoRight.setPosition(0.39);
//                this.elbow.setPosition(0.78);
                break;
            case TRANSITION:
                this.armServoLeft.setPosition(0.46);
                this.armServoRight.setPosition(0.46);
//                this.elbow.setPosition(0.05);
                break;
            case COLLECT:
                this.armServoLeft.setPosition(0.17);
                this.armServoRight.setPosition(0.17);
//                this.elbow.setPosition(0.48);
                break;

            case WAIT:
                this.armServoLeft.setPosition(0.38);
                this.armServoRight.setPosition(0.38);
                break;        }
    }
    public void elbowGoToPos(ArmState state) {
        switch(state) {
            case SCORE:
                this.elbow.setPosition(0.75);
                break;
            case TRANSITION:
                this.elbow.setPosition(0.11);
                break;
            case COLLECT:
                this.elbow.setPosition(0.44);
                break;
            case WAIT:
                this.elbow.setPosition(0.65);
        }
    }

    public Command goToPosCmd(ArmState state) {
        return new RunCommand(() -> goToPos(state)).raceWith(new WaitCommand(500));
    }
    public Command elbowGoToPosCmd(ArmState state) {
        return new RunCommand(() -> elbowGoToPos(state)).raceWith(new WaitCommand(500));
    }

}
