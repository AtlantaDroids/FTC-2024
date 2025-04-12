package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.R;

public class IntakeExt extends SubsystemBase {
    public enum IntakeExtensionState {
        HOME(0),
        //fix later
        DEPLOYED(0.29),

        MOVE(0.1)
        ;

        public final double pos;
        private IntakeExtensionState(double pos) {
            this.pos = pos;
        }
    }

    private final Servo intakeExt;
    private final Servo intakeExtRight;
    private IntakeExtensionState currentState = IntakeExtensionState.HOME;
    public IntakeExt(HardwareMap hMap) {

        this.intakeExt = hMap.get(Servo.class, "IntakeExt");
        this.intakeExtRight = hMap.get(Servo.class, "IntakeExtRight");

    }
    public void extendTo(double position){
        intakeExtRight.setPosition(position);
        intakeExt.setPosition(position);
    }

    private void extendTo(IntakeExtensionState des) {
        intakeExtRight.setPosition(des.pos);
        intakeExt.setPosition(des.pos);
    }


    private void setIntakeHalfExtended() {
        this.currentState = IntakeExtensionState.MOVE;
    }

    private void setIntakeExtended() {
        this.currentState = IntakeExtensionState.DEPLOYED;
    }

    private void setIntakeRetracted() {
        this.currentState = IntakeExtensionState.HOME;
    }

    public Command extendIntakeCmd() {
        if (currentState == IntakeExtensionState.DEPLOYED) {
            return new InstantCommand(() -> extendTo(IntakeExtensionState.DEPLOYED));
        }

        return new RunCommand(() -> extendTo(IntakeExtensionState.DEPLOYED), this)
            .withTimeout(300)
            .whenFinished(this::setIntakeExtended);

    }
    public Command retractIntakeCmd() {
        if (currentState == IntakeExtensionState.HOME) {
            return new InstantCommand(() -> extendTo(IntakeExtensionState.HOME));
        }

        return new RunCommand(() -> extendTo(IntakeExtensionState.HOME), this)
            .withTimeout(500)
            .whenFinished(this::setIntakeRetracted);

    }
}
