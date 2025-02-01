package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.function.DoubleSupplier;

public class IntakeClaw extends SubsystemBase {
    private Servo intakeClaw;
    private Servo clawPivot;
    private Servo intakeExt;
    private Servo intakePivot;
    private IntakePosition intakePosition;
    public enum IntakePosition {
        HOME,
        COLLECT,
        STORE


}
    public IntakeClaw(HardwareMap hMap) {
        this.clawPivot = hMap.get(Servo.class, "ClawPivot");
        this.intakeExt = hMap.get(Servo.class, "IntakeExt");
        this.intakePosition = IntakePosition.HOME;
        this.intakePivot = hMap.get(Servo.class, "IntakePivot");
    }

    public void openIntakeClaw(){
        intakeClaw.setPosition(1);

    }

    public void closeIntakeClaw(){
        intakeClaw.setPosition(0);

    }

    public void rotateClawTo(double theta){
        clawPivot.setPosition(theta);
    }
    public void extendTo(double position){
        intakeExt.setPosition(position);
    }
    public void pivotTo(IntakePosition pivot){
        switch (pivot) {
            case HOME:
                intakePivot.setPosition(0.5);
                break;
            case STORE:
                intakePivot.setPosition(0.2);
                break;
            case COLLECT:
                intakePivot.setPosition(1);
                break;
        }
    }
    public Command pivotClawCmd(IntakePosition intakePosition){
        return new RunCommand(()-> pivotTo(intakePosition), this).withTimeout(500);
    }
    public Command openClawCmd() {
        return new RunCommand(this::openIntakeClaw, this).withTimeout(500);
    }

    public Command closeClawCmd() {
        return new RunCommand(this::closeIntakeClaw, this).withTimeout(500);
    }

    public Command extendIntakeCmd(){
        return new RunCommand(() -> extendTo(1), this).withTimeout(1000);

    }
    public Command retractIntakeCmd(){
        return new RunCommand(() -> extendTo(0), this).withTimeout(1000);

    }

    public Command rotateClawToCmd(double theta){
        return new RunCommand(()-> rotateClawTo(theta), this).withTimeout(500);
    }

    public Command rotateClawToCmd(DoubleSupplier sup){
        return new RunCommand(()-> rotateClawTo(sup.getAsDouble()), this).withTimeout(500);
    }

    public Command rotateTo90(){
        return this.rotateClawToCmd(0.3);
    }

    public Command rotateTo0(){
        return this.rotateClawToCmd(0);
    }




}
