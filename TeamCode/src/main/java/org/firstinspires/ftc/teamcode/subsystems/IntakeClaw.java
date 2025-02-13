package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.function.DoubleSupplier;

public class IntakeClaw extends SubsystemBase {
    private final Servo intakeClaw;
    private final Servo clawPivot;
    private final Servo intakePivot;
    private final IntakePosition intakePosition;
    public enum IntakePosition {
        HOME(0.5),
        COLLECT(0.2),
        READY(0.4),
        STORE(0.98);

        public final double pos;
        private IntakePosition(double pos) {
            this.pos = pos;
        }

    }
    public IntakeClaw(HardwareMap hMap) {
        this.intakeClaw = hMap.get(Servo.class, "IntakeClaw");
        this.clawPivot = hMap.get(Servo.class, "ClawPivot");
        this.intakePosition = IntakePosition.HOME;
        this.intakePivot = hMap.get(Servo.class, "IntakePivot");
    }

    public void openIntakeClaw(){
        intakeClaw.setPosition(0);

    }

    public void closeIntakeClaw(){
        intakeClaw.setPosition(0.25);

    }

    public void rotateClawTo(double theta){
        clawPivot.setPosition(theta);
    }

    public void pivotTo(IntakePosition pivot){
        intakePivot.setPosition(pivot.pos);
    }
    public Command pivotClawCmd(IntakePosition intakePosition){
        return new InstantCommand(()-> pivotTo(intakePosition));
    }
    public Command openClawCmd() {
        return new InstantCommand(this::openIntakeClaw, this);
    }

    public Command closeClawCmd() {
        return new InstantCommand(this::closeIntakeClaw, this);
    }

    public Command openClawCmdBlocking() {
        return openClawCmd().andThen(new WaitCommand(100));
    }

    public Command closeClawCmdBlocking() {
        return closeClawCmd().andThen(new WaitCommand(100));
    }

    public Command pivotClawCmdBlocking(IntakePosition pos) {
        return pivotClawCmd(pos).andThen(new WaitCommand(100));
    }

    public Command rotateClawToCmd(double theta){
        return new InstantCommand(()-> rotateClawTo(theta), this);
    }

    public Command rotateClawToCmd(DoubleSupplier sup){
        return new InstantCommand(()-> rotateClawTo(sup.getAsDouble()), this);
    }

    public Command waitFor(int millis, Command... commands) {
        return new ParallelDeadlineGroup(new WaitCommand(millis), commands);
    }

    public Command rotateTo90(){
        return this.rotateClawToCmd(0.4);
    }

    public Command rotateTo0(){
        return this.rotateClawToCmd(0.04);
    }
    public Command rotateToThird(){
        return this.rotateClawToCmd(0.5);
    }



}
