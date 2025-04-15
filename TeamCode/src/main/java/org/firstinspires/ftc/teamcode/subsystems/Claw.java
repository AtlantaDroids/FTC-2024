package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw extends SubsystemBase {
    private final Servo claw;

    public void goToPos(ClawState clawState) {
    }

    //    private final Servo clawFlip;
    public enum ClawState {
        TRANSITION,
        SCORE,
        COLLECT,
        ;


    }

    public Claw(HardwareMap hmap) {
        this.claw = hmap.get(Servo.class, "claw");
       
    }

    public void openClaw() {
        this.claw.setPosition(0.4);
    }

    public void closeClaw() {
        this.claw.setPosition(0.10);
    }

    


   


    public Command closeClawCommand() {
        return new RunCommand(this::closeClaw, this).withTimeout(200);
    }

    public Command openClawCommand() {
        return new RunCommand(this::openClaw, this).withTimeout(100);
    }
//    public Command SetClawPosition(Claw.ClawState state) {
//        return new InstantCommand(() -> goToPos(state));
//    }



}
