package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw extends SubsystemBase {
    private final Servo claw;
    private final Servo clawFlip;


    public Claw(HardwareMap hmap) {
        this.claw = hmap.get(Servo.class, "claw");
        this.clawFlip = hmap.get(Servo.class, "ClawFlip");
    }

    public void openClaw() {
        this.claw.setPosition(0.97);
    }

    public void closeClaw() {
        this.claw.setPosition(0);
    }

    public void claw0(){
        this.clawFlip.setPosition(0.72);
    }
    public void claw180(){
        this.clawFlip.setPosition(0);
    }


    public Command clawTo180(){
        return new RunCommand(this::claw180, this).withTimeout(500);
    }
    public Command clawTo0(){
        return new RunCommand(this::claw0, this).withTimeout(500);
    }


    public Command closeClawCommand() {
        return new RunCommand(this::closeClaw, this).withTimeout(100);
    }

    public Command openClawCommand() {
        return new RunCommand(this::openClaw, this).withTimeout(100);
    }
}
