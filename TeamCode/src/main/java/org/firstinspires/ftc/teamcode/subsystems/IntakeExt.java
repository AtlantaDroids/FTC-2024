package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeExt extends SubsystemBase {
    private final Servo intakeExt;
    public IntakeExt(HardwareMap hMap) {
        this.intakeExt = hMap.get(Servo.class, "IntakeExt");
    }
    public void extendTo(double position){
        intakeExt.setPosition(position);
    }
    public Command extendIntakeCmd(){
        return new RunCommand(() -> extendTo(1), this).withTimeout(1000);

    }
    public Command retractIntakeCmd(){
        return new RunCommand(() -> extendTo(0), this).withTimeout(1000);

    }
}
