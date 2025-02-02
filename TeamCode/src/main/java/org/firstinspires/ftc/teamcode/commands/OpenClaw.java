package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Claw;

public class OpenClaw extends CommandBase {
    private final Claw claw;

    public OpenClaw(Claw claw) {
        this.claw = claw;
        addRequirements(claw);
    }

    public void execute() {
        claw.openClaw();
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}

