package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;

public class SetClawPosition extends CommandBase {

    private final Claw claw;
    private Claw.ClawState clawState = Claw.ClawState.COLLECT;
    public SetClawPosition(Claw claw, Claw.ClawState state) {
        this.claw = claw;
        this.clawState = clawState;

    }

    @Override
    public void execute() {
        this.claw.goToPos(this.clawState);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
