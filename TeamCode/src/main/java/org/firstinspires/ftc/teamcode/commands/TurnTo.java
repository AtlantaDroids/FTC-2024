package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.pathgen.PathChain;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

public class TurnTo extends CommandBase {
    private Drivetrain drivetrain;
    private double angle;

    public TurnTo(Drivetrain drivetrain, double angle) {
        this.drivetrain = drivetrain;
        this.angle = angle;

        addRequirements(drivetrain);
    }
    @Override
    public void initialize() {
        this.drivetrain.turnTo(angle);
    }

    @Override
    public void execute() {
        this.drivetrain.update();
    }

    @Override
    public boolean isFinished() {
        return !this.drivetrain.isBusy();
    }
}
