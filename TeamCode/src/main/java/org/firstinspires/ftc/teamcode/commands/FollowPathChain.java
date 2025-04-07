package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.pathgen.PathChain;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

public class FollowPathChain extends CommandBase {
    private Drivetrain drivetrain;
    private PathChain pathChain;
    private boolean hold;

    public FollowPathChain(Drivetrain drivetrain, PathChain pathChain, boolean hold) {
        this.drivetrain = drivetrain;
        this.pathChain = pathChain;
        this.hold = hold;

        addRequirements(drivetrain);
    }

    public FollowPathChain(Drivetrain drivetrain, PathChain chain) {
        this(drivetrain, chain, false);
    }

    @Override
    public void initialize() {
        this.drivetrain.followPath(pathChain, hold);
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
