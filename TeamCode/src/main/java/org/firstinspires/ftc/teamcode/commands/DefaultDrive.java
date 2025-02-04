package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

import java.util.function.DoubleSupplier;

public class DefaultDrive extends CommandBase {
    private final Drivetrain drivetrain;

    private final DoubleSupplier strafe;
    private final DoubleSupplier fwd;
    private final DoubleSupplier rot;
    public DefaultDrive(Drivetrain drivetrain, DoubleSupplier strafe, DoubleSupplier fwd, DoubleSupplier rot) {
        this.drivetrain = drivetrain;

        this.fwd = fwd;
        this.rot = rot;
        this.strafe = strafe;

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        drivetrain.driveFieldCentric(fwd.getAsDouble(), strafe.getAsDouble(), rot.getAsDouble());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
