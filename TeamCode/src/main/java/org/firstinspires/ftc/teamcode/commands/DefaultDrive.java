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
    public void initialize() {
        super.initialize();
        this.drivetrain.setTeleOpMode();
    }

    @Override
    public void execute() {
        double forward = Math.pow(fwd.getAsDouble(), 2) * Math.signum(fwd.getAsDouble());
        double translation = Math.pow(strafe.getAsDouble(), 2)*Math.signum(strafe.getAsDouble());
        double turn = Math.pow(rot.getAsDouble(),2)*Math.signum(rot.getAsDouble());
        drivetrain.driveFieldCentric(forward, -translation, -turn);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
