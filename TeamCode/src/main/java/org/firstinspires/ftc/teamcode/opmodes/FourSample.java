package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.FollowPathChain;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.IntakeClaw;
import org.firstinspires.ftc.teamcode.subsystems.IntakeExt;

@Autonomous
public class FourSample extends CommandOpMode {
    public Drivetrain drivetrain;
    public IntakeExt intakeExt;
    public IntakeClaw intakeClaw;


    @Override
    public void initialize() {
        drivetrain = new Drivetrain(this.hardwareMap, new Pose(10, 72.32, Math.toRadians(180)), telemetry);
        intakeExt = new IntakeExt(this.hardwareMap);
        intakeClaw = new IntakeClaw(this.hardwareMap);

        register(drivetrain, intakeClaw, intakeExt);
        waitForStart();
        schedule(new RunCommand(telemetry::update));
    }
}
