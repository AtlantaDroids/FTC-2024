//package org.firstinspires.ftc.teamcode.opmodes;
//
//import com.acmerobotics.roadrunner.Action;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.arcrobotics.ftclib.command.CommandOpMode;
//import com.arcrobotics.ftclib.command.ParallelCommandGroup;
//import com.arcrobotics.ftclib.command.RunCommand;
//import com.arcrobotics.ftclib.command.SequentialCommandGroup;
//
//import org.firstinspires.ftc.teamcode.commands.CloseClaw;
//import org.firstinspires.ftc.teamcode.commands.ElevatorGoTo;
//import org.firstinspires.ftc.teamcode.commands.PivotIntake;
//import org.firstinspires.ftc.teamcode.commands.RetractIntake;
//import org.firstinspires.ftc.teamcode.commands.TrajectoryCommand;
//import org.firstinspires.ftc.teamcode.subsystems.Claw;
//import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
//import org.firstinspires.ftc.teamcode.subsystems.Elevator;
//import org.firstinspires.ftc.teamcode.subsystems.Intake;
//
//
//public class FiveSpecimenAuto extends CommandOpMode{
//    private Drivetrain drivetrain;
//    private Claw claw;
//    private Elevator elevator;
//    private Intake intake;

//    @Override
//    public void initialize() {
//        drivetrain = new Drivetrain(hardwareMap, new Pose2d(0, -61, Math.toRadians(180)), telemetry);
//        claw = new Claw(hardwareMap);
//        elevator = new Elevator(hardwareMap, telemetry);
//        intake = new Intake(hardwareMap);
//
//        Action pushFirstSample = drivetrain.getTrajectoryBuilder(new Pose2d());
//
//        Action ScoreFirstSample = drivetrain.getTrajectoryBuilder(new Pose2d());
//
//        Action pushSecondSample = drivetrain.getTrajectoryBuilder(new Pose2d());
//
//        Action ScoreSecondSample = drivetrain.getTrajectoryBuilder(new Pose2d());
//
//        Action pushThirdSample = drivetrain.getTrajectoryBuilder(new Pose2d());
//
//        Action ScoreThirdSample = drivetrain.getTrajectoryBuilder(new Pose2d());
//
//        Action pushFourthSample = drivetrain.getTrajectoryBuilder(new Pose2d());
//
//        Action ScoreFourthSample = drivetrain.getTrajectoryBuilder(new Pose2d());
//
//
//
//        Action driveToFirstScore = drivetrain.getTrajectoryBuilder(new Pose2d(0, -61, Math.toRadians(180)))
//                .strafeTo(new Vector2d(0, -31))
//                .build();
//        schedule(new RunCommand(() -> telemetry.update()));
//        register(drivetrain, claw, elevator);
//        waitForStart();
//
//        schedule(new SequentialCommandGroup(
//                new RetractIntake(intake),
//                new PivotIntake(Intake.IntakeState.HOME, intake),
//                new CloseClaw(claw),
//                new ParallelCommandGroup(
//                        new TrajectoryCommand(driveToFirstScore, drivetrain),
//                        new ElevatorGoTo(elevator, 25).withTimeout(2000))






//    }
//
//}
