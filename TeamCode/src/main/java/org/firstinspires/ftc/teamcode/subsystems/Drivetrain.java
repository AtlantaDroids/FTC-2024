package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

import java.util.Collections;
import java.util.Set;


public class Drivetrain extends SubsystemBase {
    private static final int kFrontLeft = 0;
    private static final int kFrontRight = 1;
    private static final int kBackLeft = 2;
    private static final int kBackRight = 3;



    private Motor frontLeft;
    private Motor backLeft;
    private Motor frontRight;
    private Motor backRight;

    private RevIMU imu;

    private final Follower follower;
    private final Telemetry telemetry;
    private final Pose startPose;

//    private static Drivetrain instance;
    public Drivetrain(HardwareMap hmap, Pose pose, Telemetry telemetry) {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hmap);
        follower.setStartingPose(pose);

        startPose = pose;
        this.telemetry = telemetry;
    }

    public void reset(){
    }

    private double clipRange(double value) {
        return value <= -1.0 ? -1.0
                : value >= 1.0 ? 1.0
                : value;
    }

    public Pose getPose() {
        return this.follower.getPose();
    }

    /**
     * Normalize the wheel speeds
     */
    private void normalize(double[] wheelSpeeds, double magnitude) {
        double maxMagnitude = Math.abs(wheelSpeeds[0]);
        for (int i = 1; i < wheelSpeeds.length; i++) {
            double temp = Math.abs(wheelSpeeds[i]);
            if (maxMagnitude < temp) {
                maxMagnitude = temp;
            }
        }
        for (int i = 0; i < wheelSpeeds.length; i++) {
            wheelSpeeds[i] = (wheelSpeeds[i] / maxMagnitude) * magnitude;
        }

    }

    /**
     * Normalize the wheel speeds
     */
    private void normalize(double[] wheelSpeeds) {
        double maxMagnitude = Math.abs(wheelSpeeds[0]);
        for (int i = 1; i < wheelSpeeds.length; i++) {
            double temp = Math.abs(wheelSpeeds[i]);
            if (maxMagnitude < temp) {
                maxMagnitude = temp;
            }
        }
        if (maxMagnitude > 1) {
            for (int i = 0; i < wheelSpeeds.length; i++) {
                wheelSpeeds[i] = (wheelSpeeds[i] / maxMagnitude);
            }
        }
    }

    @Override
    public void periodic() {
        this.follower.update();
        telemetry.addData("Yaw", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Drivetrain X:", this.getPose().getX());
        telemetry.addData("Drivetrain Y:", this.getPose().getY());
    }

    public void driveArcade(double forwardSpeed, double strafeSpeed, double turnSpeed) {
        this.fieldCentricDrive(-forwardSpeed, strafeSpeed, turnSpeed);
    }

    public void driveFieldCentric(double forwardSpeed, double strafeSpeed, double turnSpeed) {
        this.fieldCentricDrive(forwardSpeed, strafeSpeed, turnSpeed);
    }

    private void fieldCentricDrive(double forwardSpeed, double strafeSpeed, double turnSpeed) {

//        strafeSpeed = clipRange(strafeSpeed);
//        forwardSpeed = clipRange(forwardSpeed);
//        turnSpeed = clipRange(turnSpeed);
//        Vector2d input = new Vector2d(strafeSpeed, forwardSpeed);
//        input = input.rotateBy(-gyroAngle);
//
//        double theta = input.angle();
//
//        double[] wheelSpeeds = new double[4];
//        wheelSpeeds[kFrontLeft] = Math.sin(theta + Math.PI / 4);
//        wheelSpeeds[kFrontRight] = Math.sin(theta - Math.PI / 4);
//        wheelSpeeds[kBackLeft] = Math.sin(theta - Math.PI / 4);
//        wheelSpeeds[kBackRight] = Math.sin(theta + Math.PI / 4);
//
//        normalize(wheelSpeeds, input.magnitude());
//
//        wheelSpeeds[kFrontLeft] += turnSpeed;
//        wheelSpeeds[kFrontRight] -= turnSpeed;
//        wheelSpeeds[kBackLeft] += turnSpeed;
//        wheelSpeeds[kBackRight] -= turnSpeed;

//        normalize(wheelSpeeds);

        this.follower.setTeleOpMovementVectors(forwardSpeed, strafeSpeed, turnSpeed, false);

    }

    public void setTeleOpMode(){
        this.follower.startTeleopDrive();
    }

    public PathBuilder getTrajectoryBuilder(Pose2d initalPose) {
        return this.follower.pathBuilder();
    }

    public void followPath(PathChain pathChain, boolean hold) {
        this.follower.followPath(pathChain, hold);
    }

    public void turnTo(double angle) {
        this.follower.turnTo(angle);
    }

    public void update() {
        this.follower.update();
    }

    public boolean isBusy() {
        return this.follower.isBusy();
    }

    public void resetFollower() {
        this.follower.setPose(startPose);
    }
}
