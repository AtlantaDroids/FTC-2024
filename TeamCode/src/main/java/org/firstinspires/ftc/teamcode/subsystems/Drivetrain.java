package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;


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

    private final MecanumDrive mecanumDrive;

    private final Telemetry telemetry;

//    private static Drivetrain instance;
    public Drivetrain(HardwareMap hmap, Pose2d pose, Telemetry telemetry) {
        this.mecanumDrive = new MecanumDrive(hmap, pose);
        this.telemetry = telemetry;
    }

    private double clipRange(double value) {
        return value <= -1.0 ? -1.0
                : value >= 1.0 ? 1.0
                : value;
    }

    public Pose2d getPose() {
        return this.mecanumDrive.pose;
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
        this.mecanumDrive.updatePoseEstimate();
        telemetry.addData("Yaw", Math.toDegrees(mecanumDrive.pose.heading.log()));
        telemetry.addData("Drivetrain X:", this.getPose().position.x);
        telemetry.addData("Drivetrain Y:", this.getPose().position.y);
    }

    public void driveArcade(double forwardSpeed, double strafeSpeed, double turnSpeed) {
        this.fieldCentricDrive(-forwardSpeed, strafeSpeed, turnSpeed, 0.0);
    }

    public void driveFieldCentric(double forwardSpeed, double strafeSpeed, double turnSpeed) {
        this.fieldCentricDrive(-forwardSpeed, -strafeSpeed, turnSpeed, Math.toDegrees(mecanumDrive.pose.heading.log()) + 90);
    }

    private void fieldCentricDrive(double forwardSpeed, double strafeSpeed, double turnSpeed, double gyroAngle) {

        strafeSpeed = clipRange(strafeSpeed);
        forwardSpeed = clipRange(forwardSpeed);
        turnSpeed = clipRange(turnSpeed);
        Vector2d input = new Vector2d(strafeSpeed, forwardSpeed);
        input = input.rotateBy(-gyroAngle);

        double theta = input.angle();

        double[] wheelSpeeds = new double[4];
        wheelSpeeds[kFrontLeft] = Math.sin(theta + Math.PI / 4);
        wheelSpeeds[kFrontRight] = Math.sin(theta - Math.PI / 4);
        wheelSpeeds[kBackLeft] = Math.sin(theta - Math.PI / 4);
        wheelSpeeds[kBackRight] = Math.sin(theta + Math.PI / 4);

        normalize(wheelSpeeds, input.magnitude());

        wheelSpeeds[kFrontLeft] += turnSpeed;
        wheelSpeeds[kFrontRight] -= turnSpeed;
        wheelSpeeds[kBackLeft] += turnSpeed;
        wheelSpeeds[kBackRight] -= turnSpeed;

        normalize(wheelSpeeds);

        this.mecanumDrive.leftFront.setPower(wheelSpeeds[kFrontLeft]);
        this.mecanumDrive.rightFront.setPower(wheelSpeeds[kFrontRight]);
        this.mecanumDrive.leftBack.setPower(wheelSpeeds[kBackLeft]);
        this.mecanumDrive.rightBack.setPower(wheelSpeeds[kBackRight]);
    }

    public TrajectoryActionBuilder getTrajectoryBuilder(Pose2d initalPose) {
        return this.mecanumDrive.actionBuilder(initalPose);
    }
}
