package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Elevator extends SubsystemBase {
    private static final double TICKS_PER_MM = 0.335;
    private static final double MAX_HEIGHT = 2100;
    private static final double KP = 0.008;
    private static final double KF = 0.15;
    private final Motor elevatorLeft;
    private Motor elevatorRight;
    private final DigitalChannel limitSwitch;
    private final Telemetry telemetry;

    private double elevatorPower;
    private double target;
    public Elevator(HardwareMap hMap, Telemetry telemetry){
        this.elevatorLeft = new Motor(hMap, "ElevatorLeft");
        this.limitSwitch = hMap.get(DigitalChannel.class, "limitSwitch");
        //this.elevatorRight = new Motor(hMap, "ElevatorRight");

        this.telemetry = telemetry;
        this.elevatorLeft.setInverted(false);
        this.elevatorLeft.encoder.setDirection(Motor.Direction.FORWARD);

        // Zero encoder when at the limit switch
        new Trigger(this::atLimitSwitch).whenActive(this::stopAndReset);

    }

    @Override
    public void periodic() {

        //elevatorRight.set(this.elevatorPower + KF);
//        if (!limitSwitch.getState() && this.target == 0) {
//            elevatorLeft.resetEncoder();
//        }
        elevatorLeft.set(this.elevatorPower +KF);

        int currentPos = this.elevatorLeft.getCurrentPosition(); //Right

        telemetry.addData("Pos",this.elevatorLeft.getCurrentPosition());
        telemetry.addData("Elevator Power", this.elevatorPower);
        telemetry.addData("Elevator Target", this.target);
        telemetry.addData("Elevator Error", this.target - currentPos);
    }

    public boolean atLimitSwitch() {
        return !this.limitSwitch.getState();
    }

    public void stopAndReset() {
        this.elevatorPower = 0;
        this.elevatorLeft.resetEncoder();
    }

    public void stop() {
        this.elevatorPower = 0;
    }

    public void setTarget(double target) {
        if (target > MAX_HEIGHT) { // specimen height is 31
            this.target = MAX_HEIGHT;
            return;
        }

        if (target < 0) {
            this.target = 0;
            return;
        }


        this.target = target;

    }

    public void goToPos() {
        int currentPos = this.elevatorLeft.getCurrentPosition(); //Right
        double error = target - currentPos;
        this.elevatorPower = error * KP;
    }

    public boolean atTarget() {
        int currentPos = this.elevatorLeft.getCurrentPosition(); //Right
        //  target + 5 > currentPosMM && target - 5 < currentPosMM
        return currentPos < target + 20 &&
            currentPos > target - 20;

    }

    public void manualControl(double pow) {
        telemetry.addData("pow", pow);
        int currentPos = this.elevatorLeft.getCurrentPosition(); //Right
        if (pow > 0 && currentPos > MAX_HEIGHT) {
            elevatorPower = 0;
            return;
        } else if (pow < 0 && currentPos <= 15) {
            elevatorPower = 0;
            return;
        }

        elevatorPower = pow;


    }

}
