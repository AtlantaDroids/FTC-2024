package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Elevator extends SubsystemBase {
    private static final double TICKS_PER_MM = 0.335;
    private static final double KP = 0.01;
    private static final double KF = 0.07;
    private Motor elevatorLeft;
    private Motor elevatorRight;
    private DigitalChannel limitSwitch;
    private Telemetry telemetry;

    private double elevatorPower;
    private double target;
    public Elevator(HardwareMap hMap, Telemetry telemetry){
        this.elevatorLeft = new Motor(hMap, "ElevatorLeft");
        this.limitSwitch = hMap.get(DigitalChannel.class, "limitSwitch");
        //this.elevatorRight = new Motor(hMap, "ElevatorRight");

        this.telemetry = telemetry;
        this.elevatorLeft.setInverted(false);
        this.elevatorLeft.encoder.setDirection(Motor.Direction.FORWARD);

    }

    @Override
    public void periodic() {

        //elevatorRight.set(this.elevatorPower + KF);
//        if (!limitSwitch.getState() && this.target == 0) {
//            elevatorLeft.resetEncoder();
//        }
        elevatorLeft.set(this.elevatorPower + KF);


        telemetry.addData("Pos", (this.elevatorLeft.getCurrentPosition() * TICKS_PER_MM) / 25.4 );
        telemetry.addData("Elevator Power", this.elevatorPower);
        telemetry.addData("Elevator Target", this.target);
    }

    public void setTarget(double target) {
        if (target > 80) { // specimen height is 31
            this.target = 80 * 25.4;
            return;
        }

        if (target < 0) {
            this.target = 0;
            return;
        }


        this.target = target * 25.4;

    }

    public void goToPos() {
        int currentPos = this.elevatorLeft.getCurrentPosition(); //Right
        double currentPosMM = currentPos * TICKS_PER_MM;
        double error = target - currentPosMM;
        this.elevatorPower = error * KP;
    }

    public boolean atTarget() {
        int currentPos = this.elevatorLeft.getCurrentPosition(); //Right
        double currentPosMM = currentPos * TICKS_PER_MM;
        //  target + 5 > currentPosMM && target - 5 < currentPosMM
        if (currentPosMM < target + 10 &&
            currentPosMM > target - 10) {
            return true;
        }

        return false;

    }

    public void manualControl(double pow) {
        telemetry.addData("pow", pow);
        int currentPos = this.elevatorLeft.getCurrentPosition(); //Right
        double currentPosMM = currentPos * TICKS_PER_MM;
        if (pow > 0 && currentPosMM > 36 * 25.4) {
            elevatorPower = 0;
            return;
        } else if (pow < 0 && currentPos <= 15) {
            elevatorPower = 0;
            return;
        }

        elevatorPower = pow;


    }

}
