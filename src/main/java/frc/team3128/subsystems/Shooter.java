package frc.team3128.subsystems;

import common.core.controllers.Controller;
import common.core.controllers.Controller.Type;
import common.core.subsystems.NAR_PIDSubsystem;
import common.hardware.motorcontroller.NAR_CANSpark;
import common.hardware.motorcontroller.NAR_CANSpark.ControllerType;
import common.hardware.motorcontroller.NAR_Motor.Neutral;
import common.utility.narwhaldashboard.NarwhalDashboard;
import common.utility.narwhaldashboard.NarwhalDashboard.State;
import edu.wpi.first.wpilibj2.command.Command;

import static frc.team3128.Constants.ShooterConstants.*;

public class Shooter extends NAR_PIDSubsystem {

    public NAR_CANSpark leftMotor;
    public NAR_CANSpark rightMotor;

    private static Shooter instance;

    private Shooter(){
        super(new Controller(PIDConstants, Type.VELOCITY));
        setConstraints(MIN_RPM, MAX_RPM);
        setTolerance(TOLERANCE);
        configMotors();
        initShuffleboard();
        NarwhalDashboard.getInstance().checkState(getName(), ()-> getRunningState());
    }

    public static synchronized Shooter getInstance(){
        if(instance == null){
            instance = new Shooter();
        }
        return instance;
    }

    private void configMotors(){
        leftMotor = new NAR_CANSpark(LEFT_MOTOR_ID, ControllerType.CAN_SPARK_FLEX);
        rightMotor = new NAR_CANSpark(RIGHT_MOTOR_ID, ControllerType.CAN_SPARK_FLEX);
        
        leftMotor.setInverted(true);
        rightMotor.setInverted(true);
        leftMotor.setUnitConversionFactor(GEAR_RATIO);

        leftMotor.setNeutralMode(Neutral.COAST);
        rightMotor.setNeutralMode(Neutral.COAST);
    }

    private void setPower(double power){
        disable();
        leftMotor.set(power);
    }

    @Override
    protected void useOutput(double output, double setpoint) {
        leftMotor.setVolts(output);
        rightMotor.setVolts(Math.max(0, (setpoint - 2000) * PIDConstants.kV));
    }

    @Override
    public double getMeasurement() {
        return leftMotor.getVelocity();
    }

    public Command shoot(double setpoint){
        return runOnce(() -> startPID(setpoint));
    }

    public Command setShooter(double power) {
        return runOnce(()-> setPower(power));
    }

    public Command runBottomRollers(double power) {
        return runOnce(()-> rightMotor.set(power));
    }

    public State getRunningState() {
        if (rightMotor.getState() != State.DISCONNECTED && leftMotor.getState() != State.DISCONNECTED) {
            return State.RUNNING; 
        }
        if (rightMotor.getState() != State.DISCONNECTED) {
            return State.PARTIALLY_RUNNING; 
        }
        return State.DISCONNECTED;
    }
}