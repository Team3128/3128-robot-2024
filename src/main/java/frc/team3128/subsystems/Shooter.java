package frc.team3128.subsystems;

import common.core.controllers.Controller;
import common.core.controllers.Controller.Type;
import common.core.subsystems.NAR_PIDSubsystem;
import common.hardware.motorcontroller.NAR_CANSpark;
import common.hardware.motorcontroller.NAR_CANSpark.ControllerType;
import common.hardware.motorcontroller.NAR_Motor.Neutral;
import common.utility.shuffleboard.NAR_Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;

import static frc.team3128.Constants.ShooterConstants.*;

import com.revrobotics.CANSparkLowLevel.MotorType;

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
        rightMotor.follow(leftMotor, false);
        leftMotor.setUnitConversionFactor(GEAR_RATIO);

        leftMotor.setNeutralMode(Neutral.COAST);
        rightMotor.setNeutralMode(Neutral.COAST);
    }

    private void setPower(double power){
        disable();
        leftMotor.set(power);
    }

    public double interpolate(double dist){
        return 0;
    }

    public double getVelocity(){
        return leftMotor.getVelocity();
    }

    @Override
    protected void useOutput(double output, double setpoint) {
        leftMotor.setVolts(output);
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

    @Override
    public void initShuffleboard(){
        super.initShuffleboard();
        NAR_Shuffleboard.addSendable("Commands", "Shooter Commands", this, 0, 3);
        NAR_Shuffleboard.addData(getSubsystem(), "left Voltge", leftMotor.getAppliedOutput());
    }

}