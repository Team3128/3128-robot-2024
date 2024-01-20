package frc.team3128.subsystems;

import common.hardware.motorcontroller.NAR_CANSparkMax;
import common.hardware.motorcontroller.NAR_CANSparkMax.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SimpleMotorSubsystem extends SubsystemBase{
    
    private static SimpleMotorSubsystem instance;

    private NAR_CANSparkMax motor;

    public static synchronized SimpleMotorSubsystem getInstance() {
        if (instance == null) {
            instance = new SimpleMotorSubsystem();
        }
        return instance;
    }

    public SimpleMotorSubsystem(){
        configMotor();
    }

    public void configMotor(){
        motor = new NAR_CANSparkMax(21);
        motor.setCurrentLimit(40);
        motor.setStatusFrames(SparkMaxConfig.POSITION);
    }

    public void setVoltage(double voltage){
        motor.setVolts(voltage);
    }

    public void resetPosiiton(double position){
        motor.resetPosition(position);
    }

    public double getPosition(){
        return motor.getPosition();
    }

    public double getVelocity(){
        return motor.getVelocity();
    }

    public void stop(){
        setVoltage(0);
    }
}
