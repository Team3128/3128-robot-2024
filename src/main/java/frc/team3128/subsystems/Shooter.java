package frc.team3128.subsystems;

import common.core.controllers.Controller;
import common.core.controllers.Controller.Type;
import common.core.subsystems.NAR_PIDSubsystem;
import common.hardware.motorcontroller.NAR_CANSparkMax;
import common.hardware.motorcontroller.NAR_Motor.Neutral;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;

import static frc.team3128.Constants.ShooterConstants.*;

public class Shooter extends NAR_PIDSubsystem {
    
    public NAR_CANSparkMax leftMotor;
    public NAR_CANSparkMax rightMotor;

    private static Shooter instance;
    
    private Shooter(){
        super(new Controller(PIDConstants, Type.VELOCITY));
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
        leftMotor = new NAR_CANSparkMax(LEFT_MOTOR_ID);
        rightMotor = new NAR_CANSparkMax(RIGHT_MOTOR_ID);
        leftMotor.setInverted(false);
        rightMotor.follow(leftMotor, true);
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
        leftMotor.set(MathUtil.clamp(output/12.0, -1, 1));
    }

    @Override
    protected double getMeasurement() {
        return leftMotor.getVelocity();
    }
    
    public Command shoot(double setpoint){
        return runOnce(() -> startPID(setpoint));
    }

    public Command setShooter(double power) {
        return runOnce(()-> setPower(power));
    }
}