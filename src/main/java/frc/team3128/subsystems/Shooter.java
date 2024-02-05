package frc.team3128.subsystems;

import common.core.controllers.Controller;
import common.core.controllers.Controller.Type;
import common.core.subsystems.NAR_PIDSubsystem;
import common.hardware.motorcontroller.NAR_CANSparkMax;
import common.hardware.motorcontroller.NAR_Motor.Neutral;
import common.utility.shuffleboard.NAR_Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;

import static frc.team3128.Constants.ShooterConstants.*;

public class Shooter extends NAR_PIDSubsystem {

    public NAR_CANSparkMax leftMotor;
    public NAR_CANSparkMax rightMotor;

    private static Shooter instance;

    private Shooter(){
        super(new Controller(PIDConstants, Type.VELOCITY));
        setConstraints(0, 5700);
        setTolerance(80);
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

}