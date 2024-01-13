package frc.team3128.subsystems;

import common.core.controllers.Controller;
import common.core.controllers.Controller.Type;
import common.core.subsystems.NAR_PIDSubsystem;
import common.hardware.motorcontroller.NAR_CANSparkMax;
import common.hardware.motorcontroller.NAR_Motor.Neutral;
import common.utility.shuffleboard.NAR_Shuffleboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.team3128.Constants.ShooterConstants.*;
import static frc.team3128.Constants.ManipulatorConstants.*;

public class Shooter extends NAR_PIDSubsystem{
    
    public NAR_CANSparkMax m_leftmotor;
    public NAR_CANSparkMax m_rightmotor;
    private static Shooter instance;
    public SetpointTest setpointtest = new SetpointTest("Shooter Non Trap", 200, 20.0);
    
    public Shooter(){
        super(new Controller(PIDConstants, Type.VELOCITY));
        configMotors();
    }
    
    public static synchronized Shooter getInstance(){
        if(instance == null){
            instance = new Shooter();
        }
        return instance;
    }
    
    public void configMotors(){
        m_leftmotor = new NAR_CANSparkMax(MANIP_LEFT_MOTOR_ID);
        m_rightmotor = new NAR_CANSparkMax(MANIP_RIGHT_MOTOR_ID);
        m_leftmotor.setInverted(false);
        m_rightmotor.follow(m_leftmotor, true);
        m_leftmotor.setUnitConversionFactor(MANIP_GEAR_RATIO);
        m_leftmotor.setNeutralMode(Neutral.COAST);
        m_rightmotor.setNeutralMode(Neutral.COAST);
    }
    
    public double getCurrent(){
        return m_rightmotor.getStallCurrent();
    }
    
    public boolean hasObjectPresent(){
        return Math.abs(getCurrent()) > ABSOLUTE_THRESHOLD;
    }
    
    public void setPower(double power){
        disable();
        m_leftmotor.set(power);
    }
    
    public void intake(){
        setPower(INTAKE_POWER);
    }
    
    public void outtake(){
        setPower(SHOOTING_POWER);
    }
    
    public void stopShooter(){
        setPower(0);
    }
    
    @Override
    protected void useOutput(double output, double setpoint) {
        m_leftmotor.set(output);
    }

    @Override
    protected double getMeasurement() {
        return m_leftmotor.getVelocity();
    }
    
    public CommandBase shoot(double setpoint){
        return runOnce(() -> startPID(setpoint));
    }
    
    public CommandBase runIntake(double power){
        return runOnce(() -> setPower(power));
    }
}
