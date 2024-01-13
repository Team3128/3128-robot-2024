package frc.team3128.subsystems;

import common.core.controllers.Controller;
import common.core.controllers.TrapController;
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
import static frc.team3128.Constants.Shooter2Constants.*;
import static frc.team3128.Constants.ManipulatorConstants.*;
import frc.team3128.subsystems.Swerve;

public class ShooterTrap extends NAR_PIDSubsystem{
    public NAR_CANSparkMax m_leftmotor;
    public NAR_CANSparkMax m_rightmotor;
    private static ShooterTrap instance;
    public SetpointTest setpointtest = new SetpointTest("Shooter Trap", 200, 2000.0);
    
    
    public ShooterTrap(){
        super(new TrapController(PIDConstants, MAX_ACCELERATION));
        configMotors();
    }
    
    public static synchronized ShooterTrap getInstance(){
        if(instance == null){
            instance = new ShooterTrap();
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
        enable();
        m_leftmotor.set(output);
    }

    @Override
    protected double getMeasurement() {
        return m_leftmotor.getVelocity();
    }
    
    public void shoot(double setpoint){
        startPID(setpoint);
    }
    
    public CommandBase runIntake(double power){
        return runOnce(() -> setPower(power));
    }
    
    public double getDist(){
        double x = Swerve.getInstance().getPose().getX();
        double y = Swerve.getInstance().getPose().getY();
        return Math.sqrt(Math.pow(x - SPEAKER_X, 2) + Math.pow(y - SPEAKER_Y, 2));
    }
}
