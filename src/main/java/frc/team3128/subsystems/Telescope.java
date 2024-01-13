package frc.team3128.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.team3128.Constants.TelescopeConstants.*;

import common.core.controllers.Controller;
import common.core.controllers.Controller.Type;
import common.core.subsystems.NAR_PIDSubsystem;
import common.hardware.motorcontroller.NAR_CANSparkMax;
import common.hardware.motorcontroller.NAR_Motor.Neutral;

public class Telescope extends NAR_PIDSubsystem {
    
    private static Telescope instance;
    
    private NAR_CANSparkMax m_teleMotor;
    
    public SetpointTest setpointtest = new SetpointTest("Telescope Non Trap", 90, 2000.0);
    
    
    public Telescope() {
        super(new Controller(PIDConstants, Type.POSITION));
        configMotors();
    }
    
    public static synchronized Telescope getInstance(){
        if (instance == null){
            instance = new Telescope();
        }
        return instance;
    }
    
    private void configMotors() {
        m_teleMotor = new NAR_CANSparkMax(TELE_MOTOR_ID);
        m_teleMotor.setCurrentLimit(TELE_CURRENT_LIMIT);
        m_teleMotor.enableVoltageCompensation(12.0);
        m_teleMotor.setUnitConversionFactor(TELE_RATIO); //CHANGE THIS!!
        m_teleMotor.setNeutralMode(Neutral.COAST);
    }
    
    public void extend(){
        setPower(TELE_POWER);
    }
    
    public void retract(){
        setPower(-TELE_POWER);
    }
    
    public double getDist(){
        return -m_teleMotor.getPosition();
    }
    
    public void setPower(double power) {
        disable();
        m_teleMotor.set(power);
    }
    @Override
    protected void useOutput(double output, double setpoint) {
        m_teleMotor.set(output);
    }

    @Override
    protected double getMeasurement() {
        return getDist();
    }
    
    public CommandBase moveTele(double setpoint){
        return runOnce(() -> startPID(setpoint));
    }
    
}
