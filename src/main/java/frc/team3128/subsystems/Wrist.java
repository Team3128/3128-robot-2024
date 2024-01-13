package frc.team3128.subsystems;

import common.core.controllers.Controller;
import common.core.controllers.Controller.Type;
import common.core.subsystems.NAR_PIDSubsystem;
import common.hardware.motorcontroller.NAR_CANSparkMax;
import common.hardware.motorcontroller.NAR_Motor.Neutral;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team3128.Constants.TelescopeConstants.*;

import static frc.team3128.Constants.WristConstants.*;
import static org.junit.Assert.fail;


public class Wrist extends NAR_PIDSubsystem{
    
    private static Wrist instance;
    private NAR_CANSparkMax m_wristMotor;
    public double offset;
    private Encoder wristEncoder;
    public SetpointTest setpointtest = new SetpointTest("Wrist Non Trap", 90, 2000.0);
    
    public Wrist(){
        super(new Controller(PIDConstants, Type.POSITION));
        configMotors();
        getInstance().setTolerance(WRIST_TOLERANCE);
    }
    
    public static synchronized Wrist getInstance(){
        if(instance == null){
            instance = new Wrist();
        }
        return instance;
    }
    
    private void configMotors(){
        //check how to code this: 
        m_wristMotor = new NAR_CANSparkMax(WRIST_MOTOR_ID);
        m_wristMotor.setCurrentLimit(WRIST_CURRENT_LIMIT);
        m_wristMotor.setInverted(false);
        m_wristMotor.setUnitConversionFactor(360);
        m_wristMotor.setNeutralMode(Neutral.COAST);
        
    }
    
    public void setPower(double power){
        disable();
        m_wristMotor.set(power);
    }
    
    //no clue if this is correct
    public void resetwrist() {
        m_wristMotor.resetPosition(ANGLE_OFFSET);;
    }
    
    public void stopwrist(){
        setPower(0);
    }
    
    public double getAngle(){
        return MathUtil.inputModulus(-wristEncoder.get() * 360 - ANGLE_OFFSET,-180, 180);
    }
    @Override
    protected void useOutput(double output, double setpoint) {
        setPower(output);
    }
    
    public boolean atSetpoint() {
        return getController().atSetpoint();
    }

    @Override
    protected double getMeasurement() {
        return getAngle();
    }
    
    public void initShuffleboard() {
        //kill me now i don't wanna do this
    }
    
    public CommandBase wrist(double setpoint){
        return runOnce(() -> startPID(setpoint));
    }
    
    
}
