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

import static frc.team3128.Constants.PivotConstants.*;
import static org.junit.Assert.fail;


public class Pivot extends NAR_PIDSubsystem{
    
    private static Pivot instance;
    private NAR_CANSparkMax m_pivotMotor;
    public double offset;
    private Encoder pivotEncoder;
    public SetpointTest setpointtest = new SetpointTest("Pivot Non Trap", 200, 20.0);
    
    public Pivot(){
        super(new Controller(PIDConstants, Type.POSITION));
        configMotors();
        getInstance().setTolerance(PIVOT_TOLERANCE);
    }
    
    public static synchronized Pivot getInstance(){
        if(instance == null){
            instance = new Pivot();
        }
        return instance;
    }
    
    private void configMotors(){
        //check how to code this: 
        m_pivotMotor = new NAR_CANSparkMax(PIVOT_MOTOR_ID);
        m_pivotMotor.setCurrentLimit(PIVOT_CURRENT_LIMIT);
        m_pivotMotor.setInverted(false);
        m_pivotMotor.setUnitConversionFactor(360);
        m_pivotMotor.setNeutralMode(Neutral.COAST);
        
    }
    
    public void setPower(double power){
        disable();
        m_pivotMotor.set(power);
    }
    
    //no clue if this is correct
    public void resetPivot() {
        m_pivotMotor.resetPosition(ANGLE_OFFSET);;
    }
    
    public void stopPivot(){
        setPower(0);
    }
    
    public double getAngle(){
        return MathUtil.inputModulus(-pivotEncoder.get() * 360 - ANGLE_OFFSET,-180, 180);
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
    
    public CommandBase pivot(double setpoint){
        return runOnce(() -> startPID(setpoint));
    }
    
    
}
