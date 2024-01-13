package frc.team3128.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import common.core.controllers.TrapController;

import common.core.controllers.PIDFFConfig;
import common.core.subsystems.NAR_PIDSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

import common.hardware.motorcontroller.NAR_CANSparkMax;
import common.hardware.motorcontroller.NAR_Motor.Neutral;

import static frc.team3128.Constants.WristConstants.*;
import frc.team3128.PositionConstants.Positions;

public class Wrist extends NAR_PIDSubsystem {
    public NAR_CANSparkMax m_wrist;
    private static Wrist instance;
    
    public static synchronized Wrist getInstance() {
        if (instance == null) {
            instance = new Wrist();
        }
        return instance;
    }
    


    public Wrist() {
        super(new TrapController(new PIDFFConfig(kP,kI,kD,kS,kV,kG), new TrapezoidProfile.Constraints(maxVelocity, maxAccelerration), 0.1));
        setkG_Function(()-> Math.cos(Units.degreesToRadians(getSetpoint())));
        setConstraints(MIN_ANGLE,MAX_ANGLE);
        configMotor();
        getController().setTolerance(WRIST_TOLERANCE);
    }

    public void startPID(Positions positions){
        startPID(positions.wristANGLE);
    }

    private void configMotor() {
        m_wrist = new NAR_CANSparkMax(WRIST_MOTOR_ID);
        m_wrist.setInverted(false);
        m_wrist.setNeutralMode(Neutral.COAST);
        m_wrist.setCurrentLimit(40);
        m_wrist.setUnitConversionFactor(GEAR_RATIO * 360);
        resetEncoder();
    }

    public void resetEncoder() {
        m_wrist.resetPosition(90*GEAR_RATIO/ROTATION_TO_DEGREES);
    }

    public void set(double power) {
        disable();
        m_wrist.set(power);
    }

    @Override
    protected void useOutput(double output, double setpoint) {
        double percentOutput = output / 12.0;
        m_wrist.set(MathUtil.clamp(percentOutput, -1, 1));
    }

    @Override
    protected double getMeasurement() {
        return m_wrist.getPosition();
    }

    //Commands
    public Command moveWrist(double power) {
        return runOnce(()-> set(power));
    }
    
    public Command angleWrist(Positions pivotPosition) {
        return runOnce(()-> startPID(pivotPosition.wristANGLE));
    }
}