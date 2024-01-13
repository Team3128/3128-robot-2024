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

import static frc.team3128.Constants.PivotConstants.*;
import frc.team3128.PositionConstants.Positions;

public class Pivot extends NAR_PIDSubsystem {
    public NAR_CANSparkMax m_pivot;
    private static Pivot instance;
    
    public static synchronized Pivot getInstance() {
        if (instance == null) {
            instance = new Pivot();
        }
        return instance;
    }
    


    public Pivot() {
        super(new TrapController(new PIDFFConfig(kP,kI,kD,kS,kV,kG), new TrapezoidProfile.Constraints(maxVelocity, maxAccelerration), 0.1));
        setkG_Function(()-> Math.cos(Units.degreesToRadians(getSetpoint())));
        setConstraints(MIN_ANGLE,MAX_ANGLE);
        configMotor();
        getController().setTolerance(PIVOT_TOLERANCE);
    }

    public void startPID(Positions positions){
        startPID(positions.pivotANGLE);
    }

    private void configMotor() {
        m_pivot = new NAR_CANSparkMax(PIVOT_MOTOR_ID);
        m_pivot.setInverted(false);
        m_pivot.setNeutralMode(Neutral.COAST);
        m_pivot.setCurrentLimit(40);
        m_pivot.setUnitConversionFactor(GEAR_RATIO * 360);
        resetEncoder();
    }

    public void resetEncoder() {
        m_pivot.resetPosition(90*GEAR_RATIO/ROTATION_TO_DEGREES);
    }

    public void set(double power) {
        disable();
        m_pivot.set(power);
    }

    @Override
    protected void useOutput(double output, double setpoint) {
        double percentOutput = output / 12.0;
        m_pivot.set(MathUtil.clamp(percentOutput, -1, 1));
    }

    @Override
    protected double getMeasurement() {
        return m_pivot.getPosition();
    }

    //Commands
    public Command movePivot(double power) {
        return runOnce(()-> set(power));
    }
    
    public Command anglePivot(Positions pivotPosition) {
        return runOnce(()-> startPID(pivotPosition.pivotANGLE));
    }
}