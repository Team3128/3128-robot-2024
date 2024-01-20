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

    public NAR_CANSparkMax pivotMotor;

    private static Pivot instance;
    
    public static synchronized Pivot getInstance() {
        if (instance == null) {
            instance = new Pivot();
        }
        return instance;
    }

    public Pivot() {
        super(new TrapController(new PIDFFConfig(kP, kI, kD, kS, kV, kA, kG), new TrapezoidProfile.Constraints(maxVelocity, maxAccelerration)));
        setkG_Function(()-> Math.cos(Units.degreesToRadians(getSetpoint())));
        setConstraints(MIN_ANGLE, MAX_ANGLE);
        configMotor();
        getController().setTolerance(PIVOT_TOLERANCE);
    }

    public void startPID(Positions positions){
        startPID(positions.pivotANGLE);
    }

    private void configMotor() {
        pivotMotor = new NAR_CANSparkMax(PIVOT_MOTOR_ID);
        pivotMotor.setInverted(false);
        pivotMotor.setNeutralMode(Neutral.COAST);
        pivotMotor.setUnitConversionFactor(GEAR_RATIO * 360);
        resetEncoder();
    }

    public void resetEncoder() {
        pivotMotor.resetPosition(0);
    }

    public void set(double power) {
        disable();
        pivotMotor.set(power);
    }

    @Override
    protected void useOutput(double output, double setpoint) {
        final double percentOutput = output / 12.0;
        pivotMotor.set(MathUtil.clamp(percentOutput, -1, 1));
    }

    @Override
    protected double getMeasurement() {
        return pivotMotor.getPosition();
    }

    //Commands
    public Command setPivot(double power) {
        return runOnce(()-> set(power));
    }
    
    public Command pivotTo(Positions pivotPosition) {
        return runOnce(()-> startPID(pivotPosition));
    }

    public Command pivotTo(double pivotAngle) {
        return runOnce(()-> startPID(pivotAngle));
    }

    public Command resetPivot() {
        return runOnce(()-> resetEncoder());
    }
}