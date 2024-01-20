package frc.team3128.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import common.core.controllers.Controller.Type;
import common.core.controllers.Controller;
import common.core.controllers.PIDFFConfig;
import common.core.subsystems.NAR_PIDSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

import common.hardware.motorcontroller.NAR_CANSparkMax;
import common.hardware.motorcontroller.NAR_Motor.Neutral;

import static frc.team3128.Constants.WristConstants.*;
import frc.team3128.PositionConstants.Positions;

public class Wrist extends NAR_PIDSubsystem {

    public NAR_CANSparkMax wristMotor;

    private static Wrist instance;
    
    public static synchronized Wrist getInstance() {
        if (instance == null) {
            instance = new Wrist();
        }
        return instance;
    }

    public Wrist() {
        super(new Controller(new PIDFFConfig(kP,kI,kD,kS,kV,kG), Type.POSITION));
        setkG_Function(()-> Math.cos(Units.degreesToRadians(getSetpoint())));
        setConstraints(MIN_ANGLE,MAX_ANGLE);
        configMotor();
        getController().setTolerance(WRIST_TOLERANCE);
    }

    public void startPID(Positions positions){
        startPID(positions.wristANGLE);
    }

    private void configMotor() {
        wristMotor = new NAR_CANSparkMax(WRIST_MOTOR_ID);
        wristMotor.setInverted(false);
        wristMotor.setNeutralMode(Neutral.COAST);
        wristMotor.setUnitConversionFactor(GEAR_RATIO * 360);
        resetEncoder();
    }

    public void resetEncoder() {
        wristMotor.resetPosition(0);
    }

    public void set(double power) {
        disable();
        wristMotor.set(power);
    }

    @Override
    protected void useOutput(double output, double setpoint) {
        final double percentOutput = output / 12.0;
        wristMotor.set(MathUtil.clamp(percentOutput, -1, 1));
    }

    @Override
    protected double getMeasurement() {
        return wristMotor.getPosition();
    }

    //Commands
    public Command setWrist(double power) {
        return runOnce(()-> set(power));
    }
    
    public Command moveWrist(Positions pivotPosition) {
        return runOnce(()-> startPID(pivotPosition));
    }

    public Command moveWrist(double wristAngle) {
        return runOnce(()-> startPID(wristAngle));
    }
}