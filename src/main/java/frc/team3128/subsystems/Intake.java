package frc.team3128.subsystems;

import common.core.controllers.TrapController;
import common.core.subsystems.NAR_PIDSubsystem;
import common.hardware.motorcontroller.NAR_CANSparkMax;
import common.hardware.motorcontroller.NAR_Motor.Neutral;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.team3128.Constants.IntakeConstants.*;

public class Intake extends NAR_PIDSubsystem{

    public enum State {
        EXTENDED(180, 0.9),
        PICKUP_HP(90, 0.9);

        public final double setpoint, power;
        private State(double setpoint, double power) {
            this.setpoint = setpoint;
            this.power = power;
        }
    }
    
    private static Intake instance;
    private NAR_CANSparkMax pivotMotor;
    private NAR_CANSparkMax rollerMotor;

    private Intake(){
        super(new TrapController(PIDConstants, TRAP_CONSTRAINTS));
        setkG_Function(()-> Math.cos(Units.degreesToRadians(getSetpoint())));
        configMotors();
        setTolerance(ANGLE_TOLERANCE);
        setConstraints(POSITION_MINIMUM, POSITION_MINIMUM);
        setSafetyThresh(1);
    }
    
    public static synchronized Intake getInstance(){
        if(instance == null){
            instance = new Intake();
        }
        return instance;
    }
    
    private void configMotors(){
        pivotMotor = new NAR_CANSparkMax(PIVOT_MOTOR_ID);
        rollerMotor = new NAR_CANSparkMax(INTAKE_MOTOR_ID);
        rollerMotor.setCurrentLimit(CURRENT_LIMIT);
        pivotMotor.setInverted(false);
        rollerMotor.setInverted(false);
        pivotMotor.setUnitConversionFactor(360);
        pivotMotor.setNeutralMode(Neutral.COAST);
        rollerMotor.setNeutralMode(Neutral.BRAKE);
    }
    
    private void setPivotPower(double power){
        disable();
        pivotMotor.set(power);
    }

    private void setRollerPower(double power){
        rollerMotor.set(power);
    }

    public boolean hasObjectPresent(){
        return rollerMotor.getStallCurrent() > STALL_CURRENT;
    }
    
    public Command reset() {
        return runOnce(()-> pivotMotor.resetPosition(POSITION_MINIMUM));
    }

    @Override
    protected void useOutput(double output, double setpoint) {
        pivotMotor.setVolts(output);
    }
    
    @Override
    public double getMeasurement() {
        return pivotMotor.getPosition();
    }
    
    public Command pivotTo(double setpoint){
        return runOnce(() -> startPID(setpoint));
    }

    public Command pivotTo(State setpoint){
        return runOnce(() -> startPID(setpoint.setpoint));
    }

    public Command setPivot(double power){
        return runOnce(() -> setPivotPower(power));
    }

    public Command setRoller(double power){
        return runOnce(() -> setRollerPower(power));
    }

    public Command setRoller(State setpoint){
        return runOnce(() -> setRollerPower(setpoint.power));
    }

    public Command retract(){
        return sequence(
            setRoller(STALL_POWER),
            pivotTo(Climber.getInstance().getAngle()),
            waitUntil(() -> atSetpoint()),
            runOnce(()-> disable())
        );
    }
    
    public Command intake(State setpoint) {
        return sequence(
            pivotTo(setpoint),
            runOnce(()-> setRoller(setpoint)),
            waitSeconds(0.2),
            waitUntil(() -> hasObjectPresent()),
            retract()
        );
    }

    public Command outtake(){
        return setRoller(OUTTAKE_POWER);
    }
}