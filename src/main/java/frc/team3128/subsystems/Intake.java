package frc.team3128.subsystems;

import common.core.controllers.TrapController;
import common.core.subsystems.NAR_PIDSubsystem;
import common.hardware.motorcontroller.NAR_CANSparkMax;
import common.hardware.motorcontroller.NAR_Motor.Neutral;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import static frc.team3128.Constants.IntakeConstants.*;

public class Intake extends NAR_PIDSubsystem{

    public enum State {
        EXTENDED(180, 0.5),
        AMP(75, -0.5),
        PICKUP(90, 0.5),
        RETRACTED(0, 1);

        public final double setpoint, power;
        private State(double setpoint, double power) {
            this.setpoint = setpoint;
            this.power = power;
        }
    }
    
    private static Intake instance;
    private NAR_CANSparkMax pivotMotor;
    private NAR_CANSparkMax intakeMotor;

    private boolean hasNote = false;
    
    public Intake(){
        super(new TrapController(PIDConstants, new Constraints(MAX_VELCOTIY, MAX_ACCELERATION)));
        configMotors();
        setTolerance(ANGLE_TOLERANCE);
        setConstraints(POSITION_MINIMUM, POSITION_MINIMUM);
    }
    
    public static synchronized Intake getInstance(){
        if(instance == null){
            instance = new Intake();
        }
        return instance;
    }
    
    private void configMotors(){
        pivotMotor = new NAR_CANSparkMax(PIVOT_MOTOR_ID);
        intakeMotor = new NAR_CANSparkMax(INTAKE_MOTOR_ID);
        intakeMotor.setCurrentLimit(CURRENT_LIMIT);
        pivotMotor.setInverted(false);
        intakeMotor.setInverted(false);
        pivotMotor.setUnitConversionFactor(360);
        pivotMotor.setNeutralMode(Neutral.COAST);
        intakeMotor.setNeutralMode(Neutral.BRAKE);
    }
    
    private void setPow(double power){
        disable();
        pivotMotor.set(power);
    }

    public void setVoltage(double voltage){
        pivotMotor.setVolts(voltage);
    }

    private void setIntakePower(double power){
        intakeMotor.set(power);
    }

    public boolean hasObjectPresent(){
        hasNote = intakeMotor.getStallCurrent() > STALL_CURRENT;
        return hasNote;
    }
    
    public void reset() {
        pivotMotor.resetPosition(POSITION_MINIMUM);
    }
    
    public void stop(){
        disable();
        setPow(0);
        setIntakePower(0);
    }
    
    public double getAngle(){
        return pivotMotor.getPosition();
    }

    @Override
    protected void useOutput(double output, double setpoint) {
        if(pivotMotor.getStallCurrent() > RETRACTION_CURRENT_THRESHOLD){
            disable();
        }
        setPow(output);
    }
    
    @Override
    protected double getMeasurement() {
        return pivotMotor.getPosition();
    }
    
    public Command moveTo(double setpoint){
        return runOnce(() -> startPID(setpoint));
    }

    public Command setPower(double power){
        return runOnce(() -> setPow(power));
    }

    public Command retract(){
        return Commands.sequence(
            moveTo(State.RETRACTED.setpoint),
            Commands.waitUntil(() -> atSetpoint()),
            runOnce(()-> setIntakePower(hasNote ? STALL_POWER : 0)),
            runOnce(()-> disable())
        );
    }
    
    public Command intake(){
        return Commands.sequence(
            moveTo(State.EXTENDED.setpoint),
            runOnce(()-> setIntakePower(State.EXTENDED.power)),
            Commands.waitSeconds(0.2),
            Commands.waitUntil(() -> hasObjectPresent()),
            runOnce(()-> setIntakePower(INTAKE_POWER)),
            retract()
        );
    }

    public Command intakeHP(){
        return Commands.sequence(
            moveTo(State.PICKUP.setpoint),
            runOnce(()-> setIntakePower(State.PICKUP.power)),
            Commands.waitSeconds(0.2),
            Commands.waitUntil(() -> hasObjectPresent()),
            retract()
        );
    }

    public Command outake(){
        return Commands.sequence(
            moveTo(State.AMP.setpoint),
            runOnce(() -> setIntakePower(State.AMP.power)),
            Commands.waitSeconds(0.2),
            Commands.waitUntil(() -> !hasObjectPresent()),
            retract()
        );
    }

    public Command shoot(){
        return Commands.sequence(
            runOnce(() -> setIntakePower(State.RETRACTED.power))
        );
    }
}