package frc.team3128.subsystems;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import static frc.team3128.Constants.ClimberConstants.*;

import common.core.controllers.TrapController;
import common.core.subsystems.NAR_PIDSubsystem;
import common.hardware.motorcontroller.NAR_CANSparkMax;
import common.hardware.motorcontroller.NAR_Motor.Neutral;

public class Climber extends NAR_PIDSubsystem {

    public enum State {
        EXTENDED(180),
        RETRACTED(0);

        public final double setpoint;
        private State(double setpoint) {
            this.setpoint = setpoint;
        }
    }
    
    private static Climber instance;
    
    private NAR_CANSparkMax leftMotor;
    private NAR_CANSparkMax rightMotor;
    
    public Climber() {
        super(new TrapController(PIDConstants, new Constraints(MAX_VELCOTIY, MAX_ACCELERATION)));
        configMotors();
        setTolerance(POSITION_TOLERANCE);
        setConstraints(POSITION_MINIMUM, POSITION_MAXIMUM);
    }
    
    public static synchronized Climber getInstance(){
        if (instance == null){
            instance = new Climber();
        }
        return instance;
    }
    
    private void configMotors() {
        leftMotor = new NAR_CANSparkMax(LEFT_MOTOR_ID);
        rightMotor = new NAR_CANSparkMax(RIGHT_MOTOR_ID);

        leftMotor.setInverted(false);
        rightMotor.follow(leftMotor, true);
        leftMotor.setUnitConversionFactor(GEAR_RATIO);
        
        leftMotor.setCurrentLimit(CURRENT_LIMIT);
        leftMotor.enableVoltageCompensation(12.0);

        leftMotor.setNeutralMode(Neutral.COAST);
        rightMotor.setNeutralMode(Neutral.COAST);
    }

    private void setPow(double power) {
        disable();
        leftMotor.set(power);
    }

    public void stop() {
        disable();
        leftMotor.set(0);
    }

    public void setVoltage(double voltage) {
        disable();
        leftMotor.setVolts(voltage);
    }

    @Override
    protected void useOutput(double output, double setpoint) {
        leftMotor.set(output);
    }

    public double getAngle(){
        return Math.atan(leftMotor.getPosition() / SHOOTER_PIVOT_DIST);
    }

    public void reset(){
        leftMotor.resetPosition(POSITION_MINIMUM);
    }

    public double interpolate(double dist){
        return 0;
    }

    @Override
    protected double getMeasurement() {
        return leftMotor.getPosition();
    }
    
    public Command moveTo(double setpoint){
        return runOnce(() -> startPID(setpoint));
    }

    public Command moveTo(State state) {
        return moveTo(state.setpoint);
    }

    public Command setPower(double power) {
        return runOnce(() -> setPow(power));
    }

    public Command setAngle(double angle){
        return moveTo(Math.tan(angle) * SHOOTER_PIVOT_DIST);
    }

    public Command shoot(double dist){
        return Commands.sequence(
            moveTo(dist),
            Commands.waitUntil(() -> atSetpoint())
        );
    }
}