package frc.team3128.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

import static frc.team3128.Constants.ClimberConstants.*;

import common.core.controllers.TrapController;
import common.core.subsystems.NAR_PIDSubsystem;
import common.hardware.motorcontroller.NAR_CANSparkMax;
import common.hardware.motorcontroller.NAR_Motor.Neutral;

public class Climber extends NAR_PIDSubsystem {

    public enum State {
        EXTENDED(180),
        RETRACTED(0.05);

        public final double setpoint;
        private State(double setpoint) {
            this.setpoint = setpoint;
        }
    }
    
    private static Climber instance;
    
    private NAR_CANSparkMax leftMotor;
    //private NAR_CANSparkMax rightMotor;
    
    private Climber() {
        super(new TrapController(PIDConstants, TRAP_CONSTRAINTS));
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
        //rightMotor = new NAR_CANSparkMax(RIGHT_MOTOR_ID);

        leftMotor.setInverted(false);
        //rightMotor.follow(leftMotor, true);
        leftMotor.setUnitConversionFactor(GEAR_RATIO);

        leftMotor.setNeutralMode(Neutral.COAST);
        //rightMotor.setNeutralMode(Neutral.COAST);
    }

    private void setPower(double power) {
        disable();
        leftMotor.set(power);
    }

    @Override
    protected void useOutput(double output, double setpoint) {
        leftMotor.setVolts(output);
    }

    public double getAngle(){
        return Units.radiansToDegrees(Math.atan2((getMeasurement() + HEIGHT_OFFSET), PIVOT_CLIMBER_DIST));
    }

    public Command reset(){
        return runOnce(() -> leftMotor.resetPosition(POSITION_MINIMUM - 0.05));
    }

    public double interpolate(double dist){
        return 0;
    }

    @Override
    public double getMeasurement() {
        return leftMotor.getPosition();
    }
    
    public Command climbTo(double setpoint){
        return runOnce(() -> startPID(setpoint));
    }

    public Command climbTo(State state) {
        return climbTo(state.setpoint);
    }

    public Command setClimber(double power) {
        return runOnce(() -> setPower(power));
    }


    public Command setAngle(double angle){
        return climbTo(Math.tan(Units.degreesToRadians(angle)) * PIVOT_CLIMBER_DIST - HEIGHT_OFFSET);
    }
}