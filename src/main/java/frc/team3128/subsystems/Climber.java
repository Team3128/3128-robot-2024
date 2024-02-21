package frc.team3128.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team3128.subsystems.random.Pose2dSupplier;

import static frc.team3128.Constants.ClimberConstants.*;

import java.util.function.DoubleSupplier;

import common.core.controllers.TrapController;
import common.core.controllers.Controller;
import common.core.controllers.Controller.Type;
import common.core.subsystems.NAR_PIDSubsystem;
import common.hardware.motorcontroller.NAR_CANSpark;
import common.hardware.motorcontroller.NAR_Motor.Neutral;
import common.utility.shuffleboard.NAR_Shuffleboard;

public class Climber extends NAR_PIDSubsystem {

    public enum State {
        EXTENDED(30),
        RETRACTED(0);

        public final double setpoint;
        private State(double setpoint) {
            this.setpoint = setpoint;
        }
    }
    
    private static Climber instance;
    
    private NAR_CANSpark leftMotor;
    private NAR_CANSpark rightMotor;
    
    public Pose2dSupplier location;
    
    private Climber() {
        super(new TrapController(PIDConstants, TRAP_CONSTRAINTS));
        configMotors();
        setTolerance(POSITION_TOLERANCE);
        setConstraints(POSITION_MINIMUM, POSITION_MAXIMUM);
        initShuffleboard();
        location = () -> Swerve.getInstance().getPose();
    }
    
    public static synchronized Climber getInstance(){
        if (instance == null){
            instance = new Climber();
        }
        return instance;
    }

    @Override
    public void initShuffleboard() {
        super.initShuffleboard();
        NAR_Shuffleboard.addData("Climber", "angle", ()-> getAngle(), 4, 0);
        NAR_Shuffleboard.addSendable("Commands", "Climber", this, 0, 1);
    }
    
    private void configMotors() {
        leftMotor = new NAR_CANSpark(LEFT_MOTOR_ID);
        rightMotor = new NAR_CANSpark(RIGHT_MOTOR_ID);

        leftMotor.setInverted(false);
        rightMotor.follow(leftMotor, true);
        leftMotor.setUnitConversionFactor(GEAR_RATIO * WHEEL_CIRCUMFERENCE * 100);

        leftMotor.setNeutralMode(Neutral.COAST);
        rightMotor.setNeutralMode(Neutral.COAST);
    }

    private void setPower(double power) {
        disable();
        leftMotor.set(power);
    }

    @Override
    protected void useOutput(double output, double setpoint) {
        if (Intake.getInstance().isRetracting) leftMotor.setVolts(0);
        leftMotor.setVolts(output);
    }

    public double getAngle(){
        return Units.radiansToDegrees(Math.atan2((getMeasurement() + HEIGHT_OFFSET), PIVOT_CLIMBER_DIST));
    }

    public double heightToAngle(double height){
        return Units.radiansToDegrees(Math.atan2((height + HEIGHT_OFFSET), PIVOT_CLIMBER_DIST));
    }

    public Command reset(){
        return runOnce(() -> leftMotor.resetPosition(POSITION_MINIMUM));
    }

    public double interpolate(double dist){
        return climberHeightMap.get(dist);
    }
    
    @Override
    public double getMeasurement() {
        return leftMotor.getPosition();
    }

    public Command climbTo(DoubleSupplier setpointSupplier) {
        return runOnce(()-> startPID(setpointSupplier.getAsDouble()));
    }
    
    public Command climbTo(double setpoint){
        return climbTo(()-> setpoint);
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