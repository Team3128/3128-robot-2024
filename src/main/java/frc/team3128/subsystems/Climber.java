package frc.team3128.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

import static frc.team3128.Constants.ClimberConstants.*;
import static frc.team3128.Constants.IntakeConstants.SETPOINT_TEST_PLATEAU;
import static frc.team3128.Constants.IntakeConstants.SETPOINT_TEST_TIMEOUT;

import java.util.function.DoubleSupplier;

import common.core.controllers.TrapController;
import common.core.subsystems.NAR_PIDSubsystem;
import common.hardware.motorcontroller.NAR_CANSpark;
import common.hardware.motorcontroller.NAR_Motor.Neutral;
import common.utility.narwhaldashboard.NarwhalDashboard;
import common.utility.narwhaldashboard.NarwhalDashboard.State;
import common.utility.shuffleboard.NAR_Shuffleboard;
import common.utility.tester.Tester;
import common.utility.tester.Tester.UnitTest;

public class Climber extends NAR_PIDSubsystem {

    public enum Setpoint {
        EXTENDED(30),
        AMP(18),
        RETRACTED(0);

        public final double setpoint;
        private Setpoint(double setpoint) {
            this.setpoint = setpoint;
        }
    }
    
    private static Climber instance;
    
    private NAR_CANSpark leftMotor;
    private NAR_CANSpark rightMotor;
    
    private Climber() {
        super(new TrapController(PIDConstants, TRAP_CONSTRAINTS));
        configMotors();
        setTolerance(POSITION_TOLERANCE);
        setConstraints(POSITION_MINIMUM, POSITION_MAXIMUM);
        initShuffleboard();
        NarwhalDashboard.getInstance().checkState(getName(), ()-> getRunningState());
        addClimberTests();
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
        if (Intake.getInstance().isRetracting && isNeutral()) {
            leftMotor.setVolts(0);
            return;
        }
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
        return 25 * Math.pow(dist, -0.911);
        // return 45.1 - 27 * dist + 6.86 * Math.pow(dist, 2) - 0.621 * Math.pow(dist, 3);
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

    public Command climbTo(Setpoint state) {
        return climbTo(state.setpoint);
    }

    public Command setClimber(double power) {
        return runOnce(() -> setPower(power));
    }


    public Command setAngle(double angle){
        return climbTo(Math.tan(Units.degreesToRadians(angle)) * PIVOT_CLIMBER_DIST - HEIGHT_OFFSET);
    }

    public boolean isNeutral() {
        return getMeasurement() < NEUTRAL_THRESHOLD;
    }

    public boolean isClimbed() {
        return getMeasurement() < -1.0;
    }

    public State getRunningState() {
        if (rightMotor.getState() != State.DISCONNECTED && leftMotor.getState() != State.DISCONNECTED) {
            return State.RUNNING; 
        }
        if (rightMotor.getState() != State.DISCONNECTED || leftMotor.getState() != State.DISCONNECTED) {
            return State.PARTIALLY_RUNNING; 
        }
        return State.DISCONNECTED;
    }

    public UnitTest getClimberTest() {
        return new UnitTest
        (
            "testClimber",
            climbTo(Setpoint.EXTENDED).withTimeout(CLIMBER_TEST_TIMEOUT)
        );
    }

    public void addClimberTests() {
        Tester.getInstance().addTest("Climber", getClimberTest());
    }
}