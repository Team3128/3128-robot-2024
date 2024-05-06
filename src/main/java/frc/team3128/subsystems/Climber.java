package frc.team3128.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import static frc.team3128.Constants.ClimberConstants.*;
import static frc.team3128.Constants.ClimberConstants.SETPOINT_TEST_TIMEOUT_EXTEND;
import static frc.team3128.Constants.ClimberConstants.SETPOINT_TEST_TIMEOUT_RETRACT;

import java.util.function.DoubleSupplier;

import common.core.controllers.TrapController;
import common.core.subsystems.NAR_PIDSubsystem;
import common.hardware.motorcontroller.NAR_CANSpark;
import common.hardware.motorcontroller.NAR_CANSpark.SparkMaxConfig;
import common.hardware.motorcontroller.NAR_Motor.Neutral;
import common.utility.narwhaldashboard.NarwhalDashboard.State;
import common.utility.shuffleboard.NAR_Shuffleboard;
import common.utility.tester.Tester;
import common.utility.tester.Tester.UnitTest;

public class Climber extends NAR_PIDSubsystem {

    public enum Setpoint {
        EXTENDED(30),
        RAMSHOT(24.5),
        AMP(20.4),
        RETRACTED(0);

        public final double setpoint;
        private Setpoint(double setpoint) {
            this.setpoint = setpoint;
        }
    }
    
    private static Climber instance;
    
    private NAR_CANSpark leftMotor;
    private NAR_CANSpark rightMotor;

    private DoubleSupplier x;
    
    private Climber() {
        super(new TrapController(PIDConstants, TRAP_CONSTRAINTS));
        configMotors();
        setTolerance(POSITION_TOLERANCE);
        setConstraints(POSITION_MINIMUM, POSITION_MAXIMUM);
        initShuffleboard();
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
        NAR_Shuffleboard.addSendable("Commands", "Shooter", Shooter.getInstance(), 0, 2);
        NAR_Shuffleboard.addSendable("Commands", "IntakePivot", Intake.getInstance().intakePivot, 0, 3);
        NAR_Shuffleboard.addSendable("Commands", "IntakeRollers", Intake.getInstance().intakeRollers, 0, 4);
        NAR_Shuffleboard.addSendable("Commands", "CommandScheduler", CommandScheduler.getInstance(), 3, 0);
        x = NAR_Shuffleboard.debug("Shooter Function", "Concavity", -1.1, 0, 0);

    }
    
    private void configMotors() {
        leftMotor = new NAR_CANSpark(LEFT_MOTOR_ID);
        rightMotor = new NAR_CANSpark(RIGHT_MOTOR_ID);

        leftMotor.setInverted(false);
        rightMotor.setInverted(true);
        leftMotor.setUnitConversionFactor(GEAR_RATIO * WHEEL_CIRCUMFERENCE * 100);

        leftMotor.setNeutralMode(Neutral.BRAKE);
        rightMotor.setNeutralMode(Neutral.BRAKE);

        leftMotor.setStatusFrames(SparkMaxConfig.POSITION);
        rightMotor.setStatusFrames(SparkMaxConfig.POSITION);
    }

    private void setPower(double power) {
        disable();
        leftMotor.set(power);
        rightMotor.set(power);
    }

    public Command toggleBrake(boolean isBrake) {
        return runOnce(()-> {
            leftMotor.setNeutralMode(isBrake ? Neutral.BRAKE : Neutral.COAST);
            rightMotor.setNeutralMode(isBrake ? Neutral.BRAKE : Neutral.COAST);
        });
    }

    @Override
    protected void useOutput(double output, double setpoint) {
        if (Intake.getInstance().isRetracting && isNeutral()) {
            leftMotor.setVolts(0);
            rightMotor.setVolts(0);
            return;
        }
        leftMotor.setVolts(output);
        rightMotor.setVolts(output);
    }

    public double getAngle(){
        return (Units.radiansToDegrees(Math.atan2((getMeasurement() + HEIGHT_OFFSET), PIVOT_CLIMBER_DIST)) - ANGLE_OFFSET);
    }

    public double heightToAngle(double height){
        return Units.radiansToDegrees(Math.atan2((height + HEIGHT_OFFSET), PIVOT_CLIMBER_DIST));
    }

    public Command reset(){
        return runOnce(() -> leftMotor.resetPosition(POSITION_MINIMUM));
    }

    public double interpolate(double dist){
        if(dist < 1.15) return 25;
        return 27.4 * Math.pow(dist, x.getAsDouble());
        // return 28.5 - 0.233 * dist + 9.26 * Math.pow(10, -4) * Math.pow(dist, 2) - 1.33 * Math.pow(10, -6) * Math.pow(dist, 3) + 93;
        // return 25 * Math.pow(dist + 0.07, x.getAsDouble());
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
        return getMeasurement() < -5.0;
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

    public UnitTest getClimberTestExtend() {
        return new SetpointTest
        (
            "testClimber",
            Setpoint.EXTENDED.setpoint,
            0.02,
            SETPOINT_TEST_TIMEOUT_EXTEND
        );
    }

    public UnitTest getClimberTestRetract() {
        return new SetpointTest
        (
            "testClimber",
            0,
            0.02,
            SETPOINT_TEST_TIMEOUT_RETRACT
        );
    }

    public void addClimberTests() {
        Tester.getInstance().addTest("Climber", getClimberTestExtend());
        Tester.getInstance().addTest("Climber", AmpMechanism.getInstance().getExtendTest());
        Tester.getInstance().addTest("Climber", AmpMechanism.getInstance().getRollerTest());
        Tester.getInstance().addTest("Climber", AmpMechanism.getInstance().getRetractTest());
        Tester.getInstance().addTest("Climber", getClimberTestRetract());
        Tester.getInstance().getTest("Climber").setTimeBetweenTests(1);
    }
}