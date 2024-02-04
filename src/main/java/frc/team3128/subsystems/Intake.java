package frc.team3128.subsystems;

import common.core.controllers.Controller;
import common.core.controllers.PIDFFConfig;
import common.core.controllers.TrapController;
import common.core.controllers.Controller.Type;
import common.core.subsystems.NAR_PIDSubsystem;
import common.hardware.motorcontroller.NAR_CANSparkMax;
import common.hardware.motorcontroller.NAR_Motor.Neutral;
import common.utility.shuffleboard.NAR_Shuffleboard;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.team3128.Constants.IntakeConstants.*;

public class Intake extends NAR_PIDSubsystem{

    public enum State {
        EXTENDED(-200, 0.5),
        PICKUP_HP(90, 0.5);

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
        setConstraints(-POSITION_MAXIMUM, POSITION_MINIMUM);
        setSafetyThresh(5);
        initShuffleboard();
    }
    
    public static synchronized Intake getInstance(){
        if(instance == null){
            instance = new Intake();
        }
        return instance;
    }

    @Override
    public void initShuffleboard() {
        super.initShuffleboard();
        NAR_Shuffleboard.addData("Intake", "Current", ()-> rollerMotor.getStallCurrent(), 4, 0);
        NAR_Shuffleboard.addData("Intake", "Object", ()-> hasObjectPresent(), 4, 1);
    }
    
    private void configMotors(){
        pivotMotor = new NAR_CANSparkMax(PIVOT_MOTOR_ID);
        rollerMotor = new NAR_CANSparkMax(INTAKE_MOTOR_ID);
        rollerMotor.setCurrentLimit(CURRENT_LIMIT);
        pivotMotor.setInverted(true);
        rollerMotor.setInverted(false);
        pivotMotor.setUnitConversionFactor(360 * GEAR_RATIO);
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
        return Math.abs(rollerMotor.getStallCurrent()) > STALL_CURRENT;
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
            pivotTo(0),
            waitUntil(() -> atSetpoint()),
            runOnce(()-> disable())
        );
    }
    
    public Command intake(State setpoint) {
        return sequence(
            pivotTo(setpoint),
            setRoller(setpoint),
            waitSeconds(0.2),
            waitUntil(() -> hasObjectPresent()),
            waitSeconds(0.2),
            retract()
        );
    }

    public Command outtake(){
        return setRoller(OUTTAKE_POWER);
    }
}