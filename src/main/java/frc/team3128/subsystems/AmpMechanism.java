package frc.team3128.subsystems;

import common.core.controllers.TrapController;
import common.core.subsystems.PivotTemplate;
import common.hardware.motorcontroller.MotorControllerConstants;
import common.hardware.motorcontroller.NAR_CANSpark.SparkMaxConfig;
import common.hardware.motorcontroller.NAR_Motor.Neutral;
import common.utility.narwhaldashboard.NarwhalDashboard.State;
import common.utility.shuffleboard.NAR_Shuffleboard;
import common.utility.tester.CurrentTest;
import common.utility.tester.Tester.UnitTest;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.team3128.Constants.AmpWristConstants.*;
import static frc.team3128.Constants.IntakeConstants.CURRENT_LIMIT;

import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;

public class AmpMechanism extends PivotTemplate {

    public enum Setpoint {
        AMP(55),
        RETRACTED(-90);

        private double angle;
        private Setpoint(double angle) {
            this.angle = angle;
        }
    }

    private static AmpMechanism instance;

    public static synchronized AmpMechanism getInstance() {
        if (instance == null) {
            instance = new AmpMechanism();
        }
        return instance;
    }

    private AmpMechanism() {
        super(new TrapController(PIDConstants, TRAP_CONSTRAINTS), WRIST_MOTOR);
        setkG_Function(()-> Math.cos(Units.degreesToRadians(getSetpoint())));
        setTolerance(POSITION_TOLERANCE);
        setConstraints(-90, 90);
        initShuffleboard();
        NAR_Shuffleboard.addData(getName(), "Stall Current", ()-> ROLLER_MOTOR.getStallCurrent(), 4, 0);
    }

    @Override
    protected void configMotors() {
        WRIST_MOTOR.setUnitConversionFactor(GEAR_RATIO * 360);
        WRIST_MOTOR.setCurrentLimit(CURRENT_LIMIT);
        WRIST_MOTOR.setNeutralMode(Neutral.BRAKE);

        ROLLER_MOTOR.setNeutralMode(Neutral.COAST);

        WRIST_MOTOR.setStatusFrames(SparkMaxConfig.POSITION);
        ROLLER_MOTOR.setDefaultStatusFrames();
        ROLLER_MOTOR.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, MotorControllerConstants.HIGH_PRIORITY);

        WRIST_MOTOR.resetPosition(-90);
    }

    public Command pivotTo(Setpoint setpoint) {
        return pivotTo(setpoint.angle);
    }

    public Command retract() {
        return sequence(
            pivotTo(Setpoint.RETRACTED),
            runOnce(()-> ROLLER_MOTOR.set(0))
        );
    }

    public Command extend() {
        return sequence(
            pivotTo(Setpoint.AMP),
            runOnce(()-> ROLLER_MOTOR.set(AMP_POWER))
        );
    }

    public Command runRollers(double power) {
        return runOnce(()-> ROLLER_MOTOR.set(power));
    }

    public UnitTest getExtendTest() {
        return new SetpointTest("Extend Amp", Setpoint.AMP.angle, 0.02, EXTEND_TIMEOUT);
    }

    public UnitTest getRollerTest() {
        return new CurrentTest("Amp Roller", ROLLER_MOTOR, AMP_POWER, ROLLER_TIMEOUT, ROLLER_TEST_PLATEAU, ROLLER_TEST_EXPECTED_CURRENT, 5,  this);
    }

    public UnitTest getRetractTest() {
        return new SetpointTest("Retract Amp", Setpoint.RETRACTED.angle, 0.02, RETRACTED_TIMEOUT);
    }

    public State getRunningState() {
        return WRIST_MOTOR.getState();
    }
    
}
