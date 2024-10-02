package frc.team3128.subsystems;

import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static frc.team3128.Constants.AmpConstants.*;
import static frc.team3128.Constants.FieldConstants.*;


import common.core.subsystems.PivotTemplate;
import common.utility.shuffleboard.NAR_Shuffleboard;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Amp extends PivotTemplate{

    private static Amp instance;

    public static synchronized Amp getInstance() {
        if (instance == null) {
            instance = new Amp();
        }
        return instance;
    }

    public Amp() {
        super(PIVOT_CONTROLLER, PIVOT_MOTOR);
        configController();
        initTriggers();
        initShuffleboard();
    }

    private void configController() {
        setkG_Function(()-> Math.cos(Units.degreesToRadians(getSetpoint())));
        setTolerance(POSITION_TOLERANCE);
        setConstraints(POSITION_MINIMUM, POSITION_MAXIMUM);
    }

    @Override
    protected void configMotors() {
        PIVOT_MOTOR.setUnitConversionFactor(PIVOT_GEAR_RATIO);
        PIVOT_MOTOR.setCurrentLimit(PIVOT_CURRENT_LIMIT);
        PIVOT_MOTOR.setNeutralMode(PIVOT_NEUTRAL_MODE);
        PIVOT_MOTOR.setInverted(PIVOT_INVERTED);
        PIVOT_MOTOR.setStatusFrames(PIVOT_STATUS_FRAME);

        ROLLER_MOTOR.setUnitConversionFactor(ROLLER_GEAR_RATIO);
        ROLLER_MOTOR.setCurrentLimit(ROLLER_CURRENT_LIMIT);
        ROLLER_MOTOR.setNeutralMode(ROLLER_NEUTRAL_MODE);
        ROLLER_MOTOR.setInverted(ROLLER_INVERTED);
        ROLLER_MOTOR.setDefaultStatusFrames();
    }

    private void initTriggers() {

        // Extend Amp
        new Trigger(()-> goalStateIs(AmpState.RETRACTED))                                                                                   // IF Current State is Retracted
                    .and(()-> Intake.getInstance().intakeRollers.hasObjectPresent())                                                        // AND Intake does have object
                    .and(()-> getDist(Swerve.getInstance().getPose().getTranslation(), allianceFlip(AMP)) < AMP_DISTANCE_THRESHOLD)         // AND Distance to Amp is less than 0.5m
                    .and(()-> Math.abs(Swerve.getInstance().getYaw() - 90) < AMP_ANGLE_THRESHOLD)                                           // AND Robot is facing Amp direction
                    .onTrue(setState(AmpState.AMP));                                                                                        // THEN Set States Extended

        // Retract Amp
    new Trigger(()-> goalStateIs(AmpState.AMP))                                                                                             // IF Current State is Extended
                    .and(()-> Intake.getInstance().intakeRollers.hasObjectPresent())                                                        // AND Intake does not have object
                    .and(()-> ROLLER_MOTOR.getStallCurrent() > ROLLER_STALL_CURRENT)                                                        // AND Rollers have current spike
                    .onTrue(setState(AmpState.RETRACTED));                                                                                  // THEN Set States Retracted
    }

    @Override
    public  void initShuffleboard() {
        super.initShuffleboard();
        NAR_Shuffleboard.addData("Amp Mechanism", "Stall Current", ()-> ROLLER_MOTOR.getStallCurrent(), 4, 0);
    }

    public Command pivotTo(AmpState setpoint) {
        return super.pivotTo(setpoint.angle);
    }

    public Command runRollers(double speed) {
        return runOnce(()-> ROLLER_MOTOR.set(speed));
    }

    public Command setState(AmpState setpoint){
        return sequence(
            pivotTo(setpoint.angle),
            runRollers(setpoint.rollerPower)
        );
    }

    public boolean goalStateIs(AmpState setpoint) {
        return getSetpoint() == setpoint.angle && ROLLER_MOTOR.getAppliedOutput() == setpoint.rollerPower;
    }
}
