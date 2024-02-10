package frc.team3128.subsystems;

import common.core.controllers.Controller;
import common.core.controllers.TrapController;
import common.core.controllers.Controller.Type;
import common.core.subsystems.ManipulatorTemplate;
import common.core.subsystems.PivotTemplate;
import common.hardware.motorcontroller.NAR_Motor.Neutral;
import common.utility.shuffleboard.NAR_Shuffleboard;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.team3128.Constants.IntakeConstants.*;

public class Intake {

    public enum State {
        EXTENDED(-195),
        AMP(-89);

        public final double angle;
        private State(double angle) {
            this.angle = angle;
        }
    }

    private class IntakePivot extends PivotTemplate {
        
        private IntakePivot() {
            super(new TrapController(PIDConstants, TRAP_CONSTRAINTS), PIVOT_MOTOR);
            setkG_Function(()-> Math.cos(Units.degreesToRadians(getSetpoint())));
            setTolerance(ANGLE_TOLERANCE);
            setConstraints(-POSITION_MAXIMUM, POSITION_MINIMUM);
            setSafetyThresh(5);
            initShuffleboard();
        }

        @Override
        protected void configMotors() {
            PIVOT_MOTOR.setInverted(true);
            PIVOT_MOTOR.setUnitConversionFactor(360 * GEAR_RATIO);
            PIVOT_MOTOR.setNeutralMode(Neutral.COAST);
        }

        @Override
    public void initShuffleboard(){
        super.initShuffleboard();
        NAR_Shuffleboard.addSendable("Commands", "Intake Commands", this, 0, 3);
    }
    }

    private class IntakeRollers extends ManipulatorTemplate {

        private IntakeRollers() {
            super(STALL_CURRENT, INTAKE_POWER, OUTTAKE_POWER, STALL_POWER, 0.2, ROLLER_MOTOR);
            initShuffleboard();
        }

        @Override
        protected void configMotors() {
            ROLLER_MOTOR.setInverted(false);
            ROLLER_MOTOR.setNeutralMode(Neutral.BRAKE);
            ROLLER_MOTOR.setCurrentLimit(CURRENT_LIMIT);
        }

        private Command runRollersNoRequirements(double power) {
            return new InstantCommand(()-> setPower(power));
        }
    }
    
    private static Intake instance;

    private IntakePivot intakePivot;
    private IntakeRollers intakeRollers;

    public boolean isRetracting = false;

    public static synchronized Intake getInstance(){
        if(instance == null){
            instance = new Intake();
        }
        return instance;
    }

    private Intake(){
        intakePivot = new IntakePivot();
        intakeRollers = new IntakeRollers();
    }

    public Command retract(){
        return sequence(
            runOnce(()-> isRetracting = true),
            intakeRollers.runManipulator(STALL_POWER),
            intakePivot.pivotTo(0),
            waitUntil(() -> intakePivot.atSetpoint()),
            intakePivot.runPivot(0.5),
            waitSeconds(0.1),
            intakePivot.runPivot(0),
            waitSeconds(0.5),
            intakePivot.reset(0),
            runOnce(()-> isRetracting = false)
        );
    }
    
    public Command intake(State setpoint) {
        return sequence(
            runOnce(()-> isRetracting = true),
            intakePivot.pivotTo(setpoint.angle),
            intakeRollers.intake(),
            retract()
        );
    }

    public Command reset() {
        return intakePivot.reset(0);
    }

    public Command pivotTo(double setpoint) {
        return intakePivot.pivotTo(setpoint);
    }

    public Command runPivot(double power) {
        return intakePivot.runPivot(power);
    }

    public Command runRollers(double power) {
        return intakeRollers.runManipulator(power);
    }

    public Command outtakeNoRequirements() {
        return intakeRollers.runRollersNoRequirements(OUTTAKE_POWER);
    }

    public Command stopRollersNoRequirements() {
        return intakeRollers.runRollersNoRequirements(0);
    
    }

    public Command stopRollers() {
        return intakeRollers.runManipulator(0);
    }

    public Command outtake() {
        return intakeRollers.outtake();
    }
}