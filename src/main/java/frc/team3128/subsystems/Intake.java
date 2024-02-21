package frc.team3128.subsystems;

import common.core.controllers.TrapController;
import common.core.subsystems.ManipulatorTemplate;
import common.core.subsystems.PivotTemplate;
import common.hardware.motorcontroller.NAR_Motor.Neutral;
import common.utility.narwhaldashboard.NarwhalDashboard;
import common.utility.shuffleboard.NAR_Shuffleboard;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalSource;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.team3128.commands.CmdManager;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.team3128.Constants.IntakeConstants.*;

public class Intake {

    public enum State {
        EXTENDED(-195),
        AMP(-90);

        public final double angle;
        private State(double angle) {
            this.angle = angle;
        }
    }

    public class IntakePivot extends PivotTemplate {
        
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
    }

    public class IntakeRollers extends ManipulatorTemplate {
        private DigitalInput limitSwitch;

        private IntakeRollers() {
            super(STALL_CURRENT, INTAKE_POWER, OUTTAKE_POWER, STALL_POWER, 0.3, ROLLER_MOTOR);
            limitSwitch = new DigitalInput(9);
            initShuffleboard();
            NAR_Shuffleboard.addData(getSubsystem(), "Current", ()-> ROLLER_MOTOR.getStallCurrent());
        }

        @Override
        public void periodic() {
            if (ROLLER_MOTOR.getStallCurrent() > STALL_CURRENT) {
                runManipulator(0).schedule();
            }
        }

        @Override
        protected void configMotors() {
            ROLLER_MOTOR.setInverted(false);
            ROLLER_MOTOR.setNeutralMode(Neutral.BRAKE);
            ROLLER_MOTOR.setCurrentLimit(CURRENT_LIMIT);
        }

        public Command runNoRequirements(double power) {
            return new InstantCommand(()-> setPower(power));
        }

        public Command outtakeNoRequirements() {
            return runNoRequirements(OUTTAKE_POWER);
        }

        public Command intakeNoRequirements() {
            return runNoRequirements(INTAKE_POWER);
        }

        @Override
        public boolean hasObjectPresent() {
            return !limitSwitch.get();
        }
    }
    
    private static Intake instance;

    public IntakePivot intakePivot;
    public IntakeRollers intakeRollers;

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
        NAR_Shuffleboard.addData("IsRetracting", "Boolean", ()-> isRetracting,0, 0);
        NarwhalDashboard.getInstance().checkState("Intake", ()-> getRunningState());
    }

    public Command retract(boolean shouldStall) {
        return sequence(
            CmdManager.vibrateController(),
            runOnce(()-> isRetracting = true),
            waitUntil(()-> Climber.getInstance().isNeutral()),
            intakeRollers.runManipulator(shouldStall ? STALL_POWER : 0),
            intakePivot.pivotTo(0),
            waitUntil(() -> intakePivot.atSetpoint()),
            intakePivot.runPivot(0.5),
            waitSeconds(0.1),
            intakePivot.runPivot(0),
            waitSeconds(0.25),
            runOnce(()-> isRetracting = false),
            intakePivot.reset(0)
        );
    }
    
    public Command intake(State setpoint) {
        return sequence(
            runOnce(()-> isRetracting = true),
            intakePivot.pivotTo(setpoint.angle),
            intakeRollers.intake(),
            retract(true)
        );
    }

    public Command intakeAuto() {
        return sequence(
            intakePivot.pivotTo(State.EXTENDED.angle),
            intakeRollers.intake(),
            retractAuto()
        );
    }

    public Command retractAuto() {
        return sequence(
            waitUntil(()-> Climber.getInstance().isNeutral()),
            intakeRollers.runManipulator(STALL_POWER),
            intakePivot.pivotTo(0),
            waitUntil(() -> intakePivot.atSetpoint()),
            intakePivot.runPivot(0)
        );
    }

    public NarwhalDashboard.State getRunningState() {
        if (ROLLER_MOTOR.getTemperature() != 0 && PIVOT_MOTOR.getTemperature() != 0) {
            return NarwhalDashboard.State.RUNNING; 
        }
        if (ROLLER_MOTOR.getTemperature() != 0 || PIVOT_MOTOR.getTemperature() != 0) {
            return NarwhalDashboard.State.PARTIALLY_RUNNING; 
        }
        return NarwhalDashboard.State.DISCONNECTED;
    }
}