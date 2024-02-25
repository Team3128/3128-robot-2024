package frc.team3128.subsystems;

import common.core.controllers.TrapController;
import common.core.subsystems.ManipulatorTemplate;
import common.core.subsystems.PivotTemplate;
import common.hardware.motorcontroller.NAR_Motor.Neutral;
import common.utility.narwhaldashboard.NarwhalDashboard;
import common.utility.narwhaldashboard.NarwhalDashboard.State;
import common.utility.shuffleboard.NAR_Shuffleboard;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalSource;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.team3128.Constants.LedConstants.Colors;
import frc.team3128.commands.CmdManager;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.team3128.Constants.IntakeConstants.*;

public class Intake {

    public enum Setpoint {
        EXTENDED(-195),
        AMP(-90);

        public final double angle;
        private Setpoint(double angle) {
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
        }

        @Override
        public void periodic() {
            if (Math.abs(ROLLER_MOTOR.getStallCurrent()) > STALL_CURRENT) {
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
            runOnce(()-> Leds.getInstance().setLedColor(Colors.PIECE)),
            CmdManager.vibrateController(),
            runOnce(()-> isRetracting = true),
            waitUntil(()-> Climber.getInstance().isNeutral()),
            intakeRollers.runManipulator(shouldStall ? STALL_POWER : 0),
            intakePivot.pivotTo(-5),
            waitUntil(() -> intakePivot.atSetpoint()),
            intakePivot.runPivot(0.5),
            waitSeconds(0.1),
            intakePivot.runPivot(0),
            waitSeconds(0.25),
            runOnce(()-> isRetracting = false),
            intakePivot.reset(0),
            runOnce(()-> Leds.getInstance().setDefaultColor())
        );
    }
    
    public Command intake(Setpoint setpoint) {
        return sequence(
            runOnce(()-> isRetracting = true),
            intakeRollers.runManipulator(INTAKE_POWER),
            intakePivot.pivotTo(setpoint.angle),
            intakeRollers.intake(),
            retract(true)
        );
    }

    public Command intakeAuto() {
        return sequence(
            intakePivot.pivotTo(Setpoint.EXTENDED.angle),
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

    public State getRunningState() {
        if (ROLLER_MOTOR.getState() != State.DISCONNECTED && PIVOT_MOTOR.getState() != State.DISCONNECTED) {
            return State.RUNNING; 
        }
        if (ROLLER_MOTOR.getState() != State.DISCONNECTED || PIVOT_MOTOR.getState() != State.DISCONNECTED) {
            return State.PARTIALLY_RUNNING; 
        }
        return State.DISCONNECTED;
    }
}