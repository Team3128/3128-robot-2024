package frc.team3128.subsystems;

import common.core.controllers.TrapController;
import common.core.subsystems.ManipulatorTemplate;
import common.core.subsystems.PivotTemplate;
import common.hardware.motorcontroller.NAR_CANSpark.SparkMaxConfig;
import common.hardware.motorcontroller.NAR_Motor.Neutral;
import common.utility.narwhaldashboard.NarwhalDashboard.State;
import common.utility.tester.CurrentTest;
import common.utility.tester.Tester;
import common.utility.tester.Tester.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.team3128.Constants.IntakeConstants.*;
import java.util.function.DoubleSupplier;

public class Intake {

    public enum Setpoint {
        EXTENDED(200),
        SOURCE(60),
        AMP(90);
        

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
            setConstraints(POSITION_MINIMUM, POSITION_MAXIMUM);
            setSafetyThresh(5);
            initShuffleboard();
        }

        @Override
        protected void configMotors() {
            PIVOT_MOTOR.setInverted(true);
            PIVOT_MOTOR.setUnitConversionFactor(360 * GEAR_RATIO);
            PIVOT_MOTOR.setNeutralMode(Neutral.COAST);
            PIVOT_MOTOR.setStatusFrames(SparkMaxConfig.POSITION);
        }

        @Override
        public void useOutput(double output, double setpoint) {
            PIVOT_MOTOR.setVolts(MathUtil.clamp(output, -12, 12));
        }

        public Command hardReset(double power) {
            return sequence(
                waitUntil(()-> intakePivot.atSetpoint()).withTimeout(1.5),
                intakePivot.runPivot(power),
                waitSeconds(0.1),
                intakePivot.runPivot(0),
                intakePivot.reset(0)
            );
        }

        public Command pivotTo(DoubleSupplier setpoint) {
            return runOnce(()-> startPID(setpoint.getAsDouble()));
        }

        public SetpointTest getPivotTest() {
            return new SetpointTest
            (
                "testIntakePivot",
                Setpoint.EXTENDED.angle,
                SETPOINT_TEST_PLATEAU,
                SETPOINT_TEST_TIMEOUT
            );
        }
    }

    public class IntakeRollers extends ManipulatorTemplate {
        private DigitalInput limitSwitch;

        private IntakeRollers() {
            super(STALL_CURRENT, INTAKE_POWER, OUTTAKE_POWER, STALL_POWER, 0.3, RIGHT_ROLLER_MOTOR);
            limitSwitch = new DigitalInput(4);
            initShuffleboard();
        }

        @Override
        protected void configMotors() {
            RIGHT_ROLLER_MOTOR.setInverted(false);
            RIGHT_ROLLER_MOTOR.enableVoltageCompensation(9);
            RIGHT_ROLLER_MOTOR.setNeutralMode(Neutral.COAST);
            RIGHT_ROLLER_MOTOR.setCurrentLimit(CURRENT_LIMIT);
            RIGHT_ROLLER_MOTOR.setStatusFrames(SparkMaxConfig.VELOCITY);
        }

        public Command runNoRequirements(double power) {
            return new InstantCommand(()-> setPower(power));
        }

        public Command outtakeWithTimeout(double timeout) {
            return sequence(
                runNoRequirements(OUTTAKE_POWER),
                waitSeconds(timeout),
                runNoRequirements(0)
            );
        }

        public Command ampOuttake(double timeout) {
            return sequence(
                runNoRequirements(-0.3),
                waitSeconds(timeout),
                runNoRequirements(0)
            );
        }

        public Command serialize() {
            return sequence(
                runManipulator(-0.1),
                waitUntil(()-> !hasObjectPresent()),
                runManipulator(0.1),
                waitUntil(()-> hasObjectPresent()),
                runManipulator(0)
            );
        }

        @Override
        public boolean hasObjectPresent() {
            return !limitSwitch.get();
        }

        public CurrentTest getRollersTest() {
            return new CurrentTest
            (
                "testRollers", 
                RIGHT_ROLLER_MOTOR, 
                CURRENT_TEST_POWER, 
                CURRENT_TEST_TIMEOUT,
                CURRENT_TEST_PLATEAU,
                CURRENT_TEST_EXPECTED_CURRENT,
                CURRENT_TEST_TOLERANCE, 
                this
            );
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
    }

    public Command retract(boolean serialize) {
        return sequence(
            waitUntil(()-> Climber.getInstance().isNeutral()),
            runOnce(()-> isRetracting = true),
            intakePivot.pivotTo(5),
            intakePivot.hardReset(-0.2),
            runOnce(()-> isRetracting = false),
            either(intakeRollers.serialize().withTimeout(1).andThen(intakeRollers.runManipulator(0)), intakeRollers.runManipulator(0), ()-> serialize)
        );
    }
    
    public Command intake(Setpoint setpoint) {
        return sequence(
            deadline(
                intakeRollers.intake(),
                sequence(
                    intakePivot.pivotTo(setpoint.angle)
                    // either(
                    //     intakePivot.stallIntakePivot(0.1),
                    //     none(),
                    //     ()-> setpoint == Setpoint.EXTENDED
                    // )
                )
            ),
            retract(true)
        );
    }

    public Command outtake() {
        return sequence (
            intakePivot.pivotTo(Setpoint.EXTENDED.angle),
            waitUntil(()-> intakePivot.getMeasurement() > 45).withTimeout(2),
            intakeRollers.runManipulator(-1),
            waitSeconds(0.5),
            retract(false)
        );
    }

    public Command intakeAuto() {
        return sequence(
            deadline(
                intakeRollers.intake(),
                sequence(
                    intakePivot.pivotTo(Setpoint.EXTENDED.angle)
                    // intakePivot.stallIntakePivot(0.1)
                )
            ),
            retractAuto()
        );
    }

    public Command retractAuto() {
        return sequence(
            intakeRollers.runManipulator(STALL_POWER),
            intakePivot.pivotTo(()-> Climber.getInstance().getAngle()),
            waitUntil(() -> intakePivot.atSetpoint()).withTimeout(1),
            intakePivot.runPivot(0)
        );
    }

    public State getRunningState() {
        if (PIVOT_MOTOR.getState() == State.DISCONNECTED) return State.DISCONNECTED;
        if (RIGHT_ROLLER_MOTOR.getState() == State.DISCONNECTED) return State.DISCONNECTED;
        return State.RUNNING;
    }

    // public UnitTest getIntakeTest() {
    //     return new UnitTest
    //     (
    //         "testIntakeExtend",
    //         intake(Setpoint.EXTENDED).withTimeout(INTAKE_TEST_TIMEOUT)
    //     );
    // }

    // public void addIntakeTests() {
    //     Tester.getInstance().addTest("Intake", intakeRollers.getRollersTest());
    //     Tester.getInstance().addTest("Intake", intakePivot.getPivotTest());
    //     Tester.getInstance().addTest("Intake", getIntakeTest());
    //     Tester.getInstance().getTest("Intake").setTimeBetweenTests(1);
    // }
}