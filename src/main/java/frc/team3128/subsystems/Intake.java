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
        EXTENDED(-202),
        SOURCE(-60),
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
            PIVOT_MOTOR.setStatusFrames(SparkMaxConfig.POSITION);
        }

        @Override
        public void useOutput(double output, double setpoint) {
            PIVOT_MOTOR.setVolts(MathUtil.clamp(output, -12, 12));
        }

        public Command pivotTo(DoubleSupplier setpoint) {
            return runOnce(()-> startPID(setpoint.getAsDouble()));
        }

        public Command pivotNoRequirements(double setpoint) {
            return new InstantCommand(()-> startPID(setpoint));
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
            super(STALL_CURRENT, INTAKE_POWER, OUTTAKE_POWER, STALL_POWER, 0.3, ROLLER_MOTOR);
            limitSwitch = new DigitalInput(9);
            initShuffleboard();
        }

        @Override
        protected void configMotors() {
            ROLLER_MOTOR.setInverted(false);
            ROLLER_MOTOR.setNeutralMode(Neutral.COAST);
            ROLLER_MOTOR.setCurrentLimit(CURRENT_LIMIT);
            ROLLER_MOTOR.enableVoltageCompensation(12);
            ROLLER_MOTOR.setStatusFrames(SparkMaxConfig.VELOCITY);
        }

        public Command runNoRequirements(double power) {
            return new InstantCommand(()-> setPower(power));
        }

        public Command ramShotOuttake() {
            return runNoRequirements(-1);
        }

        public Command outtakeNoRequirements() {
            return runNoRequirements(OUTTAKE_POWER);
        }

        public Command intakeNoRequirements() {
            return runNoRequirements(INTAKE_POWER);
        }

        public Command miniOuttake() {
            return sequence(
                runManipulator(-0.1),
                waitSeconds(0.25),
                runManipulator(0)
            );
        }

        public Command serialize() {
            return sequence(
                // runOnce(()-> DriverStation.reportWarning("Serialize: CommandStarting", false)),
                runManipulator(-0.1),
                waitUntil(()-> !hasObjectPresent()),
                runManipulator(0.1),
                waitUntil(()-> hasObjectPresent()),
                runManipulator(0)
                // runOnce(()-> DriverStation.reportWarning("Serialize: CommandEnding", false))
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
                ROLLER_MOTOR, 
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
        // NAR_Shuffleboard.addData("IsRetracting", "Boolean", ()-> isRetracting,0, 0);
    }

    public Command retract(boolean shouldStall) {
        return sequence(
            // runOnce(()-> DriverStation.reportWarning("Retract: CommandStarting", false)),
            // CmdManager.vibrateController(),
            runOnce(()-> isRetracting = true),
            waitUntil(()-> Climber.getInstance().isNeutral()),
            //intakeRollers.runManipulator(shouldStall ? STALL_POWER : 0),
            intakePivot.pivotTo(-10),
            waitUntil(() -> intakePivot.atSetpoint()),
            intakePivot.runPivot(0.2),
            waitSeconds(0.1),
            intakePivot.runPivot(0),
            parallel(
                either(intakeRollers.serialize().withTimeout(4).andThen(intakeRollers.runManipulator(0)), intakeRollers.runManipulator(0), ()-> shouldStall).withTimeout(0.5),
                sequence(
                    waitSeconds(0.5),
                    runOnce(()-> isRetracting = false),
                    intakePivot.reset(0)
                )
            )
            // runOnce(()-> DriverStation.reportWarning("Retract: CommandEnding", false))
        );
    }
    
    public Command intake(Setpoint setpoint) {
        return sequence(
            // runOnce(()-> DriverStation.reportWarning("Intake: CommandStarting", false)),
            runOnce(()-> isRetracting = true),
            intakeRollers.runManipulator(INTAKE_POWER),
            intakePivot.pivotTo(setpoint.angle),
            intakeRollers.intake(),
            retract(true)
            // runOnce(()-> DriverStation.reportWarning("Intake: CommandEnding", false))
        );
    }

    public Command outtake() {
        return sequence (
            // runOnce(()-> DriverStation.reportWarning("Outtake: CommandStarting", false)),
            intakePivot.pivotTo(Setpoint.EXTENDED.angle),
            waitUntil(()-> intakePivot.getMeasurement() < -45).withTimeout(2),
            intakeRollers.runManipulator(-1),
            waitSeconds(0.5),
            retract(false)
            // runOnce(()-> DriverStation.reportWarning("Outtake: CommanedEnding", false))
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
            intakeRollers.runManipulator(STALL_POWER),
            intakePivot.pivotTo(()-> Climber.getInstance().getAngle()),
            waitUntil(() -> intakePivot.atSetpoint()),
            intakePivot.runPivot(0)
        ).withTimeout(3);
    }

    public State getRunningState() {
        if (ROLLER_MOTOR.getState() != State.DISCONNECTED && PIVOT_MOTOR.getState() != State.DISCONNECTED) {
            return State.RUNNING; 
        }
        return State.DISCONNECTED;
    }

    public UnitTest getIntakeTest() {
        return new UnitTest
        (
            "testIntakeExtend",
            intake(Setpoint.EXTENDED).withTimeout(INTAKE_TEST_TIMEOUT)
        );
    }

    public void addIntakeTests() {
        Tester.getInstance().addTest("Intake", intakeRollers.getRollersTest());
        Tester.getInstance().addTest("Intake", intakePivot.getPivotTest());
        Tester.getInstance().addTest("Intake", getIntakeTest());
        Tester.getInstance().getTest("Intake").setTimeBetweenTests(1);
    }
}