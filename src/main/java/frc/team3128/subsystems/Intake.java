package frc.team3128.subsystems;

import static frc.team3128.Constants.IntakeConstants.*;
import static edu.wpi.first.wpilibj2.command.Commands.deadline;
// import static edu.wpi.first.wpilibj2.command.Commands.either;
// import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;
// import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;

// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import static edu.wpi.first.wpilibj2.command.Commands.*;

// import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

// import com.ctre.phoenix.motorcontrol.NeutralMode;

import common.core.subsystems.ManipulatorTemplate;
import common.core.subsystems.PivotTemplate;

import common.core.controllers.TrapController;

// import common.hardware.motorcontroller.NAR_CANSpark;
// import common.hardware.motorcontroller.NAR_CANSpark.SparkMaxConfig;
// import common.hardware.motorcontroller.NAR_Motor;
import common.hardware.motorcontroller.NAR_Motor.Neutral;

// import common.utility.narwhaldashboard.NarwhalDashboard.State;
// import common.utility.shuffleboard.NAR_Shuffleboard;

import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj.DigitalInput;

import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

import common.utility.tester.CurrentTest;


import common.utility.tester.CurrentTest;
import common.utility.tester.Tester;
import common.utility.tester.Tester.*;


public class Intake {

    public enum Setpoint {

        //define your enums here (the diff heights you want to place cones at - ie. low pole, mid pole, etc)
        INTAKING(0),
        LOWPOLE(20),
        MIDPOLE(45),
        HIGHPOLE(70);

        public final double angle;
        private Setpoint(double angle) {
            this.angle = angle;
        }
    }

    public class IntakePivot extends PivotTemplate {

        private double currSetpoint;

       private IntakePivot() {
           super(new TrapController(PIDConstants, TRAP_CONSTRAINTS), PIVOT_MOTOR);
           setTolerance(ANGLE_TOLERANCE);
           setConstraints(POSITION_MINIMUM, POSITION_MAXIMUM);
           initShuffleboard();
           currSetpoint = Setpoint.INTAKING.angle;
           //define your controller here (using trap controller)
           //setTolerance here (aka the max angle error that's acceptable)
           //setConstraints (min and max position of the intake - in degrees)
           //call initShuffleboard so you can debug using shuffleboard


       }


       @Override 
       protected void configMotors() {
           PIVOT_MOTOR.setInverted(false);
           PIVOT_MOTOR.setNeutralMode(Neutral.COAST);
           //define one motor (use NAR_CANspark as the type of motor)
           //motor should have setInverted as false
           //set motor's NeutralMode as COAST
      
       }


       @Override
       public void useOutput(double output, double setpoint) {
           PIVOT_MOTOR.setVolts(MathUtil.clamp(output, -12, 12));
          // motor.setVolts(MathUtil.clamp(output, -12, 12));
       }
      
       public Command pivotTo(double setpoint) {
        currSetpoint = setpoint;
       return runOnce(()-> startPID(setpoint));
        }

       public Command pivotTo(DoubleSupplier setpoint) {
            currSetpoint = setpoint.getAsDouble();
           return runOnce(()-> startPID(setpoint.getAsDouble()));
       }

        public double GetSetpoint() {
            return currSetpoint;
        }

        public Command reset() {
            return runOnce(() -> PIVOT_MOTOR.resetPosition(Setpoint.INTAKING.angle));
        }
   }

   public class IntakeRollers extends ManipulatorTemplate {

        public Command pivotTo(double setpoint) {
            return runOnce(()-> startPID(setpoint));
        }

        public SetpointTest getPivotTest(Setpoint setpoint) {
            return new SetpointTest(
                "testIntakePivot",
                setpoint.angle,
                SETPOINT_TEST_PLATEAU,
                SETPOINT_TEST_TIMEOUT
            );
        }


   }

    
   public class IntakeRollers extends ManipulatorTemplate {


    // private DigitalInput limitSwitch;
 
 
    private IntakeRollers() {
        super(STALL_CURRENT, INTAKE_POWER, OUTTAKE_POWER, STALL_POWER, 0.3, RIGHT_ROLLER_MOTOR);

 
 
        /*
        *define constants for all of the above parameters
        * Creates an Manipulator object.
        *currentThreshold Current when object is intook.
       
        *intakePower Intake power.
        *outtakePower Outtake power.
        *stallPower Stall Power, run when Manipulator has a game piece.
        *lagSeconds Time before current check is run.
        *motors Manipulator motors.
        */
       
        //??????????????????????????????????????limitSwitch = new DigitalInput(7);
        initShuffleboard();
    }
 
    @Override
    protected void configMotors() {
        //NAR_CANSpark
        //define one motor (again, using nar_canspark)???
        //TYLER I did it in constants
      
        //setInverted as false
        RIGHT_ROLLER_MOTOR.setInverted(false);
        //setNeutralMode as COAST
        RIGHT_ROLLER_MOTOR.setNeutralMode(Neutral.COAST);
        //setCurrentLimit as 40
        RIGHT_ROLLER_MOTOR.setCurrentLimit(CURRENT_LIMIT);
    }

    public void setPower(double power) {
        setPower(power);
    }

    public CurrentTest getRollersTest() {
        return new CurrentTest(
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
    //create a getInstance() so you can always create a new intake
    //ie. public static synchronized Intake getInstance(){
    public static synchronized Intake getInstance() {
        if(instance == null) {
            instance = new Intake();
        }
        return instance;
    }

    private Intake(){
        //create a new intakePivot
        intakePivot = new IntakePivot();
        //create a new intakeRollers
        intakeRollers = new IntakeRollers();
    }

    public Command intake(Setpoint setpoint) {
        return sequence(
            deadline(
                //use intakeRollers to call intake(). Remember to add a comma after your sentence (proper format for sequence/parallel commands)
                intakeRollers.intake(),
                sequence(
                    //have your intakePivot pivot to setpoint.angle
                    intakePivot.pivotTo(setpoint.angle),
                    waitUntil(() -> hasObjectPresent()),
                    runOnce(() -> intakeRollers.setPower(STALL_POWER))
                )
            )
        );
    }

    public Command outtake(Setpoint setpoint) {
        return sequence (
            //outtaking so have your intakePivot pivot to your enum state
            intakePivot.pivotTo(setpoint.angle),
            //have intakeRollers run manipulator at outake power
            intakeRollers.runManipulator(OUTTAKE_POWER)
        );
    }
   
    public boolean hasObjectPresent(){
        return Math.abs(getCurrent()) > STALL_CURRENT;
    }
   
    public double getCurrent(){
        return ROLLER_MOTOR.getStallCurrent();
    }


    private void setPower(double stallPower) {
        ROLLER_MOTOR.set(stallPower);
    }

    public State getRunningState() {
        //how do you find the state?
        if (PIVOT_MOTOR.getState() == State.DISCONNECTED) return State.DISCONNECTED;
        if (ROLLER_MOTOR.getState() == State.DISCONNECTED) return State.DISCONNECTED;
        return State.RUNNING;
    }

    public Setpoint GetSetpointUp(double setpoint) {
        if (setpoint == Setpoint.MIDPOLE.angle) {
            return Setpoint.HIGHPOLE;
        } else if (setpoint == Setpoint.LOWPOLE.angle) {
            return Setpoint.MIDPOLE;
        } else if (setpoint == Setpoint.INTAKING.angle) {
            return Setpoint.LOWPOLE;
        }

        return Setpoint.HIGHPOLE;
    }

    public Setpoint GetSetpointDown(double setpoint) {
        if (setpoint == Setpoint.LOWPOLE.angle) {
            return Setpoint.INTAKING;
        } else if (setpoint == Setpoint.MIDPOLE.angle) {
            return Setpoint.LOWPOLE;
        } else if (setpoint == Setpoint.HIGHPOLE.angle) {
            return Setpoint.MIDPOLE;
        }

        return Setpoint.INTAKING;
    }
}
