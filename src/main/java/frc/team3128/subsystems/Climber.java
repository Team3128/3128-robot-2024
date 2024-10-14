package frc.team3128.subsystems;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
import common.core.controllers.TrapController;
import common.core.subsystems.NAR_PIDSubsystem;
import common.hardware.motorcontroller.NAR_Motor;

public class Climber extends NAR_PIDSubsystem {
    
    public enum Setpoint{
        /*define where the climber where reach (states) */;
        public final double setpoint;
        private Setpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    }

    public Climber() {
        super(new TrapController(null, null) /*define your controller using TrapController*/); 
        configMotors();
        //setTolerance of controller (max error of position)
        //setConstraints of controller (min pos of climber and max pos of climber)
    }

    /*create a getInstance() so you can use it later in commands
    ie. public static synchronized Climber getInstance(){}
    */

    private void configMotors() {
        //define left and right motors (use NAR_CANspark as the type of motor)
        //be sure to set one motor as inverted
        //be sure to setNeutralMode as BRAKE
        //setStatusFrames to define rate of transmission between motor and controller
    }
    
    @Override
    protected void useOutput(double output, double setpoint) {
        if (/* if intake is retracting and neutral*/) {
            //set left motor to 0 volts (ie. leftMotor.setVolts()
            //set right motor to 0 volts
            return;
        }

        //set right motor to output volts
        //set left motor to output volts
    }
    

    public Command setClimber(double power) {
        return sequence(
            runOnce(() -> disable()),
            //then runOnce so the left motor will set power
            //then runOnce so the right motor will set power
        );
    }

    @Override
    public double getMeasurement() {
        // return motors[0].getPosition();
    }
    
    /*create a boolean isClimbed(), using the getmeasurement() 
    method so the robot will know if climber was used
    */


    public Command climbTo(DoubleSupplier setpoint){
        return runOnce(() -> startPID(setpoint.getAsDouble()));
    }

    public Command climbToState(Setpoint state) {
        /*
         using the climbTo command, also create a command which will 
         climb to the enum state you want
         */
    }
    
    /**
     * Reset climber position.
     * @param position Position to reset to.
     * @return Command that resets the climber position.
     */
    public Command reset(double position) {
       return runOnce(() -> /*motor.resetPosition(0)*/);
    }
}
