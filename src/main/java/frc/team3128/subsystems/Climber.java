package frc.team3128.subsystems;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
import common.core.controllers.TrapController;
import common.core.subsystems.NAR_PIDSubsystem;
import common.hardware.motorcontroller.NAR_Motor;

public class Climber extends NAR_PIDSubsystem {

    private boolean getmeasurement() {
        throw new UnsupportedOperationException("Not supported yet.");
    }
    
    public enum Setpoint{
        /*define where the climber where reach (states) */;
        public final double setpoint;
        private Setpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    }

    public Climber() {
        super(new TrapController(PID_Constants, TRAP_CONSTRAINTS)); /*define your controller using TrapController*/
        configMotors();
        setTolerance(POSITION_TOLERANCE);
        setConstraints(POSITION_MINIMUM,POSITION_MAXIMUM);

    public static synchronized Climber getInstance(){
        return this; /*placeholder replace later */
    };
    

    private void configMotors() {
        NAR_CANspark leftMotor= new NAR_CANspark();
        NAR_CANspark rightMotor= new NAR_CANspark();
        leftMotor.setInverted(true);
        //be sure to set one motor as inverted
        String setNeutralMode=BRAKE;
        //setStatusFrames to define rate of transmission between motor and controller
        leftMotor.setStatusFrames(SparkMaxConfig.DEFAULT);
        rightMotor.setStatusFrames(SparkMaxConfig.FOLLOWER);
    }
    
    @Override
    protected void useOutput(double output, double setpoint) {
        if ( retracting and neutral) {
            leftMotor.setVolts(0)
            rightMotor.setVolts(0)
            return;
        }

        //set right motor to output volts
        rightMotor.setVolts(output);
        //set left motor to output volts
        leftMotor.setVolts(output);
    }
    

    public Command setClimber(double power) {
        return sequence(
            runOnce(() -> disable()),
            //then runOnce so the left motor will set power
            runOnce(() -> leftMotor.setVolts(power));
            //then runOnce so the right motor will set power
            runOnce(() -> rightMotor.setVolts(power));
        );
    }

    @Override
    public double getMeasurement() {
        return motors[0].getPosition();
    }
    
    /*create a boolean isClimbed(), using the getmeasurement() 
    method so the robot will know if climber was used
    */
    public boolean isClimbed(){
      if (getmeasurement()) {
        return true;
      }  
      else 
        return false;
    }


    public Command climbTo(DoubleSupplier setpoint){
        return runOnce(() -> startPID(setpoint.getAsDouble()));
    }

    public Command climbToState(Setpoint state) {
        /*
         using the climbTo command, also create a command which will 
         climb to the enum state you want
         */

    }
    setClimber=0
    public Command reset(double position) {
       return runOnce(() -> motor.resetPosition(0));
    }
}
}
    /**
     
     * @param position Position to reset to.
