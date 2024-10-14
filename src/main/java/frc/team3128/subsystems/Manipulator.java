package frc.team3128.subsystems;

import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import common.hardware.motorcontroller.NAR_CANSpark;
import common.hardware.motorcontroller.NAR_Motor;
import common.utility.shuffleboard.NAR_Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Manipulator extends SubsystemBase {

    public final double currentThreshold;
    private final double intakePower;
    private final double outtakePower;
    private final double stallPower;
    private final double lagSeconds;
    private BooleanSupplier debug;
    private DoubleSupplier powerSetpoint;
    private NAR_CANSpark motors;

     /**
      * Creates an Manipulator object.
      * @param currentThreshold Current when object is intook.
      * @param intakePower Intake power.
      * @param outtakePower Outtake power.
      * @param stallPower Stall Power, run when Manipulator has a game piece.
      * @param lagSeconds Time before current check is run.
      * @param motors Manipulator motors.
      */

    public Manipulator(double currentThreshold, double intakePower, double outtakePower, double stallPower, double lagSeconds, NAR_CANSpark motors){
         this.currentThreshold = currentThreshold;
         this.intakePower = intakePower;
         this.outtakePower = outtakePower;
         this.stallPower = stallPower;
         this.lagSeconds = lagSeconds;
         this.motors = motors;
         configMotors();
    }

    //create a getInstance() so you can use it later in command
    
    public void configMotors() {
        //define one motor (use NAR_CANspark as the type of motor)
        //motor should have setInverted as false
        //set motor's NeutralMode as Brake
    };

    public double getCurrent(){
       return motors.getStallCurrent();
    }
    
    public boolean hasObjectPresent(){
        return Math.abs(getCurrent()) > currentThreshold;
    }
    
    protected void setPower(double power){
        final double output = (debug != null && debug.getAsBoolean()) ? powerSetpoint.getAsDouble() : power;
        //set the power
    }
    
    public Command run(double power){
        return runOnce(()-> setPower(power));
    }
    
    public Command intake() {
        return sequence(
            run(intakePower),
            waitSeconds(lagSeconds),
            waitUntil(()-> hasObjectPresent()),
            runOnce(()-> setPower(stallPower))
        );
    }
    
    public Command outtake() {
        return run(outtakePower);
    }
    
    /**
     * Initializes shuffleboard with debug elements.
     */
    public void initShuffleboard() {
        NAR_Shuffleboard.addData(getName(), "Object Present", ()-> hasObjectPresent(), 0, 0);
        NAR_Shuffleboard.addData(getName(), "Manip current", () -> getCurrent(), 0, 1);
        NAR_Shuffleboard.addData(getName(), "TOGGLE", false, 1, 0).withWidget("Toggle Button").getEntry();
        debug = NAR_Shuffleboard.getBoolean(getName(), "TOGGLE");
        NAR_Shuffleboard.addData(getName(), "DEBUG", ()-> debug.getAsBoolean(), 1, 1);
        powerSetpoint = NAR_Shuffleboard.debug(getName(), "Debug_Setpoint", 0, 1,2);
    }
}
