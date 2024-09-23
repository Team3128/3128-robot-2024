package frc.team3128.subsystems;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import common.hardware.motorcontroller.NAR_CANSpark;
import common.utility.shuffleboard.NAR_Shuffleboard;

public class Manipulator extends SubsystemBase {

    public NAR_CANSpark m_roller;

    private static Manipulator instance;

    public static boolean CONE = true;

    public boolean outtaking = false;

    public Manipulator(){
        //configure motor
    }

    public static Manipulator getInstance() {
        if (instance == null){
            //instance = manipulator (new)
        }
        
        //return instance
    }

    @Override
    public void periodic() {
        if (Math.abs(getCurrent()) > ABSOLUTE_THRESHOLD + 40 && !outtaking)
            //stall power
    }

    public void configMotor(){
        //add roller to a sparkmax
        //set roller inverted - false/true??
        //set roller to neutral mode
        //enable roller voltage composition 
    }

    public void set(double power){
        //set power of roller
    }

    public void forward(){
        //set power to roller power
    }

    public void reverse(){
        //set power to roller power(negative)
    }

    public void stopRoller(){
        //set roller to 0
    }

    public double getCurrent(){
        //return 0
        //get current stator motor
    }

    public double getVoltage() {
        //return 0 
        //return roller - output voltage
    }

    public boolean hasObjectPresent(){
        //return outtaking
        //return absolute threshodl
    }

    public void intake(boolean cone) {
        //set outtaking to false
        //cone == ??
        //if statement - when cone moves forward
        //else ??
    }    

    public void outtake(){
        //set outtaking to false
        //if statement - cone in reverse
        //else statement - ??
    }

    public void stallPower() {
        //set cone to stall power
    }

    public void initShuffleboard() {
        NAR_Shuffleboard.addData("Manipulator", "Manip current", () -> getCurrent(), 0, 1);
        NAR_Shuffleboard.addData("Manipulator", "get", () -> m_roller.getMotorOutputPercent(), 0, 3);
        NAR_Shuffleboard.addData("Manipulator", "ObjectPresent", ()-> hasObjectPresent(), 1, 1);
    }
}