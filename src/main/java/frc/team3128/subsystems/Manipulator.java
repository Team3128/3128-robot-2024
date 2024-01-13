package frc.team3128.subsystems;

import common.core.controllers.Controller;
import common.core.controllers.Controller.Type;
import common.core.subsystems.NAR_PIDSubsystem;
import common.hardware.motorcontroller.NAR_CANSparkMax;
import common.hardware.motorcontroller.NAR_Motor.Neutral;
import common.utility.shuffleboard.NAR_Shuffleboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.team3128.Constants.ManipulatorConstants.*;

import com.revrobotics.CANSparkMax.IdleMode;
public class Manipulator extends SubsystemBase {
    
    public NAR_CANSparkMax m_leftmotor;
    public NAR_CANSparkMax m_rightmotor;
    public boolean outtaking = false;
    
    private static Manipulator instance;
    
    
    public Manipulator(){
        configMotors();
    }
    
    public static synchronized Manipulator getInstance(){
        if(instance == null){
            instance = new Manipulator();
        }
        return instance;
    }
    
    public void configMotors(){
        m_leftmotor = new NAR_CANSparkMax(MANIP_LEFT_MOTOR_ID);
        m_rightmotor = new NAR_CANSparkMax(MANIP_RIGHT_MOTOR_ID);
        m_leftmotor.setInverted(true);
        m_leftmotor.enableVoltageCompensation(12.0);
        m_rightmotor.enableVoltageCompensation(12.0);
        m_leftmotor.setNeutralMode(Neutral.COAST);
        m_rightmotor.setNeutralMode(Neutral.COAST);
    }
    
    public void intake(){
        outtaking = false;
        setPower(INTAKE_POWER);
    }
    
    public void outtake(){
        outtaking = true;
        setPower(SHOOTING_POWER);
    }
    
    public double getCurrent(){
        return m_rightmotor.getStallCurrent();
    }
    
    public boolean hasObjectPresent(){
        return Math.abs(getCurrent()) > ABSOLUTE_THRESHOLD;
    }
    
    public void setPower(double power){
        m_leftmotor.set(power);
        m_rightmotor.set(power);
    }
    
    public void stopManip(){
        setPower(0);
    }
    
    public void initShuffleboard() {
        NAR_Shuffleboard.addComplex(getSubsystem(), getName(), instance, 0, 0);
        NAR_Shuffleboard.addData("Manipulator", "Manip current", () -> getCurrent(), 0, 1);
        NAR_Shuffleboard.addData("Manipulator", "ObjectPresent", ()-> hasObjectPresent(), 1, 1);
    }
    
    public CommandBase Intake(){
        return runOnce(() -> intake());
    }

}
