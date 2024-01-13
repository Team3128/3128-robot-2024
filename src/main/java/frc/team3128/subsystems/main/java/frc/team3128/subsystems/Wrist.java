package frc.team3128.subsystems;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;

import static frc.team3128.Constants.WristConstants.*;
import static frc.team3128.PositionConstants.Position;

import frc.team3128.common.hardware.motorcontroller.NAR_CANSparkMax;
import frc.team3128.common.hardware.motorcontroller.NAR_CANSparkMax.EncoderType;

public class Wrist extends NAR_PIDSubsystems{
    public NAR_CANSparkMax m_wrist;
    private static Wrist instance;

    public static synchronized Wrist getInstnace() {
        if(instance == null){
            instance = new Pivot();
        }
        return instance;
    }

    public Wrist() {
        super(new PIDController(kP,kI,kD), kS, kV, kG);
    }
}
