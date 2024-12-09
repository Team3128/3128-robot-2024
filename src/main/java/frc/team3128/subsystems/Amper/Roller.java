package frc.team3128.subsystems.Amper;

import common.core.controllers.Controller;
import common.core.controllers.PIDFFConfig;
import common.core.subsystems.VelocitySubsystemBase;
import common.hardware.motorcontroller.NAR_CANSpark;
import common.hardware.motorcontroller.NAR_Motor;
import common.hardware.motorcontroller.NAR_TalonFX;
import common.hardware.motorcontroller.NAR_CANSpark.ControllerType;
import common.hardware.motorcontroller.NAR_Motor.MotorConfig;

public class Roller extends VelocitySubsystemBase{

    private static Roller instance;

    private static PIDFFConfig controllerConfig = new PIDFFConfig(0.00218, 0, 0, 0, 0.002, 0);
    private static Controller controller = new Controller(controllerConfig, Controller.Type.VELOCITY);

    private static NAR_Motor leftMotor = new NAR_CANSpark(20, ControllerType.CAN_SPARK_FLEX);
    // private static NAR_Motor rightMotor = new NAR_TalonFX(4);

    private static MotorConfig motorConfig = 
        new MotorConfig(
            1, 
            1, 
            40, 
            12, 
            true, 
            NAR_Motor.Neutral.COAST, 
            NAR_Motor.StatusFrames.VELOCITY
        );

    public static synchronized Roller getInstance() {
        if (instance == null)
            instance = new Roller();
        return instance;
    }

    private Roller() {
        super(controller, leftMotor);
    }

    @Override
    protected void configMotors() {
        leftMotor.configMotor(motorConfig);
        // rightMotor.configMotor(motorConfig.invertFollower());
    }

    @Override
    protected void configController() {
        controller.setTolerance(500);
        controller.setInputRange(0, 5500);
        setSafetyThresh(2);
    }
}