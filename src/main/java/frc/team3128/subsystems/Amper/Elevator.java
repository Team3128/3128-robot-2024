package frc.team3128.subsystems.Amper;

import common.core.controllers.Controller;
import common.core.controllers.PIDFFConfig;
import common.core.subsystems.PositionSubsystemBase;
import common.hardware.motorcontroller.NAR_Motor;
import common.hardware.motorcontroller.NAR_TalonFX;
import edu.wpi.first.math.util.Units;
import common.hardware.motorcontroller.NAR_Motor.MotorConfig;

public class Elevator extends PositionSubsystemBase{

    private static Elevator instance;

    private static PIDFFConfig controllerConfig = new PIDFFConfig(0.95, 0, 0, 0.21115, 0.00182, 0.00182, 0.0);
    private static Controller controller = new Controller(controllerConfig, Controller.Type.POSITION);

    private static NAR_Motor leftMotor = new NAR_TalonFX(0);
    // private static NAR_Motor rightMotor = new NAR_TalonFX(1);

    public static final double GEAR_RATIO = 1.0 / (6 + 2/3);
    public static final double WHEEL_CIRCUMFERENCE = Units.inchesToMeters(0.9023) * Math.PI;
    public static final double UNIT_CONV_FACTOR = GEAR_RATIO * WHEEL_CIRCUMFERENCE * 100;

    public static synchronized Elevator getInstance() {
        if (instance == null)
            instance = new Elevator();
        return instance;
    }

    private Elevator() {
        super(controller, leftMotor);
        initShuffleboard();
    }

    @Override
    protected void configMotors() {
        MotorConfig motorConfig = 
        new MotorConfig(
            UNIT_CONV_FACTOR, 
            1, 
            40, 
            12, 
            true, 
            NAR_Motor.Neutral.BRAKE, 
            NAR_Motor.StatusFrames.POSITION
        );

        leftMotor.configMotor(motorConfig);
        // rightMotor.configMotor(motorConfig.invertFollower());
    }

    @Override
    protected void configController() {
        controller.setTolerance(0.25);
        controller.setInputRange(0, 30);
        setSafetyThresh(3);
    }
}