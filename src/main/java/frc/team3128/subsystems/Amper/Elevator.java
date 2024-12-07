// package frc.team3128.subsystems.Amper;

// import common.core.controllers.Controller;
// import common.core.controllers.PIDFFConfig;
// import common.core.subsystems.PositionSubsystemBase;
// import common.hardware.motorcontroller.NAR_Motor;
// import common.hardware.motorcontroller.NAR_TalonFX;
// import common.hardware.motorcontroller.NAR_Motor.MotorConfig;

// public class Elevator extends PositionSubsystemBase{

//     private static PIDFFConfig controllerConfig = new PIDFFConfig(0, 0, 0, 0, 0, 0);
//     private static Controller controller = new Controller(controllerConfig, Controller.Type.POSITION);

//     private static NAR_Motor leftMotor = new NAR_TalonFX(0);
//     private static NAR_Motor rightMotor = new NAR_TalonFX(1);

//     public Elevator() {
//         super(controller, leftMotor, rightMotor);
//         initShuffleboard();
//     }

//     @Override
//     protected void configMotors() {
//         MotorConfig motorConfig = 
//         new MotorConfig(
//             12, 
//             1, 
//             40, 
//             12, 
//             false, 
//             NAR_Motor.Neutral.BRAKE, 
//             NAR_Motor.StatusFrames.POSITION
//         );

//         leftMotor.configMotor(motorConfig);
//         rightMotor.configMotor(motorConfig.invertFollower());
//     }

//     @Override
//     protected void configController() {
//         controller.setTolerance(0.1);
//         controller.setInputRange(0, 30);
//         setSafetyThresh(3);
//     }
// }