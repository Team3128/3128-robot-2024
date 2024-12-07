// package frc.team3128.subsystems.Amper;

// import common.core.controllers.Controller;
// import common.core.controllers.PIDFFConfig;
// import common.core.subsystems.VelocitySubsystemBase;
// import common.hardware.motorcontroller.NAR_Motor;
// import common.hardware.motorcontroller.NAR_TalonFX;
// import common.hardware.motorcontroller.NAR_Motor.MotorConfig;

// public class Roller extends VelocitySubsystemBase{

//     private static PIDFFConfig controllerConfig = new PIDFFConfig(0, 0, 0, 0, 0, 0);
//     private static Controller controller = new Controller(controllerConfig, Controller.Type.VELOCITY);

//     private static NAR_Motor leftMotor = new NAR_TalonFX(3);
//     private static NAR_Motor rightMotor = new NAR_TalonFX(4);

//     private static MotorConfig motorConfig = 
//         new MotorConfig(
//             6, 
//             1, 
//             40, 
//             12, 
//             false, 
//             NAR_Motor.Neutral.COAST, 
//             NAR_Motor.StatusFrames.VELOCITY
//         );

//     public Roller() {
//         super(controller, leftMotor, rightMotor);
//     }

//     @Override
//     protected void configMotors() {
//         leftMotor.configMotor(motorConfig);
//         rightMotor.configMotor(motorConfig.invertFollower());
//     }

//     @Override
//     protected void configController() {
//         controller.setTolerance(300);
//         controller.setInputRange(0, 4000);
//         setSafetyThresh(2);
//     }
// }