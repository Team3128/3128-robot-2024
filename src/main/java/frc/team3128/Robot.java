// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.team3128;


import java.util.Optional;


import org.littletonrobotics.junction.Logger;


import common.core.misc.NAR_Robot;
import common.hardware.camera.Camera;
import common.hardware.limelight.LEDMode;
import common.utility.Log;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation.MatchType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import static edu.wpi.first.wpilibj2.command.Commands.*;


// import frc.team3128.Constants.LedConstants.Colors;
import frc.team3128.autonomous.AutoPrograms;
// import frc.team3128.commands.CmdManager;


/**
* The VM is configured to automatically run this class, and to call the functions corresponding to
* each mode, as described in the TimedRobot documentation.
*/
public class Robot extends NAR_Robot {


   private boolean hasInitialized = false;


   public static Alliance alliance;


   public static Alliance getAlliance() {
       if (alliance == null) {
           Optional<Alliance> DSalliance = DriverStation.getAlliance();
           if (DSalliance.isPresent()) alliance = DSalliance.get();
       }
       return alliance;
   }


   public static Robot instance;


   public static RobotContainer m_robotContainer = new RobotContainer();
   public static AutoPrograms autoPrograms;


   public static synchronized Robot getInstance() {
       if (instance == null) {
           instance = new Robot();
       }
       return instance;
   }


   @Override
   public void robotInit(){
      
       try {
           Class<?> container = Class.forName("processor.ClassManager");
           container.getDeclaredMethod("process").invoke(null);
       } catch (Exception e) {
           Log.info("Annotation Processor", "Failed to invoke ClassManager");
           e.printStackTrace();
       }


       autoPrograms = new AutoPrograms();
       m_robotContainer.initDashboard();
       LiveWindow.disableAllTelemetry();
       // runOnce(()-> Swerve.getInstance().zeroGyro(Robot.getAlliance() == Alliance.Red ? 0 : 180));
       // Swerve.getInstance().resetOdometry((new Pose2d(new Translation2d(1.45, 4.1), Rotation2d.fromDegrees(180)))); //1.45, 4.1
       // Alliance allianceTemp = getAlliance();
       // if (allianceTemp == null) {
       //     Log.info("Alliance", "Did not have correct color");
       // }
       // else {
       //     Log.info("Alliance", "We are alliance " + allianceTemp);
       // }
       // Log.info("Gyro Angle", "" + Swerve.getInstance().getYaw());
   }


   @Override
   public void driverStationConnected() {
       Log.info("State", "DS Connected");
       Log.info("Alliance", getAlliance().toString());
       if (getAlliance() == Alliance.Red) {
           Camera.addIgnoredTags(3, 4, 5, 11, 12);
       } else {
           Camera.addIgnoredTags(6, 7, 8, 15, 16);
       }
       if (!NAR_Robot.logWithAdvantageKit) return;
       if(DriverStation.getMatchType() != MatchType.None){
           addReceiver(true, LoggingState.FULLMATCH);
       }else{
           addReceiver(true, LoggingState.SESSION);
       }


       Logger.start();
   }


   @Override
   public void robotPeriodic(){
       Camera.updateAll();
   }


   @Override
   public void autonomousInit() {
       RobotContainer.limelight.setLEDMode(LEDMode.ON);
   


       Camera.enableAll();
       Camera.overrideThreshold = 0;
       Camera.validDist = 30;
  
       Command m_autonomousCommand = autoPrograms.getAutonomousCommand();
       if (m_autonomousCommand != null) {
           m_autonomousCommand.schedule();
       }
   }


   @Override
   public void autonomousPeriodic() {
       CommandScheduler.getInstance().run();
   }


   @Override
   public void teleopInit() {
       RobotContainer.limelight.setLEDMode(LEDMode.OFF);
      
       Camera.overrideThreshold = 30;
       Camera.validDist = 0.5;
       Camera.enableAll();
       CommandScheduler.getInstance().cancelAll();
      
       // CmdManager.neutral().schedule();
   }


   @Override
   public void teleopPeriodic() {
       CommandScheduler.getInstance().run();
   }


   @Override
   public void simulationInit() {
      
   }


   @Override
   public void simulationPeriodic() {
       CommandScheduler.getInstance().run();
   }


   @Override
   public void disabledInit() {
     
       CommandScheduler.getInstance().cancelAll();
       sequence(
           waitSeconds(3.0).ignoringDisable(true)
       ).schedule();


       if (hasInitialized) {
         
       }
       hasInitialized = true;
   }


   @Override
   public void disabledExit() {
  
   }
  
   @Override
   public void disabledPeriodic() {
       CommandScheduler.getInstance().run();
   }
}
