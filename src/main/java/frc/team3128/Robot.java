// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team3128;

import java.util.Optional;

import common.core.misc.NAR_Robot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.MatchType;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.team3128.autonomous.AutoPrograms;
import frc.team3128.subsystems.Swerve;

import org.littletonrobotics.junction.Logger;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation.
 */
public class Robot extends NAR_Robot {

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
    public static AutoPrograms autoPrograms = new AutoPrograms();

    public static synchronized Robot getInstance() {
        if (instance == null) {
            instance = new Robot();
        }
        return instance;
    }

    @Override
    public void robotInit(){
        new Trigger(this::isEnabled).negate().debounce(2).onTrue(new InstantCommand(()-> Swerve.getInstance().setBrakeMode(false)).ignoringDisable(true));
        LiveWindow.disableAllTelemetry();

        addReceiver(true, LoggingState.SESSION);
        
    }

    @Override
    public void robotPeriodic(){
      
    }

    @Override
    public void autonomousInit() {
        Command m_autonomousCommand = autoPrograms.getAutonomousCommand();
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }

        if(DriverStation.getMatchType() != MatchType.None)
            addReceiver(true, LoggingState.FULLMATCH);
        Logger.start();

    }

    @Override
    public void autonomousPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void teleopInit() {
        CommandScheduler.getInstance().cancelAll();

        if(DriverStation.getMatchType() != MatchType.None)
            addReceiver(true, LoggingState.FULLMATCH);
        Logger.start();
        
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
        Swerve.getInstance().setBrakeMode(true);
    }
    
    @Override
    public void disabledPeriodic() {

    }
}
