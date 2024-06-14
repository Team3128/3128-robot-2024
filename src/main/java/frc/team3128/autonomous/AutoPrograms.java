package frc.team3128.autonomous;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team3128.Robot;
import frc.team3128.subsystems.Swerve;
import common.utility.narwhaldashboard.NarwhalDashboard;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.team3128.commands.CmdManager.autoShoot;

import java.util.HashMap;

/**
 * Class to store information about autonomous routines.
 * @author Daniel Wang, Mason Lam
 */

public class AutoPrograms {

    private HashMap<String, Command> autoMap = new HashMap<String, Command>();

    public AutoPrograms() {
        Trajectories.initTrajectories();
        initAutoSelector();
    }

    private void initAutoSelector() {
        final String[] autoStrings = new String[] {
            "default",
            "bottom_7note",
            "middle_2note",
            "middle_6note",
            "middleClose_4note",
            "middleClose_5note",
            "special_3note",
            "top_2note",
            "topFar_4note",
            "topFarCopy_4note",
            "bottom_3note_inner",
            "bottom_3note_outter"
        };
        
        NarwhalDashboard.getInstance().addAutos(autoStrings);
        for (final String auto : autoStrings) {
            if (auto.equals("default")) continue;
            autoMap.put(auto, Trajectories.getPathPlannerAuto(auto));
            autoMap.put(auto + "_red", Trajectories.getPathPlannerAuto(auto + "_red"));
        }
    }

    public Command getAutonomousCommand() {
        String selectedAutoName = NarwhalDashboard.getInstance().getSelectedAuto();
        String hardcode = "bottom_3note_outter";
        
        Command autoCommand;
        if (selectedAutoName == null) {
            selectedAutoName = hardcode;
        }
        else if (selectedAutoName.equals("default")) {
            defaultAuto();
        }
        if (Robot.getAlliance() == Alliance.Red) {
            selectedAutoName += "_red";
        }
        autoCommand = autoMap.get(selectedAutoName);

        return autoCommand.beforeStarting(Trajectories.resetAuto());
    }

    private Command defaultAuto(){
        return none();
        // sequence(
        //         Trajectories.resetAuto(),
        //         waitSeconds(5),
        //         autoShoot(),
        //         waitSeconds(2),
        //         run(()-> Swerve.getInstance().drive(new Translation2d(Robot.getAlliance() == Alliance.Blue ? 1 : -1, 0), 0, true))
        //     );
    }
}