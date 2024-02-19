package frc.team3128.autonomous;

import edu.wpi.first.wpilibj2.command.Command;

import common.utility.narwhaldashboard.NarwhalDashboard;
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
            "bottom_2note",
            "bottom_3note_mid",
            "bottom_4note",
            "leave",
            "middle_2note",
            "middle_4note",
            "top_2note",
            "top_4note",
            "topRush_4note",
            "top_5note"
        };
        
        NarwhalDashboard.getInstance().addAutos(autoStrings);
        for (final String auto : autoStrings) {
            autoMap.put(auto, Trajectories.getPathPlannerAuto(auto));
        }
    }

    public Command getAutonomousCommand() {
        String selectedAutoName = NarwhalDashboard.getInstance().getSelectedAuto();
        // String selectedAutoName = "Test";
        final Command autoCommand = autoMap.get(selectedAutoName);

        return autoCommand.beforeStarting(Trajectories.resetAuto());
    }
}