package frc.team3128.autonomous;

import edu.wpi.first.wpilibj2.command.Command;

import common.utility.narwhaldashboard.NarwhalDashboard;

/**
 * Class to store information about autonomous routines.
 * @author Daniel Wang, Mason Lam
 */

public class AutoPrograms {

    public AutoPrograms() {

        Trajectories.initTrajectories();
        initAutoSelector();
    }

    private void initAutoSelector() {
        final String[] autoStrings = new String[] {
            "lady-amp_3note",
            "lady-amp_4note",
            "lady-amp_5note",
            "lady-middle_1note_leave",
            "lady-middle_2note",
            "lady-source_1note_leave",
            "lady-source_3note",
            "lady-source_leave",
            "ram-amp_3note",
            "ram-amp_4note",
            "ram-middle_1note_leave",
            "ram-middle_2note",
            "ram-source_3note",
            "ram-source_leave"
        };
        NarwhalDashboard.getInstance().addAutos(autoStrings);
    }

    public Command getAutonomousCommand() {
        String selectedAutoName = NarwhalDashboard.getInstance().getSelectedAuto();
        // String selectedAutoName = "Test";
        final Command autoCommand = Trajectories.getPathPlannerAuto(selectedAutoName);

        return autoCommand.beforeStarting(Trajectories.resetAuto());
    }
}