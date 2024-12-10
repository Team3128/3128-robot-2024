package frc.team3128.subsystems.Amper;

public enum AmperStates {
    EXTENDED(10, 5500),
    PRIMED(10 * 0.7, 5500),
    IDLE(0, 0);

    private double elevatorSetpoint;
    private double rollerSetpoint;

    private AmperStates(double elevatorSetpoint, double rollerSetpoint) {
        this.elevatorSetpoint = elevatorSetpoint;
        this.rollerSetpoint = rollerSetpoint;
    }

    public double getElevatorSetpoint() {
        return elevatorSetpoint;
    }

    public double getRollerSetpoint() {
        return rollerSetpoint;
    }
}