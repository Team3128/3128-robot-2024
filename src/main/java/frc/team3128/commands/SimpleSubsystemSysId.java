package frc.team3128.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team3128.util.FFCharacterization;
import frc.team3128.util.PolynomialDerivative;
import frc.team3128.util.PolynomialRegression;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.function.BiConsumer;
import java.util.function.Consumer;
import java.util.function.Supplier;

import common.utility.shuffleboard.NAR_Shuffleboard;


public class SimpleSubsystemSysId extends Command{
    private static final double startDelaySecs = 2.0;
  private static final double rampRateVoltsPerSec = 0.02;

  private final boolean forwards;

  private final FFCharacterization data;
  private final Consumer<Double> voltageConsumer;
  private final Supplier<Double> velocitySupplier;
  private final Supplier<Double> positionSupplier;

  private final double upperPositionConstraint;
  private final double lowerPositionConstraint;

  private double currVoltageInput;

  private final Timer timer = new Timer();

  /** Creates a new FeedForwardCharacterization for a simple subsystem. */

  public SimpleSubsystemSysId(
    Subsystem subsystem,
    boolean forwards,
    FFCharacterization data,
    Consumer<Double> voltageConsumer,
    Supplier<Double> velocitySupplier,
    Supplier<Double> positionSupplier,
    double upperPositionConstraint,
    double lowerPositionConstraint
    ) {
    addRequirements(subsystem);
    this.forwards = forwards;
    this.data = data;
    this.voltageConsumer = voltageConsumer;
    this.velocitySupplier = velocitySupplier;
    this.positionSupplier = positionSupplier;
    this.upperPositionConstraint = upperPositionConstraint;
    this.lowerPositionConstraint = lowerPositionConstraint;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    setVoltage(0.0);
    new WaitCommand(startDelaySecs).schedule();
    initShuffleboard();
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (timer.get() < startDelaySecs) return;

    double voltage = (timer.get() - startDelaySecs) * rampRateVoltsPerSec * (forwards ? 1 : -1);
    setVoltage(voltage);
    updateData(voltage);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    setVoltage(0.0);
    timer.stop();
    data.print();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double position = positionSupplier.get();
    return position >= upperPositionConstraint || position <= lowerPositionConstraint || currVoltageInput >= 12.0;
  }

  //supplies a voltage to given consumers
  public void setVoltage(double voltage) {
    currVoltageInput = voltage;
    voltageConsumer.accept(voltage);
  }

  //updates velocity, time, and voltage data
  public void updateData(double voltage) {
    data.add(timer.get(), velocitySupplier.get(), voltage);
  }

  public void initShuffleboard(){
    NAR_Shuffleboard.addData("SysID", "Position", positionSupplier, 0, 0);
    NAR_Shuffleboard.addData("SysID", "Velocity", velocitySupplier, 0, 1);
    NAR_Shuffleboard.addData("SysID", "Voltage", ()-> currVoltageInput, 0, 2);
    
  }
}
