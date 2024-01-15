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

public class CmdSysId extends Command {

  private static final double startDelaySecs = 2.0;
  private static final double rampRateVoltsPerSec = 0.05;

  private final boolean forwards;
  private final boolean isDrive;

  private final FFCharacterization dataPrimary;
  private final FFCharacterization dataSecondary;
  private final Consumer<Double> voltageConsumerSimple;
  private final BiConsumer<Double, Double> voltageConsumerDrive;
  private final Supplier<Double> velocitySupplierPrimary;
  private final Supplier<Double> velocitySupplierSecondary;

  private final Timer timer = new Timer();

  /** Creates a new FeedForwardCharacterization for a differential drive. */
  public CmdSysId(
    Subsystem drive,
    boolean forwards,
    FFCharacterization leftData,
    FFCharacterization rightData,
    BiConsumer<Double, Double> voltageConsumer,
    Supplier<Double> leftVelocitySupplier,
    Supplier<Double> rightVelocitySupplier
  ) {
    addRequirements(drive);
    this.forwards = forwards;
    this.isDrive = true;
    this.dataPrimary = leftData;
    this.dataSecondary = rightData;
    this.voltageConsumerSimple = null;
    this.voltageConsumerDrive = voltageConsumer;
    this.velocitySupplierPrimary = leftVelocitySupplier;
    this.velocitySupplierSecondary = rightVelocitySupplier;
  }

  /** Creates a new FeedForwardCharacterization for a simple subsystem. */
  public CmdSysId(
    Subsystem subsystem,
    boolean forwards,
    FFCharacterization data,
    Consumer<Double> voltageConsumer,
    Supplier<Double> velocitySupplier
  ) {
    addRequirements(subsystem);
    this.forwards = forwards;
    this.isDrive = false;
    this.dataPrimary = data;
    this.dataSecondary = null;
    this.voltageConsumerSimple = voltageConsumer;
    this.voltageConsumerDrive = null;
    this.velocitySupplierPrimary = velocitySupplier;
    this.velocitySupplierSecondary = null;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    setVoltage(0.0);
    new WaitCommand(startDelaySecs).schedule();
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (timer.get() < startDelaySecs) return;

    double voltage =
      (timer.get() - startDelaySecs) *
      rampRateVoltsPerSec *
      (forwards ? 1 : -1);
    setVoltage(voltage);
    updateData(voltage);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    setVoltage(0.0);
    timer.stop();
    dataPrimary.print();
    if (isDrive) {
      dataSecondary.print();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  //supplies a voltage to given consumers
  public void setVoltage(double voltage) {
    if (isDrive) {
      voltageConsumerDrive.accept(voltage, voltage);
    } else {
      voltageConsumerSimple.accept(voltage);
    }
  }

  //updates velocity, time, and voltage data
  public void updateData(double voltage) {
    dataPrimary.add(timer.get(), velocitySupplierPrimary.get(), voltage);
    if (isDrive) dataSecondary.add(
      timer.get(),
      velocitySupplierSecondary.get(),
      voltage
    );
  }
}