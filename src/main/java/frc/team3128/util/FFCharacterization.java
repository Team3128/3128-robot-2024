package frc.team3128.util;

import java.util.LinkedList;
import java.util.List;

public class FFCharacterization {

    private final String name;
    private final List<Double> velocityData = new LinkedList<>();
    private final List<Double> voltageData = new LinkedList<>();
    private final List<Double> timeData = new LinkedList<>();
    private final List<Double> accelerationData = new LinkedList<>();
    private PolynomialRegression voltageVelocityRegression;
    private PolynomialRegression velocityTimeRegression;
    private PolynomialRegression voltageAccelerationRegression;


    public FFCharacterization(String name) {
      this.name = name;
    }

    public void add(double time, double velocity, double voltage) {
      if (Math.abs(velocity) > 1E-4) {
        velocityData.add(Math.abs(velocity));
        voltageData.add(Math.abs(voltage));
        timeData.add(time);
      }
    }

    public void print() {
      if (velocityData.size() == 0 || voltageData.size() == 0) {
        return;
      }

      //calculates regression function for voltage v velocity
      voltageVelocityRegression = new PolynomialRegression(
        velocityData.stream().mapToDouble(Double::doubleValue).toArray(),
        voltageData.stream().mapToDouble(Double::doubleValue).toArray(),
        1
      );

      //calculates regression fucnction for velocity v time
      velocityTimeRegression = new PolynomialRegression(
        timeData.stream().mapToDouble(Double::doubleValue).toArray(),
        velocityData.stream().mapToDouble(Double::doubleValue).toArray(),
        4
      );

      //calculates acceleration data from derivative of velocity v time regression function
      accelerationData.addAll(
        new PolynomialDerivative(velocityTimeRegression).evaluate(timeData)
      );

      //calculates regression function for voltage v acceleration
      voltageAccelerationRegression = new PolynomialRegression(
        accelerationData.stream().mapToDouble(Double::doubleValue).toArray(),
        voltageData.stream().mapToDouble(Double::doubleValue).toArray(),
        1
      );

      System.out.println("FF Characterization Results (" + name + "):");
      System.out.println(
        "\tCount=" + Integer.toString(velocityData.size()) + ""
      );
      System.out.println(String.format("\tR2=%.5f", voltageVelocityRegression.R2()));            //R2
      System.out.println(String.format("\tkS=%.5f", voltageVelocityRegression.beta(0)));       //ks
      System.out.println(String.format("\tkV=%.5f", voltageVelocityRegression.beta(1)));       //kv
      System.out.println(String.format("\tkA=%.5f", voltageAccelerationRegression.beta(1)));   //ka
    }
  }
