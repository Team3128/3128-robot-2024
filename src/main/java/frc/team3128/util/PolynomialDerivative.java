package frc.team3128.util;

import java.util.ArrayList;
import java.util.List;

public class PolynomialDerivative {

    private double[] functionCoefficients;
    private double[] derivativeCoefficients;

    /**
     * Creates a new PolynomialDerivative object
     * @param function the polynomial regression function to be differentiated
     */
    public PolynomialDerivative(PolynomialRegression function){

      this.functionCoefficients = new double[function.degree() + 1];  //plus 1 is to inlcude for 0th degree
      this.derivativeCoefficients = new double[functionCoefficients.length - 1]; //minus 1 is to remove the 0th degree

      //load function coefficients
      for(int i = 0; i <= function.degree(); i++){
        functionCoefficients[i] = function.beta(i);
      }

      //calculate derivative coefficients
      for (int i = 1; i < functionCoefficients.length; i++) {
        derivativeCoefficients[i - 1] = functionCoefficients[i] * i;
      }
    }

    //returns the derivative of the function at x
    public double evaluate(double x){
      double y = 0;
      for(int i = 0; i < derivativeCoefficients.length; i++){
        y += derivativeCoefficients[i] * Math.pow(x, i);
      }
      return y;
    }

    //returns the derivative of the function at each x value in the list
    public List<Double> evaluate(List<Double> inputs){
      List<Double> output = new ArrayList<Double>();
      for(Double xVal : inputs){
        output.add(evaluate(xVal));
      }
      return output;
    }
  }
