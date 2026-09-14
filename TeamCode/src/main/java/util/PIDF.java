package util;




public class PIDF {
    private double kP;
    private double kI;
    private double kD;
    private double kF;
    private double target;
    private double integral;
    private double previousError;

    public void setCoefficients(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
    }

    public void setTarget(double target) {
        this.target = target;
    }

    public double calculate(double currentPosition) {
        double error = target - currentPosition;
        double pTerm = kP * error;
        integral += error;
        double iTerm = kI * integral;
        double derivative = error - previousError;
        double dTerm = kD * derivative;
        double fTerm = kF * target;
        previousError = error;
        return pTerm + iTerm + dTerm + fTerm;
    }
}
