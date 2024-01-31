package frc.robot.Utils.MathUtils;

public class LookUpTable {
    public final double[] xValues;
    public final double[] yValues;
    final int n;
    final double bestFitLineSlope, bestFitLineIntersect;

    public LookUpTable(double[] xValues, double[] yValues) {
        if (xValues.length != yValues.length)
            throw new IllegalArgumentException("look up table length not match");
        double prev_x = -Double.POSITIVE_INFINITY;
        for (double x:xValues) {
            if (x < prev_x)
                throw new IllegalArgumentException("look up table X must be in increasing order");
            prev_x = x;
        }
        this.xValues = xValues;
        this.yValues = yValues;
        this.n = xValues.length;

        this.bestFitLineIntersect = StatisticsUtils.getBestFitLineIntersect(xValues, yValues);
        this.bestFitLineSlope = StatisticsUtils.getBestFitLineSlope(xValues, yValues);
    }

    public double getYPrediction(double x) {
        for (int i = 0; i < n-1; i++)
            if (xValues[i] < x && x < xValues[i+1])
                return linearInterpretation(xValues[i], yValues[i], xValues[i+1], yValues[i+1], x);
        return x * bestFitLineSlope + bestFitLineIntersect; // if no interval satisfies, we use best-fit line instead
    }

    private double linearInterpretation(double x1, double y1, double x2, double y2, double x) {
        return y1 + (x-x1) * (y2-y1) / (x2-x1);
    }
}
