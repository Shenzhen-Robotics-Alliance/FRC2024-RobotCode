package frc.robot.Utils.MathUtils;

public class Rotation2D extends Transformation2D {
    private final double radian;
    public Rotation2D(double radian) {
        super();
        radian = AngleUtils.simplifyAngle(radian);
        double[] iHat = { Math.cos(radian), Math.sin(radian) };
        double[] jHat = { Math.cos(radian + Math.PI / 2), Math.sin(radian + Math.PI / 2) };
        super.setIHat(iHat);
        super.setJHat(jHat);

        this.radian =radian;
    }

    public double getRadian() {
        return radian;
    }

    @Override
    public String toString() {
        return "rotation with radian: " + this.getRadian() + "\nand vector value: " +  super.toString();
    }
}

