package frc.robot;

import edu.wpi.first.wpiutil.math.MathUtil;

public class UtilEverything {

    /**
     * Map value to set range.
     *
     * @param value         - Value to map.
     * @param minimumInput  - Minimum input in input range.
     * @param maximumInput  - Maximum input in input range.
     * @param minimumOutput - Minimum output int output range.
     * @param maximumOutput - maximum output in output range.
     * @return mapped value.
     */
    public static double map(double value, double minimumInput, double maximumInput, double minimumOutput, double maximumOutput) {
        return (value - minimumInput) * (maximumOutput - minimumOutput) / (maximumInput - minimumInput) + minimumOutput;
    }

    /**
     * Convert all negative angles to positive.
     *
     * @param angle - angle to convert.
     * @return converted angle.
     */
    public static double toPositive(double angle) {
        if (angle < 0) return angle + 360;
        return angle;
    }

    /**
     * Convert angles from POV like representation to unit circle.
     *
     * @param angle - angle to convert.
     * @return converted angle.
     */
    public static double fromPOVtoUnitCircle(double angle) {
        angle %= 360;
        if (angle > 0 && angle < 90) {
            return 90 - angle;
        }
        if (angle > 90 && angle < 180) {
            return 360 - (angle - 90);
        }
        if (angle > 180 && angle < 270) {
            return 180 + (270 - angle);
        }
        if (angle > 270 && angle < 360) {
            return 90 + (360 - angle);
        }
        if (angle == 0 || angle == 180) {
            return angle + 90;
        }
        return angle - 90;
    }

    /**
     * Convert angles from (-180) - 180 to unit circle.
     *
     * @param angle - angle to convert.
     * @return converted angle.
     */
    public static double fromMinus180Plus180(double angle) {
        angle = toPositive(angle);
        return fromPOVtoUnitCircle(angle);
    }


    /**
     * Convert angles from unit circle to (-180) - 180.
     *
     * @param angle - angle to convert.
     * @return converted angle.
     */
    public static double toMinus180Plus180(double angle) {
        return MathUtil.inputModulus(fromPOVtoUnitCircle(angle), -180, 180);
    }
}
