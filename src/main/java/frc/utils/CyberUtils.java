package frc.utils;

/**
 * Contains basic functions that are used often.
 */
public final class CyberUtils {
    /** Prevent this class from being instantiated. */
    private CyberUtils() {
    }
    
    public static double deadband(double signal, double deadband) {
        if (Math.abs(signal) > Math.abs(deadband)) {
            return signal;
        }
        return 0.0;
    }
}
