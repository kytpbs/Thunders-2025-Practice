package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final double kDriveSensitivity = 0.5;
    public static final double kDriveDeadband = 0.02;
    public static final double kMinBoost = -0.7;
    public static final double kMaxBoost = 1;
  }

  public static class DriveConstants {
    // TODO: Update CAN IDs
    public static final int kLeftMotorPort = 1;
    public static final int kLeftMotorFollowerPort = 2;
    public static final int kRightMotorPort = 3;
    public static final int kRightMotorFollowerPort = 4;
  }
}
