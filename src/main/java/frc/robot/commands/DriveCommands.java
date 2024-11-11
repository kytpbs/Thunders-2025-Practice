package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.DriveSubsystem;

public class DriveCommands {
  public static Command driveWithController(
      DriveSubsystem driveSubsystem, XboxController controller) {
    return driveSubsystem.drive(
        () ->
            addBoost(
                -controller.getLeftY(), // inverted because the controller is inverted
                getBoostByValue(controller)),
        () ->
            addBoost(
                -controller.getRightX(), // ! THIS IS INVERTED BECAUSE +y IS THE LEFT OF ROBOT
                getBoostByValue(controller)));
  }

  private static double addBoost(double value, double boost) {
    /*
     * we wish to add the DriveSensitivity amount of the value as boost
     * so calculation is:
     *
     * (1 - boost) * value + boost * value
     *
     * when we factor for "value" we get:
     *
     * value * ( driveSensitivity * (1 - boost) +  boost)
     *
     */
    final var boostMultiplier = OperatorConstants.kDriveSensitivity * (1 - boost) + boost;
    return MathUtil.applyDeadband(value * boostMultiplier, OperatorConstants.kDriveDeadband);
  }

  /**
   * Get the boost value for the drive command Boosts According to the shoulder analog inputs
   * doesn't go less than 0.2 for the convince of the driver
   *
   * @param controller
   * @return a double value between {@link OperatorConstants#kMinBoost} and {@link
   *     OperatorConstants#kMaxBoost}
   */
  private static double getBoostByValue(XboxController controller) {
    var boostBy = controller.getRightTriggerAxis();
    var reduceAmount = controller.getLeftTriggerAxis(); // between 0 and 1

    double totalBoost = boostBy - reduceAmount;

    totalBoost =
        MathUtil.clamp(totalBoost, OperatorConstants.kMinBoost, OperatorConstants.kMaxBoost);
    return totalBoost;
  }
}
