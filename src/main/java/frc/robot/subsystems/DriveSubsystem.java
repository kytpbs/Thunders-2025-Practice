package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import java.util.function.DoubleSupplier;

public class DriveSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private final WPI_VictorSPX leftMotor = new WPI_VictorSPX(DriveConstants.kLeftMotorPort);

  private final VictorSPX leftMotorFollower = new VictorSPX(DriveConstants.kLeftMotorFollowerPort);
  private final WPI_VictorSPX rightMotor = new WPI_VictorSPX(DriveConstants.kRightMotorPort);
  private final VictorSPX rightMotorFollower =
      new VictorSPX(DriveConstants.kRightMotorFollowerPort);

  private final DifferentialDrive differentialDrive;

  private Field2d field = new Field2d();

  private DifferentialDrivetrainSim differentialDriveSim =
      DifferentialDrivetrainSim.createKitbotSim(
          KitbotMotor.kDualCIMPerSide, // 2 CIMs per side.
          KitbotGearing.k10p71, // 10.71:1
          KitbotWheelSize.kSixInch, // 6" diameter wheels.
          null // No measurement noise.
          );

  public DriveSubsystem() {
    // Right motor should go forward when given a positive value.
    // so we set it to be inverted.
    rightMotor.setInverted(true);
    // Follow the leaders as from now on, the leftMotor and rightMotor are the leaders
    // and you can control them as if they were a single motor
    leftMotorFollower.follow(leftMotor);
    rightMotorFollower.follow(rightMotor);

    // create a differential drive using the leader motors
    // this will use both motors on each side as a single motor
    // thanks to the followers
    differentialDrive = new DifferentialDrive(leftMotor, rightMotor);

    if (RobotBase.isSimulation()) {
      // we only use field2d when we are in simulation cause we can't calculate our location in real
      // life due to not having encoders or gyro in the practice bot
      SmartDashboard.putData(field);
    }
  }

  public Command stop() {
    return this.runOnce(differentialDrive::stopMotor);
  }

  public Command drive(double forwardSpeed, double rotationSpeed) {
    return this.drive(() -> forwardSpeed, () -> rotationSpeed);
  }

  public Command drive(double forwardSpeed, double rotationSpeed, boolean squareInputs) {
    return this.drive(() -> forwardSpeed, () -> rotationSpeed, squareInputs);
  }

  public Command drive(DoubleSupplier forwardSpeed, DoubleSupplier rotationSpeed) {
    return this.drive(forwardSpeed, rotationSpeed, true);
  }

  public Command drive(
      DoubleSupplier forwardSpeed, DoubleSupplier rotationSpeed, boolean squareInputs) {
    return this.runEnd(
        () ->
            this.driveRobot(forwardSpeed.getAsDouble(), rotationSpeed.getAsDouble(), squareInputs),
        differentialDrive::stopMotor);
  }

  private void driveRobot(double forwardSpeed, double rotationSpeed, boolean squareInputs) {
    differentialDrive.arcadeDrive(forwardSpeed, rotationSpeed, squareInputs);
    if (RobotBase.isSimulation()) {
      SmartDashboard.putNumber("Forward Speed", forwardSpeed);
      SmartDashboard.putNumber("Rotation Speed", rotationSpeed);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    differentialDriveSim.setInputs(
        leftMotor.get() * 12, rightMotor.get() * 12); // Convert to volts.
    differentialDriveSim.update(0.02); // 20ms per update.

    field.setRobotPose(differentialDriveSim.getPose());
  }
}
