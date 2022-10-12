package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

import java.util.function.DoubleSupplier;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

public class BallTrackDrive extends CommandBase {
  private final DrivetrainSubsystem m_drivetrainSubsystem;

  private final DoubleSupplier m_translationXSupplier;
  private final DoubleSupplier m_translationYSupplier;
  private final DoubleSupplier m_rotationSupplier;
  private final PhotonCamera camera;

  public BallTrackDrive(DrivetrainSubsystem drivetrainSubsystem,
      DoubleSupplier translationXSupplier,
      DoubleSupplier translationYSupplier,
      DoubleSupplier rotationSupplier,
      PhotonCamera camera) {
    this.m_drivetrainSubsystem = drivetrainSubsystem;
    this.m_translationXSupplier = translationXSupplier;
    this.m_translationYSupplier = translationYSupplier;
    this.m_rotationSupplier = rotationSupplier;
    this.camera = camera;

    addRequirements(drivetrainSubsystem);
  }

  @Override
  public void execute() {
    // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of
    // field-oriented movement
    double rotation = m_rotationSupplier.getAsDouble();
    PhotonPipelineResult result = camera.getLatestResult();
    if (result.hasTargets()) {
      rotation = result.getBestTarget().getYaw() * (1 / 8.0) * -0.5;
      SmartDashboard.putNumber("Turn Force", rotation);
      SmartDashboard.putNumber("Target yaw", result.getBestTarget().getYaw());
    }
    m_drivetrainSubsystem.drive(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            m_translationXSupplier.getAsDouble(),
            m_translationYSupplier.getAsDouble(),
            rotation,
            m_drivetrainSubsystem.getGyroscopeRotation()));
  }

  @Override
  public void end(boolean interrupted) {
    m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
  }
}
