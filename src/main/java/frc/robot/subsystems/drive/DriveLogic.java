package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class DriveLogic {
  
  public static SwerveModuleState[] calcModuleStates(double driveX, double driveY, double rotate,
      Translation2d centerOfRotation, Rotation2d robotAngle, SwerveDriveKinematics kinematics,
      double maxSpeedMetersPerSecond) {
    var swerveModuleStates = kinematics.toSwerveModuleStates(
        ChassisSpeeds.fromFieldRelativeSpeeds(driveX, driveY, rotate, robotAngle),
        centerOfRotation);
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, maxSpeedMetersPerSecond);
    return swerveModuleStates;
  }

}
