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

  public static double boundRotatePID(double headingChangeDeg, double toleranceDeg, double rotPIDSpeed, double minRotPower, double maxRotPower) {
    if (Math.abs(headingChangeDeg) <= toleranceDeg) {
      rotPIDSpeed = 0; // don't wiggle
    } else if (Math.abs(rotPIDSpeed) < minRotPower) {
      rotPIDSpeed = Math.copySign(minRotPower, rotPIDSpeed);
    } else if (rotPIDSpeed > maxRotPower) {
      rotPIDSpeed = maxRotPower;
    } else if (rotPIDSpeed < -maxRotPower) {
      rotPIDSpeed = -maxRotPower;
    }
    return rotPIDSpeed;
  }

}
