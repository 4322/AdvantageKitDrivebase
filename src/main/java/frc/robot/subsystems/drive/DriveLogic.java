package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.DriveConstants;

public class DriveLogic {
  
  public static SwerveModuleState[] calcModuleStates(double driveX, double driveY, double rotate,
      Translation2d centerOfRotation, Rotation2d robotAngle, SwerveDriveKinematics kinematics) {
    var swerveModuleStates = kinematics.toSwerveModuleStates(
        ChassisSpeeds.fromFieldRelativeSpeeds(driveX, driveY, rotate, robotAngle),
        centerOfRotation);
    return swerveModuleStates;
  }

  // do not need to return anything since desaturateWheelSpeeds mutates the object itself
  public static void desaturateModuleStates(SwerveModuleState[] states) {
    SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.maxSpeedMetersPerSecond);
  }

  // reduce rotation power when driving fast to not lose forward momentum
  public static double detMaxAutoRotate(boolean robotOverSlowRotateFtPerSec) {
    if (robotOverSlowRotateFtPerSec) {
      return DriveConstants.Auto.slowAutoRotatePower;
    } else {
      return DriveConstants.Auto.maxAutoRotatePower;
    }
  }

  // greater percision when lining up for something
  public static double detMinAutoRotate(boolean robotMoving) {
    if (robotMoving) {
      return DriveConstants.Auto.minAutoRotateMovingPower;
    } else {
      return DriveConstants.Auto.minAutoRotateStoppedPower;
    }
  }

  // greater percision when lining up for something
  public static double detToleranceDeg(boolean robotMoving) {
    if (robotMoving) {
      return DriveConstants.Auto.rotateMovingToleranceDegrees;
    } else {
      return DriveConstants.Auto.rotateStoppedToleranceDegrees;
    }
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

  // Only works if you have exactly 4 swerve modules, but why would you not?
  public static Translation2d avgModuleVectors(double[] moduleAngles, double[] moduleMagnitudes) {
    Translation2d vectorsXY = new Translation2d();

    // sum wheel velocity vectors
    for (int i = 0; i < 4; i++) {
      vectorsXY = vectorsXY.plus(new Translation2d(moduleMagnitudes[i],
          Rotation2d.fromDegrees(moduleAngles[i])));
    }
    
    return vectorsXY;
  }
}
