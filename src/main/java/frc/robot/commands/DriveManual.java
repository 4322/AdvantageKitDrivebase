package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.Drive;
import frc.utility.OrangeMath;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveConstants.Manual;
import frc.robot.RobotContainer;

public class DriveManual extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  /**
   * Creates a new Drive_Manual.
   *
   * @param subsystem The subsystem used by this command.
   */

  private final Drive drive;
  private final AutoPose autoPose;
  private boolean done;



  public enum AutoPose {
    none, usePreset, usePresetAuto, usePresetManual, usePresetNoArmMove, loadSingleManual
  }


  public DriveManual(Drive drivesubsystem, AutoPose autoPose) {
    drive = drivesubsystem;
    this.autoPose = autoPose;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.resetRotatePID();
  }

  
  @Override
  public void execute() {
    if (Constants.joysticksEnabled) {

      // Joystick polarity:
      // Positive X is to the right
      // Positive Y is down
      // Positive Z is CW

      // WPI uses a trigonometric coordinate system with the front of the robot
      // pointing toward positive X. Thus:
      // Positive X is forward
      // Positive Y is to the left
      // Positive angles are CCW
      // Angles have a range of +/- 180 degrees (need to verify this)

      // All variables in this program use WPI coordinates
      // All "theta" variables are in radians

      // Dual driver inputs need to be processed in an additive manner
      // instead of being averaged to avoid discontinuities.

      // Cache hardware status for consistency in logic and convert
      // joystick/Xbox coordinates to WPI coordinates.
      final double driveRawX = -RobotContainer.driveStick.getY();
      final double driveRawY = -RobotContainer.driveStick.getX();
      final double rotateRaw = -RobotContainer.rotateStick.getZ();

      // Deadbands are dependent on the type of input device
      final double driveDeadband = Manual.joystickDriveDeadband;
      final double rotateLeftDeadband = Manual.joystickRotateLeftDeadband;
      final double rotateRightDeadband = Manual.joystickRotateRightDeadband;

      // Convert raw drive inputs to polar coordinates for more precise deadband
      // correction
      final double driveRawMag = OrangeMath.pythag(driveRawX, driveRawY);
      final double driveRawTheta = Math.atan2(driveRawY, driveRawX);


      // Normalize the drive input over deadband in polar coordinates.
      double driveMag = 0;
      if (driveRawMag > driveDeadband) {
        driveMag = (driveRawMag - driveDeadband) / (1 - driveDeadband);

        if (Constants.driveTuningMode) {
          // quantize input drive magnitude to 0, 0.25, 0.5, 0.75, 1.0 for PID tuning
          driveMag = Math.round(driveMag * 4.0) / 4.0;
        }
      }
      // Convert back to cartesian coordinates
      double driveX = Math.cos(driveRawTheta) * driveMag;
      double driveY = Math.sin(driveRawTheta) * driveMag;
      // Normalize the combined drive vector
      if (driveX > 1) {
        driveY /= driveX;
        driveX = 1;
      } else if (driveX < -1) {
        driveY /= -driveX;
        driveX = -1;
      }
      if (driveY > 1) {
        driveX /= driveY;
        driveY = 1; 
      } else if (driveY < -1) {
        driveX /= -driveY;
        driveY = -1;
      }

      // Normalize the rotation inputs over deadband.
      double rotatePower = 0;
      if (rotateRaw > rotateLeftDeadband) {
        rotatePower = (rotateRaw - rotateLeftDeadband) / (1 - rotateLeftDeadband);
      } else if (rotateRaw < -rotateRightDeadband) {
        rotatePower = (rotateRaw + rotateRightDeadband) / (1 - rotateRightDeadband);
      }
      rotatePower = rotatePower * Manual.manualRotationScaleFromMax;

      drive.drive(driveX, driveY, rotatePower);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

