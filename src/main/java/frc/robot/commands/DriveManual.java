package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Telescope;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.LED;
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

  private static boolean scoreAutoPoseActive;
  private static boolean loadAutoPoseActive;
  private static boolean loadAutoAlignPending;
  private static boolean armAtLoadSingle;
  private final Drive drive;
  private final AutoPose autoPose;
  private Double targetHeadingDeg;
  private boolean done;
  private Timer spinoutActivationTimer = new Timer();
  private Timer spinoutActivationTimer2 = new Timer();
  private LockedWheel lockedWheelState;
  private double initialSpinoutAngle;

  private Arm arm = Arm.getInstance();
  private Telescope telescope = Telescope.getInstance();
  private ArmMove armLoadSingleRetract;

  public enum AutoPose {
    none, usePreset, usePresetAuto, usePresetManual, usePresetNoArmMove, loadSingleManual
  }
  
  public enum LockedWheel {
    none, center, frontLeft, backLeft, backRight, frontRight;
  }

  public static boolean isScoreAutoPoseActive() {
    return scoreAutoPoseActive;
  }

  public static boolean isLoadAutoPoseActive() {
    return loadAutoPoseActive;
  }

  public static boolean isLoadAutoAlignReady() {
    return loadAutoAlignPending && armAtLoadSingle;
  }

  // prevent AutoAlignSubstation command from immediately restarting once complete
  public static void loadAutoAlignDone() {
    loadAutoAlignPending = false;
  }

  public DriveManual(Drive drivesubsystem, AutoPose autoPose) {
    drive = drivesubsystem;
    this.autoPose = autoPose;
    armLoadSingleRetract = new ArmMove(arm, telescope, ArmMove.Position.loadSingleRetract, false);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    // make command reusable
    spinoutActivationTimer.stop();
    spinoutActivationTimer2.stop();
    spinoutActivationTimer.reset();
    spinoutActivationTimer2.reset();
    lockedWheelState = LockedWheel.none;
    done = false;

    drive.resetRotatePID();
    scoreAutoPoseActive = false;
    loadAutoPoseActive = false;
    loadAutoAlignPending = false;
    armAtLoadSingle = false;

    // set auto-rotate direction, if any
    switch (autoPose) {
      case none:
        targetHeadingDeg = null;
        if (!ArmMove.isAtLoadSingleRetract()) {
          LED.getInstance().setAlignment(LED.Alignment.none);
        }
        break;
      case loadSingleManual:
        ArmMove.setArmPreset(ArmMove.Position.loadSingleExtend);
        initSubstationAlignment();
        break;
      case usePresetAuto:
        if (ArmMove.getArmPreset() == ArmMove.Position.loadSingleExtend) {
          LED.getInstance().setAutoAccepted();
        }
        // fall through
      case usePreset:
        autoRotateSetTarget(true);
        break;
      case usePresetManual:
      case usePresetNoArmMove:
        autoRotateSetTarget(false);
        break;
    }
  }

  // autoAlignMode is set to true when we want to invoke autoAlignment
  private void autoRotateSetTarget(boolean autoAlignMode) {
    switch (ArmMove.getArmPreset()) {
      case scoreLow:
        targetHeadingDeg = 180.0;
        scoreAutoPoseActive = true;
        LED.getInstance().setAlignment(LED.Alignment.none);
        break;
      case scoreMid:
      case scoreHigh:
        targetHeadingDeg = 0.0;
        scoreAutoPoseActive = true;
        LED.getInstance().setAlignment(LED.Alignment.grid);
        break;
      case loadSingleExtend:
        initSubstationAlignment();
        if (targetHeadingDeg != null) {
          loadAutoAlignPending = autoAlignMode;
        }
        break;
      default:
        // not an auto-rotate preset
        targetHeadingDeg = null;
        done = true;
        LED.getInstance().setAlignment(LED.Alignment.none);
        break;
    }
  }
  
  private void initSubstationAlignment() {
    switch (Robot.getAllianceColor()) {
      case Blue:
        targetHeadingDeg = 90.0;
        loadAutoPoseActive = true;
        LED.getInstance().setAlignment(LED.Alignment.substation);
        break;
      case Red:
        targetHeadingDeg = -90.0;
        loadAutoPoseActive = true;
        LED.getInstance().setAlignment(LED.Alignment.substation);
        break;
      default:
        // unknown direction to single substation
        targetHeadingDeg = null;
        done = true;
        LED.getInstance().setAlignment(LED.Alignment.none);
        break;
    }
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

      final double driveAngle = drive.getAngle();

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

      // if the rotate stick isn't being used
      if (rotatePower == 0) {
        // if there is a set drive auto rotate
        if (targetHeadingDeg != null) {
          drive.driveAutoRotate(driveX, driveY, targetHeadingDeg);
          if (loadAutoPoseActive && !armAtLoadSingle && ((autoPose == AutoPose.usePreset)
              || (autoPose == AutoPose.usePresetAuto) || (autoPose == AutoPose.usePresetManual))) {
            if (driveAngle >= targetHeadingDeg - Constants.AutoAlignSubstationConstants.rotateToleranceDegrees && 
                driveAngle <= targetHeadingDeg + Constants.AutoAlignSubstationConstants.rotateToleranceDegrees) {
              armLoadSingleRetract.schedule();
              armAtLoadSingle = true;
            }
          }
          return;
        } else if (Constants.driveDegradedMode == Constants.DriveDegradedMode.normal) {
          // set pseudo auto rotate heading
          targetHeadingDeg = driveAngle;
          drive.driveAutoRotate(driveX, driveY, targetHeadingDeg);
          return;
        }
      } else {  
        // rotate joystick is active
        // check if we are in the default drive manual
        if (autoPose == AutoPose.none) {
          targetHeadingDeg = null; // unlock auto rotate heading
        } else {
          // restart default driveManual command
          drive.drive(driveX, driveY, rotatePower);
          done = true;
          return;
        }
      }

      // cache value for logic consistency
      double absAngularVelocity = Math.abs(drive.getAngularVelocity());

      if (Math.abs(rotateRaw) >= Manual.spinoutRotateDeadBand) {
        if (absAngularVelocity < Manual.spinoutMinAngularVelocity) {
          spinoutActivationTimer.start();
        } else {
          spinoutActivationTimer.stop();
          spinoutActivationTimer.reset();
        }
        if (absAngularVelocity < Manual.spinoutMinAngularVelocity2) {
          spinoutActivationTimer2.start();
        } else {
          spinoutActivationTimer2.stop();
          spinoutActivationTimer2.reset();
        }
      } else {
        // if rotation stick falls under second deadband, reset rotation back to normal
        lockedWheelState = LockedWheel.none;
        spinoutActivationTimer.stop();
        spinoutActivationTimer2.stop();
        spinoutActivationTimer.reset();
        spinoutActivationTimer2.reset();
      }

      // detect if not rotating and if rotate stick past second deadband for certain
      // amount of time
      // (first deadband is rotateToleranceDegrees/xboxRotateDeadband)
      // (second deadband is past first deadband in rotation) (close to max rotation)
      if ((lockedWheelState == LockedWheel.none)
          && (spinoutActivationTimer.hasElapsed(Manual.spinoutActivationSec)
              || spinoutActivationTimer2.hasElapsed(Manual.spinout2ActivationSec))) {

        // from this, figure out which swerve module to lock onto to rotate off of (use
        // drive
        // stick direction and robotAngle)
        // How to use drive stick: module closest to direction of drivestick.
        // use gyro to find orientation
        // algorithm to determine quadrant: driveStickAngle - robotAngle
        // if drivestick angle 0 < x < 90 , in quadrant 1 (front left module)
        // if drivestick angle 90 < x < 180 , in quadrant 2 (back left module)
        // if drivestick angle -180 < x < -90 , in quadrant 3 (back right module)
        // if drivestick angle -90 < x < 0 , in quadrant 4 (front right module)

        // SPECIAL CASE: if driveStickAngle - robotAngle is exactly 0, 90, 180, -180,
        // then use the
        // rotate angle to determine wheel:
        // 0: if CW, quadrant 1 (front left); if CCW, quadrant 4 (front right)
        // 90: if CW, quadrant 2 (back left); if CCW, quadrant 1 (front left)
        // 180/-180: if CW, quadrant 3 (back right); if CCW, quadrant 2 (back left)
        // -90: if CW, quadrant 4 (front right); if CCW, quadrant 3 (back right)

        // drivestick angle - robot angle
        double robotCentricDriveTheta =
            OrangeMath.boundDegrees(Math.toDegrees(Math.atan2(driveY, driveX)) - driveAngle);
        initialSpinoutAngle = driveAngle;

        if (Constants.spinoutCenterEnabled && (driveX == 0) && (driveY == 0)) {
          lockedWheelState = LockedWheel.center;
        } else if (Constants.spinoutCornerEnabled) {
          if ((robotCentricDriveTheta > 0) && (robotCentricDriveTheta < 90)) {
            lockedWheelState = LockedWheel.frontLeft;
          } else if ((robotCentricDriveTheta > 90) && (robotCentricDriveTheta < 180)) {
            lockedWheelState = LockedWheel.backLeft;
          } else if ((robotCentricDriveTheta > -180) && (robotCentricDriveTheta < -90)) {
            lockedWheelState = LockedWheel.backRight;
          } else if ((robotCentricDriveTheta > -90) && (robotCentricDriveTheta < 0)) {
            lockedWheelState = LockedWheel.frontRight;
          }
        }

        // if robot rotates 90 degrees, reset rotation back to normal
      } else if ((Math.abs(initialSpinoutAngle - driveAngle) >= 90)
          && (lockedWheelState != LockedWheel.none)) {
        lockedWheelState = LockedWheel.none;
        spinoutActivationTimer.stop();
        spinoutActivationTimer2.stop();
        spinoutActivationTimer.reset();
        spinoutActivationTimer2.reset();
      }
      // use state machine for rotating each wheel in each direction (8 cases)
      // each module rotating CW and CCW
      double spinCornerPower = Math.copySign(DriveConstants.spinoutCornerPower, rotatePower);
      switch (lockedWheelState) {
        case none:
          drive.drive(driveX, driveY, rotatePower);
          break;
        case center:
          drive.drive(driveX, driveY, Math.copySign(DriveConstants.spinoutCenterPower, rotatePower));
          break;
        case frontLeft:
          drive.drive(driveX, driveY, spinCornerPower,
              DriveConstants.frontLeftWheelLocation);
          break;
        case backLeft:
          drive.drive(driveX, driveY, spinCornerPower,
              DriveConstants.backLeftWheelLocation);
          break;
        case backRight:
          drive.drive(driveX, driveY, spinCornerPower,
              DriveConstants.backRightWheelLocation);
          break;
        case frontRight:
          drive.drive(driveX, driveY, spinCornerPower,
              DriveConstants.frontRightWheelLocation);
          break;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}
