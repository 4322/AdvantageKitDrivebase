package frc.robot.subsystems.drive;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.SwerveModule;
import frc.robot.subsystems.drive.GyroIO.GyroIOInputs;
import frc.robot.Constants.WheelPosition;
import frc.utility.OrangeMath;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.Timer;

public class Drive extends SubsystemBase {

  private SwerveModule[] swerveModules = new SwerveModule[4];

  private GyroIO gyro;
  private GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

  private PIDController rotPID;

  private Timer runTime = new Timer();

  private double latestVelocity;
  private double latestAcceleration;
  private double pitchOffset;

  private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
      DriveConstants.frontRightWheelLocation, DriveConstants.frontLeftWheelLocation,
      DriveConstants.backLeftWheelLocation, DriveConstants.backRightWheelLocation);

  private SwerveDriveOdometry odometry;
  private ShuffleboardTab tab;

  private GenericEntry rotErrorTab;
  private GenericEntry rotSpeedTab;
  private GenericEntry rotkP;
  private GenericEntry rotkD;
  private GenericEntry yawTab;
  private GenericEntry rollTab;
  private GenericEntry pitchTab;
  private GenericEntry botVelocityMag;
  private GenericEntry botAccelerationMag;
  private GenericEntry botVelocityAngle;
  private GenericEntry botAccelerationAngle;
  private GenericEntry driveXTab;
  private GenericEntry driveYTab;
  private GenericEntry rotateTab;
  private GenericEntry odometryX;
  private GenericEntry odometryY;
  private GenericEntry odometryDegrees;
  private GenericEntry angularVel;

  private GenericEntry rampRate;
  private double lastRampRate;

  public Drive() {
    runTime.start();
    switch (Constants.currentMode) {
      // Real robot, instantiate hardware IO implementations
      case REAL:
        if (Constants.driveEnabled) {
          swerveModules[WheelPosition.FRONT_RIGHT.wheelNumber] =
              new SwerveModule(WheelPosition.FRONT_RIGHT, new SwerveModuleIOMotorControl(WheelPosition.FRONT_RIGHT));
          swerveModules[WheelPosition.FRONT_LEFT.wheelNumber] =
              new SwerveModule(WheelPosition.FRONT_LEFT, new SwerveModuleIOMotorControl(WheelPosition.FRONT_LEFT));
          swerveModules[WheelPosition.BACK_RIGHT.wheelNumber] =
              new SwerveModule(WheelPosition.BACK_RIGHT, new SwerveModuleIOMotorControl(WheelPosition.BACK_RIGHT));
          swerveModules[WheelPosition.BACK_LEFT.wheelNumber] =
              new SwerveModule(WheelPosition.BACK_LEFT, new SwerveModuleIOMotorControl(WheelPosition.BACK_LEFT));
        }
        if (Constants.gyroEnabled) {
          gyro = new GyroIONavX();
        }
        break;

      // Sim robot, instantiate physics sim IO implementations
      case SIM:
        break;

      // Replayed robot, disable hardware IO implementations
      case REPLAY:
        break;
    }

    if (swerveModules[WheelPosition.FRONT_RIGHT.wheelNumber] == null) {
      swerveModules[WheelPosition.FRONT_RIGHT.wheelNumber] =
          new SwerveModule(WheelPosition.FRONT_RIGHT, new SwerveModuleIO() {});
      swerveModules[WheelPosition.FRONT_LEFT.wheelNumber] =
          new SwerveModule(WheelPosition.FRONT_LEFT, new SwerveModuleIO() {});
      swerveModules[WheelPosition.BACK_RIGHT.wheelNumber] =
          new SwerveModule(WheelPosition.BACK_RIGHT, new SwerveModuleIO() {});
      swerveModules[WheelPosition.BACK_LEFT.wheelNumber] =
          new SwerveModule(WheelPosition.BACK_LEFT, new SwerveModuleIO() {});
    }

    if (gyro == null) {
      gyro = new GyroIO() {};
    }
  }

  public void init() {
    if (Constants.driveEnabled) {
      rotPID = new PIDController(DriveConstants.Auto.autoRotkP, 0, DriveConstants.Auto.autoRotkD);

      if (Constants.gyroEnabled) {  
        // wait for first gyro reading to be received
        try {
          Thread.sleep(2000);
        } catch (InterruptedException e) {
        }  
        odometry = new SwerveDriveOdometry(kinematics, getRotation2d(), getModulePostitions());
        resetFieldCentric(0);
      }

    if (Constants.debug) {
        tab = Shuffleboard.getTab("Drivebase");

        rotErrorTab = tab.add("Rot Error", 0).withPosition(0, 0).withSize(1, 1).getEntry();

        rotSpeedTab = tab.add("Rotation Speed", 0).withPosition(0, 1).withSize(1, 1).getEntry();

        rotkP = tab.add("Rotation kP", DriveConstants.Auto.autoRotkP).withPosition(1, 0)
            .withSize(1, 1).getEntry();

        rotkD = tab.add("Rotation kD", DriveConstants.Auto.autoRotkD).withPosition(2, 0)
            .withSize(1, 1).getEntry();

        yawTab = tab.add("Yaw", 0).withPosition(0, 3).withSize(1, 1).getEntry();

        rollTab = tab.add("Roll", 0).withPosition(1, 1).withSize(1, 1).getEntry();

        pitchTab = tab.add("Pitch", 0).withPosition(2, 1).withSize(1, 1).getEntry();

        botVelocityMag = tab.add("Bot Vel Mag", 0).withPosition(3, 0).withSize(1, 1).getEntry();

        botAccelerationMag = tab.add("Bot Acc Mag", 0).withPosition(3, 1).withSize(1, 1).getEntry();

        botVelocityAngle = tab.add("Bot Vel Angle", 0).withPosition(4, 0).withSize(1, 1).getEntry();

        botAccelerationAngle =
            tab.add("Bot Acc Angle", 0).withPosition(4, 1).withSize(1, 1).getEntry();

        angularVel = tab.add("Angular Vel", 0).withPosition(5, 0).withSize(1, 1).getEntry();

        driveXTab = tab.add("Drive X", 0).withPosition(0, 2).withSize(1, 1).getEntry();

        driveYTab = tab.add("Drive Y", 0).withPosition(1, 2).withSize(1, 1).getEntry();

        rotateTab = tab.add("Rotate", 0).withPosition(1, 3).withSize(1, 1).getEntry();

        odometryX = tab.add("Odometry X", 0).withPosition(3, 2).withSize(1, 1).getEntry();

        odometryY = tab.add("Odometry Y", 0).withPosition(4, 2).withSize(1, 1).getEntry();

        odometryDegrees =
            tab.add("Odometry Degrees", 0).withPosition(2, 2).withSize(1, 1).getEntry();

        // Customization
        rampRate = tab.add("Ramp Rate", DriveConstants.Drive.closedLoopRampSec).withPosition(5, 1)
            .getEntry();
        lastRampRate = DriveConstants.Drive.closedLoopRampSec;
      }
    }
  }

  // get the yaw angle
  public double getAngle() {
    if (gyro != null && gyroInputs.connected && !gyroInputs.calibrating && Constants.gyroEnabled) {
      return OrangeMath.boundDegrees(gyroInputs.yawAngleDeg);
    } else {
      return 0;
    }
  }

  // Get pitch in degrees. Positive angle is the front of the robot raised.
  public double getPitch() {
    if (gyro != null && gyroInputs.connected && !gyroInputs.calibrating && Constants.gyroEnabled) {
      return gyroInputs.pitchPositionDeg - pitchOffset;
    } else {
      return 0;
    }
  }

  // get the change of robot heading in degrees per sec
  public double getAngularVelocity() {
    if (gyro != null && gyroInputs.connected && !gyroInputs.calibrating && Constants.gyroEnabled) {
      return gyroInputs.yawVelocityDegPerSec;
    } else {
      return 0;
    }
  }

  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(gyroInputs.yawAngleDeg);
}

  @Override
  public void periodic() {
    if (Constants.driveEnabled) {
      // update logs
      for (SwerveModule module : swerveModules) {
        module.periodic();
      }
      if (Constants.gyroEnabled) {
        gyro.updateInputs(gyroInputs);
        Logger.getInstance().processInputs("Drive/Gyro", gyroInputs);
      }

      Translation2d vel = calcVelocity();
      Translation2d acc = calcAcceleration();

      // log velocity and acceleration here

      // update velocity
      latestVelocity = vel.getNorm() / 4;
      latestAcceleration = acc.getNorm() / 4;

      // acceleration must be calculated once and only once per periodic interval
      for (SwerveModule module : swerveModules) {
        module.snapshotAcceleration();
      }

      if (Constants.gyroEnabled) {
        updateOdometry();
      }

      if (Constants.debug) {
        botVelocityMag.setDouble(latestVelocity);
        botAccelerationMag.setDouble(latestAcceleration);
        botVelocityAngle.setDouble(vel.getAngle().getDegrees());
        botAccelerationAngle.setDouble(acc.getAngle().getDegrees());
        if (Constants.gyroEnabled) {
          yawTab.setDouble(getAngle());
          rollTab.setDouble(gyroInputs.rollPositionDeg);
          pitchTab.setDouble(getPitch());
          odometryX.setDouble(getPose2d().getX());
          odometryY.setDouble(getPose2d().getY());
          odometryDegrees.setDouble(getPose2d().getRotation().getDegrees());
          angularVel.setDouble(getAngularVelocity());
        }

        double newRampRate = rampRate.getDouble(lastRampRate);
        if (lastRampRate != newRampRate) {
          lastRampRate = newRampRate;
          for (SwerveModule module : swerveModules) {
            module.setDriveRampRate(newRampRate);
          }
        }
      }
    }
  }

  // rotation isn't considered to be movement
  public boolean isRobotMoving() {
    if (Constants.driveEnabled) {
      return latestVelocity >= DriveConstants.stoppedVelocityThresholdFtPerSec;
    } else {
      return false;
    }
  }

  public boolean isRobotMovingFast() {
    if (Constants.driveEnabled) {
      return latestVelocity >= DriveConstants.movingVelocityThresholdFtPerSec;
    } else {
      return false;
    }
  }

  public void resetFieldCentric(double offset) {
    if (Constants.driveEnabled && Constants.gyroEnabled && gyro != null) {
      gyro.setAngleAdjustment(0.0);
      gyro.setAngleAdjustment(gyroInputs.yawAngleDeg + offset);
      pitchOffset = gyroInputs.pitchPositionDeg;
    }
  }

  // this drive function is for regular driving when pivot point is at robot center point
  public void drive(double driveX, double driveY, double rotate) {
    drive(driveX, driveY, rotate, new Translation2d());
  }

  // main drive function accounts for spinout when center point is on swerve modules
  public void drive(double driveX, double driveY, double rotate, Translation2d centerOfRotation) {
    if (Constants.driveEnabled && Constants.gyroEnabled) {

      if (Constants.debug) {
        driveXTab.setDouble(driveX);
        driveYTab.setDouble(driveY);
        rotateTab.setDouble(rotate);
      }

      // convert to proper units (eventually we should move this up to DriveManual)
      rotate = rotate * DriveConstants.maxRotationSpeedRadSecond;
      driveX = driveX * DriveConstants.maxSpeedMetersPerSecond;
      driveY = driveY * DriveConstants.maxSpeedMetersPerSecond;

      // ready to drive!
      if ((driveX == 0) && (driveY == 0) && (rotate == 0)) {
        stop();
      } else {
        var swerveModuleStates =
            DriveLogic.calcModuleStates(driveX, driveY, rotate, centerOfRotation,
                getRotation2d(), kinematics);

        setModuleStates(swerveModuleStates);
      }
    }
  }

  // Rotate the robot to a specific heading while driving.
  // Must be invoked periodically to reach the desired heading.
  public void driveAutoRotate(double driveX, double driveY, double targetDeg) {
    if (Constants.driveEnabled) {

      if (Constants.debug) {
        rotPID.setP(rotkP.getDouble(DriveConstants.Auto.autoRotkP));
        rotPID.setD(rotkD.getDouble(DriveConstants.Auto.autoRotkD));
      }

      // Don't use absolute heading for PID controller to avoid discontinuity at +/- 180 degrees
      double headingChangeDeg = OrangeMath.boundDegrees(targetDeg - getAngle());
      double rotPIDSpeed = rotPID.calculate(0, headingChangeDeg);
    
      double maxAutoRotatePower = DriveLogic.detMaxAutoRotate(isRobotOverSlowRotateFtPerSec());
      double minAutoRotatePower = DriveLogic.detMinAutoRotate(isRobotMoving());
      double toleranceDeg = DriveLogic.detToleranceDeg(isRobotMoving());

      rotPIDSpeed = DriveLogic.boundRotatePID(headingChangeDeg, toleranceDeg, rotPIDSpeed,
          minAutoRotatePower, maxAutoRotatePower);

      drive(driveX, driveY, rotPIDSpeed);

      if (Constants.debug) {
        rotErrorTab.setDouble(headingChangeDeg);
        rotSpeedTab.setDouble(rotPIDSpeed);
      }
    }
  }

  public void resetRotatePID() {
    if (Constants.driveEnabled) {
      rotPID.reset();
    }
  }

  public void updateOdometry() {
    if (Constants.gyroEnabled) {
      odometry.update(getRotation2d(), getModulePostitions());
    }
  }

  public void resetOdometry(Pose2d pose) {
    if (Constants.gyroEnabled) {
      odometry.resetPosition(getRotation2d(), getModulePostitions(), pose);
    }
  }

  public void setCoastMode() {
    if (Constants.driveEnabled) {
      for (SwerveModule module : swerveModules) {
        module.setCoastmode();
      }
    }
  }

  public void setBrakeMode() {
    if (Constants.driveEnabled) {
      for (SwerveModule module : swerveModules) {
        module.setBrakeMode();
      }
    }
  }

  public void stop() {
    if (Constants.driveEnabled) {
      for (SwerveModule module : swerveModules) {
        module.stop();
      }
    }
  }

  public Pose2d getPose2d() {
    return odometry.getPoseMeters();
  }

  public void setModuleStates(SwerveModuleState[] states) {
    if (Constants.driveEnabled) {
      DriveLogic.desaturateModuleStates(states);
      int i = 0;
      for (SwerveModuleState s : states) {
        swerveModules[i].setDesiredState(s);
        i++;
      }
    }
  }

  public SwerveModulePosition[] getModulePostitions() {
    if (Constants.driveEnabled) {
      // wheel locations must be in the same order as the WheelPosition enum values
      return new SwerveModulePosition[] {
          swerveModules[WheelPosition.FRONT_RIGHT.wheelNumber].getPosition(),
          swerveModules[WheelPosition.FRONT_LEFT.wheelNumber].getPosition(),
          swerveModules[WheelPosition.BACK_LEFT.wheelNumber].getPosition(),
          swerveModules[WheelPosition.BACK_RIGHT.wheelNumber].getPosition()};
    } else {
      return null;
    }
  }

  public SwerveDriveKinematics getKinematics() {
    if (Constants.driveEnabled) {
      return kinematics;
    } else {
      return null;
    }
  }

  private Translation2d calcVelocity() {
    double[] currentAngle = new double[4];
    double[] currentVelocity = new double[4];
    for (int i = 0; i < swerveModules.length; i++) {
      currentAngle[i] = swerveModules[i].getInternalRotationDegrees();
      currentVelocity[i] = swerveModules[i].getVelocityFeetPerSec();
    }
    return DriveLogic.avgModuleVectors(currentAngle, currentVelocity);
  }

  private Translation2d calcAcceleration() {
    double[] currentAngle = new double[4];
    double[] currentAcceleration = new double[4];
    for (int i = 0; i < swerveModules.length; i++) {
      currentAngle[i] = swerveModules[i].getInternalRotationDegrees();
      currentAcceleration[i] = swerveModules[i].snapshotAcceleration();
    }
    return DriveLogic.avgModuleVectors(currentAngle, currentAcceleration);
  }

  private boolean isRobotOverSlowRotateFtPerSec() {
    return latestVelocity >= DriveConstants.Auto.slowAutoRotateFtPerSec;
  }
}
