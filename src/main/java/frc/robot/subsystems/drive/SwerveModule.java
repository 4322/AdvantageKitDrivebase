package frc.robot.subsystems.drive;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.utility.OrangeMath;
import frc.utility.CanBusUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.MathUtil;

import edu.wpi.first.wpilibj.Timer;

public class SwerveModule {
  private CANSparkMax turningMotor;
  private TalonFX driveMotor;
  private MotorOutputConfigs  mOutputConfigs;
  private TalonFX driveMotor2;
  private SparkMaxAbsoluteEncoder encoder;
  private WheelPosition wheelPosition;
  private CurrentLimitsConfigs currentLimitConfigs = new CurrentLimitsConfigs();

  private double previousRate = 0;
  private double previousTime = 0;
  private double filteredAccel = 0;

  public SwerveModule(int rotationID, int wheelID, int wheelID2, WheelPosition pos, int encoderID) {
    turningMotor = new CANSparkMax(rotationID, MotorType.kBrushless);
    driveMotor = new TalonFX(wheelID, Constants.DriveConstants.Drive.canivoreName);
    driveMotor2 = new TalonFX(wheelID2, Constants.DriveConstants.Drive.canivoreName);
    encoder = turningMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
    encoder.setInverted(true);
    mOutputConfigs = new MotorOutputConfigs();
    turningMotor.setInverted(true);
    wheelPosition = pos;

    CanBusUtil.staggerSparkMax(turningMotor);
  }

  public void init() {
    switch (Constants.driveDegradedMode) {
      case normal:
        configDrive(driveMotor, wheelPosition, false);
        configDrive(driveMotor2, wheelPosition, false);
        driveMotor2.setControl(new Follower(driveMotor.getDeviceID(), false));
        break;
      case sideMotorsOnly:
        configDrive(driveMotor, wheelPosition, false);
        configDrive(driveMotor2, wheelPosition, true);
        break;
      case centerMotorsOnly:
        TalonFX driveTemp = driveMotor;
        driveMotor = driveMotor2;
        driveMotor2 = driveTemp;
        configDrive(driveMotor, wheelPosition, false);
        configDrive(driveMotor2, wheelPosition, true);
        break;
    }
    // need rapid position/velocity feedback for control logic
    driveMotor.getPosition().setUpdateFrequency(OrangeMath.msAndHzConverter(CanBusUtil.nextFastStatusPeriodMs()), 
      Constants.controllerConfigTimeoutMs);
    driveMotor.getVelocity().setUpdateFrequency(OrangeMath.msAndHzConverter(CanBusUtil.nextFastStatusPeriodMs()), 
      Constants.controllerConfigTimeoutMs);

    configRotation(turningMotor);
  }

  private void configDrive(TalonFX talon, WheelPosition pos, boolean coastOnly) {
    Slot0Configs slot0config = new Slot0Configs();
    slot0config.kP = DriveConstants.Drive.kP;
    slot0config.kI = DriveConstants.Drive.kI;
    slot0config.kD = DriveConstants.Drive.kD;
    slot0config.kV = DriveConstants.Drive.kV;
    
    ClosedLoopRampsConfigs closedLoopConfig = new ClosedLoopRampsConfigs();
    OpenLoopRampsConfigs openLoopConfig = new OpenLoopRampsConfigs();

    if (coastOnly) {
      // for identifying failed Falcon outout shafts
      mOutputConfigs.NeutralMode = NeutralModeValue.Coast;
    } else {
      // we would like to start in coast mode, but we can't switch from coast to brake, so just use brake always
      mOutputConfigs.NeutralMode = NeutralModeValue.Brake;
    }
    mOutputConfigs.DutyCycleNeutralDeadband = DriveConstants.Drive.brakeModeDeadband;
    closedLoopConfig.VoltageClosedLoopRampPeriod = DriveConstants.Drive.closedLoopRampSec;
    openLoopConfig.VoltageOpenLoopRampPeriod = DriveConstants.Drive.openLoopRampSec;
     
    talon.getConfigurator().apply(slot0config);
    talon.getConfigurator().apply(closedLoopConfig);
    talon.getConfigurator().apply(openLoopConfig);
    talon.getConfigurator().apply(mOutputConfigs);
    
    // Invert the left side modules so we can zero all modules with the bevel gears facing outward.
    // Without this code, all bevel gears would need to face right when the modules are zeroed.
    boolean isLeftSide = (pos == WheelPosition.FRONT_LEFT) || (pos == WheelPosition.BACK_LEFT);
    talon.setInverted(isLeftSide);

    // applies stator & supply current limit configs to device
    // refer to https://pro.docs.ctr-electronics.com/en/latest/docs/api-reference/api-usage/configuration.html 
    currentLimitConfigs.StatorCurrentLimitEnable = DriveConstants.Drive.statorEnabled;
    currentLimitConfigs.StatorCurrentLimit = DriveConstants.Drive.statorLimit;
    currentLimitConfigs.SupplyCurrentLimit = DriveConstants.Drive.supplyLimit;
    currentLimitConfigs.SupplyCurrentThreshold = DriveConstants.Drive.supplyThreshold;
    currentLimitConfigs.SupplyTimeThreshold = DriveConstants.Drive.supplyTime;
    currentLimitConfigs.SupplyCurrentLimitEnable = DriveConstants.Drive.supplyEnabled;
    talon.getConfigurator().apply(currentLimitConfigs);
  }
 
  private void configRotation(CANSparkMax sparkMax) {
    SparkMaxPIDController config = sparkMax.getPIDController();
    config.setP(DriveConstants.Rotation.kP,0);
    config.setD(DriveConstants.Rotation.kD,0);
    sparkMax.setClosedLoopRampRate(DriveConstants.Rotation.configCLosedLoopRamp);
    config.setSmartMotionAllowedClosedLoopError(DriveConstants.Rotation.allowableClosedloopError,0);
    config.setOutputRange(-DriveConstants.Rotation.maxPower, DriveConstants.Rotation.maxPower);
    sparkMax.setIdleMode(IdleMode.kCoast); // Allow robot to be moved prior to enabling

    sparkMax.enableVoltageCompensation(DriveConstants.Rotation.configVoltageCompSaturation); 
    sparkMax.setSmartCurrentLimit(DriveConstants.Rotation.stallLimit, DriveConstants.Rotation.freeLimit); 
    encoder.setPositionConversionFactor(360);  // convert encoder position duty cycle to degrees
    sparkMax.getPIDController().setFeedbackDevice(encoder);
    sparkMax.getPIDController().setPositionPIDWrappingEnabled(true);
    sparkMax.getPIDController().setPositionPIDWrappingMinInput(0);
    sparkMax.getPIDController().setPositionPIDWrappingMaxInput(360);

    // need rapid position feedback for steering control
    CanBusUtil.fastPositionSparkMax(turningMotor);
  }

  public double getInternalRotationDegrees() {
    return OrangeMath.boundDegrees(encoder.getPosition());
  }

  public double getDistance() {
    return OrangeMath.falconRotationsToMeters(driveMotor.getPosition().getValue(),
        OrangeMath.getCircumference(OrangeMath.inchesToMeters(DriveConstants.Drive.wheelDiameterInches)),
        DriveConstants.Drive.gearRatio);
  }

  public double getVelocity() {
    // feet per second
    return driveMotor.getVelocity().getValue() / Constants.DriveConstants.Drive.gearRatio 
        * Math.PI * Constants.DriveConstants.Drive.wheelDiameterInches / 12;
  }

  public double snapshotAcceleration() {

    double currentRate = this.getVelocity();
    double currentTime = Timer.getFPGATimestamp();

    double acceleration = (currentRate - previousRate) / (currentTime - previousTime);

    previousRate = currentRate;
    previousTime = currentTime;

    filteredAccel = acceleration * 0.5 + filteredAccel * 0.5; // dampens random spikes due to the
                                                              // fact that we are deriving this
                                                              // value
    return filteredAccel;
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocity() * Constants.feetToMeters, 
      Rotation2d.fromDegrees(encoder.getPosition()));
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getDistance(), Rotation2d.fromDegrees(encoder.getPosition()));
  }

  public void setDesiredState(SwerveModuleState desiredState) {
    if (Constants.driveEnabled) {

      // Optimize the reference state to avoid spinning further than 90 degrees
      SwerveModuleState state =
          SwerveModuleState.optimize(desiredState, Rotation2d.fromDegrees(encoder.getPosition()));

      driveMotor.setControl(new VelocityVoltage(state.speedMetersPerSecond
        / (DriveConstants.Drive.wheelDiameterInches * Constants.inchesToMeters * Math.PI)
        * DriveConstants.Drive.gearRatio).withEnableFOC(true));
              
      if (!Constants.steeringTuningMode) {
        turningMotor.getPIDController().setReference(
          MathUtil.inputModulus(state.angle.getDegrees(), 0, 360), 
          ControlType.kPosition);
      }
    }
  }

  public void setCoastmode() {
    if (Constants.driveEnabled) {
      mOutputConfigs.NeutralMode = NeutralModeValue.Coast;
      // the following calls reset follower mode or something else that makes the robot uncontrollable
      //driveMotor.getConfigurator().apply(mOutputConfigs);
      //driveMotor2.getConfigurator().apply(mOutputConfigs);
      turningMotor.setIdleMode(IdleMode.kCoast);
    }
  }

  public void setBrakeMode() {
    if (Constants.driveEnabled) {
      mOutputConfigs.NeutralMode = NeutralModeValue.Brake;
      // the following calls reset follower mode or something else that makes the robot uncontrollable
      //driveMotor.getConfigurator().apply(mOutputConfigs);
      //driveMotor2.getConfigurator().apply(mOutputConfigs);
      turningMotor.setIdleMode(IdleMode.kBrake);
    }
  }

  public void stop() {
    if (Constants.driveEnabled) {
      if (!Constants.steeringTuningMode) {
        driveMotor.stopMotor();
        turningMotor.stopMotor();
      }
    }
  }

  public enum WheelPosition {
    // construction of SwerveDriveKinematics is dependent on this enum

    FRONT_RIGHT(0), FRONT_LEFT(1), BACK_LEFT(2), BACK_RIGHT(3);

    public int wheelNumber;

    WheelPosition(int id) {
      wheelNumber = id;
    }
  }
}
