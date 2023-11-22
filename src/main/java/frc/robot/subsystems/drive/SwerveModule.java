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
import frc.robot.Constants.WheelPosition;
import frc.utility.OrangeMath;
import frc.utility.CanBusUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.MathUtil;

import edu.wpi.first.wpilibj.Timer;

public class SwerveModule {
  private SwerveModuleIO driveIO;
  private SwerveModuleIO turnIO;

  private SwerveModuleIOInputsAutoLogged inputs = new SwerveModuleIOInputsAutoLogged();

  private double previousRate = 0;
  private double previousTime = 0;
  private double filteredAccel = 0;

  public SwerveModule(SwerveModuleIO driveIO, SwerveModuleIO turnIO) {
    this.driveIO = driveIO;
    this.turnIO = turnIO;
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
}
