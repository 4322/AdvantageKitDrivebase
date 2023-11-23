package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.revrobotics.CANSparkMax.ControlType;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.WheelPosition;
import frc.utility.OrangeMath;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.MathUtil;

import edu.wpi.first.wpilibj.Timer;

public class SwerveModule {
  private SwerveModuleIO io;
  private SwerveModuleIOInputsAutoLogged inputs = new SwerveModuleIOInputsAutoLogged();
  private WheelPosition wheelPos;

  private double previousRate = 0;
  private double previousTime = 0;
  private double filteredAccel = 0;

  public SwerveModule(WheelPosition wheelPos, SwerveModuleIO io) {
    this.io = io;
    this.wheelPos = wheelPos;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Drive/SwerveModule " + wheelPos.wheelNumber, inputs);
  }

  public double getInternalRotationDegrees() {
    return OrangeMath.boundDegrees(inputs.turnRotations);
  }

  public double getDistance() {
    return OrangeMath.falconRotationsToMeters(inputs.drive1Position,
        OrangeMath.getCircumference(OrangeMath.inchesToMeters(DriveConstants.Drive.wheelDiameterInches)),
        DriveConstants.Drive.gearRatio);
  }

  public double getVelocity() {
    // feet per second
    return inputs.drive1VelocityRadPerSec * Constants.DriveConstants.Drive.wheelDiameterInches / 12;
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
      Rotation2d.fromDegrees(inputs.turnRotations));
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getDistance(), Rotation2d.fromDegrees(inputs.turnRotations));
  }

  public void setDesiredState(SwerveModuleState desiredState) {
    if (Constants.driveEnabled) {

      // Optimize the reference state to avoid spinning further than 90 degrees
      SwerveModuleState state =
          SwerveModuleState.optimize(desiredState, Rotation2d.fromDegrees(inputs.turnRotations));

          io.setDrivePIDTargetVel(new VelocityVoltage(state.speedMetersPerSecond
          / (DriveConstants.Drive.wheelDiameterInches * Constants.inchesToMeters * Math.PI)
          * DriveConstants.Drive.gearRatio).withEnableFOC(true));
              
      if (!Constants.steeringTuningMode) {
        io.setTurnPIDReference(MathUtil.inputModulus(state.angle.getDegrees(), 0, 360), 
                                ControlType.kPosition);
      }
    }
  }

  public void setCoastmode() {
    if (Constants.driveEnabled) {
      io.setCoastMode();
    }
  }

  public void setBrakeMode() {
    if (Constants.driveEnabled) {
      io.setBrakeMode();
    }
  }

  public void stop() {
    if (Constants.driveEnabled) {
      if (!Constants.steeringTuningMode) {
        io.stopMotor();
      }
    }
  }
}
