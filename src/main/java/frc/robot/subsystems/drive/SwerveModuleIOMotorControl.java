package frc.robot.subsystems.drive;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.WheelPosition;
import frc.utility.CanBusUtil;
import frc.utility.OrangeMath;

public class SwerveModuleIOMotorControl implements SwerveModuleIO {
    private TalonFX driveMotor;

    private TalonFX turningMotor;

    //private SparkMaxAbsoluteEncoder encoder;
    
    private double[] feedForwardRPSThreshold = DriveConstants.Drive.FeedForward.feedForwardRPSThreshold.clone();
    private double[] feedForwardVolts = DriveConstants.Drive.FeedForward.voltsAtSpeedThresholds.clone();
    private double kSVolts = DriveConstants.Drive.kS;

    private double calculatedFeedForwardValue;

    public SwerveModuleIOMotorControl(WheelPosition wheelPos) {
        switch(wheelPos) {
            case FRONT_RIGHT:
                driveMotor = new TalonFX(DriveConstants.frontRightDriveID);
                turningMotor = new TalonFX(DriveConstants.frontRightRotationID);
                break;
            case FRONT_LEFT:
                driveMotor = new TalonFX(DriveConstants.frontLeftDriveID);
                turningMotor = new TalonFX(DriveConstants.frontLeftRotationID);
                break;
            case BACK_RIGHT:
                driveMotor = new TalonFX(DriveConstants.rearRightDriveID);
                turningMotor = new TalonFX(DriveConstants.rearRightRotationID);
                break;
            case BACK_LEFT: 
                driveMotor = new TalonFX(DriveConstants.rearLeftDriveID);
                turningMotor = new TalonFX(DriveConstants.rearLeftRotationID);
                break;
        }
        
        encoder = turningMotor.
        encoder = turningMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
        CanBusUtil.staggerSparkMax(turningMotor);
        CanBusUtil.staggerSparkMax(driveMotor);
        
        configDrive(driveMotor, wheelPos);

        configRotation(turningMotor);
    }

    private void configDrive(TalonFX talonFX, WheelPosition pos) {
        talonFX.getConfigurator().apply(new TalonFXConfiguration());
        TalonFXConfigurator config = talonFX.getConfigurator();
        Slot0Configs slot0Configs = new Slot0Configs();
        ClosedLoopRampsConfigs closedLoopRampsConfigs = new ClosedLoopRampsConfigs();
        OpenLoopRampsConfigs openLoopRampsConfigs = new OpenLoopRampsConfigs();
        CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
        slot0Configs.kP = DriveConstants.Drive.kP;
        slot0Configs.kI = DriveConstants.Drive.kI;
        slot0Configs.kD = DriveConstants.Drive.kD;
        slot0Configs.kV = DriveConstants.Drive.kF;
        closedLoopRampsConfigs.VoltageClosedLoopRampPeriod = DriveConstants.Drive.closedLoopRampSec;
        openLoopRampsConfigs.VoltageOpenLoopRampPeriod = DriveConstants.Drive.openLoopRampSec;
        currentLimitsConfigs.StatorCurrentLimit = DriveConstants.Drive.statorLimit;
        currentLimitsConfigs.SupplyCurrentLimit = DriveConstants.Drive.supplyLimit;
        currentLimitsConfigs.SupplyCurrentLimitEnable = DriveConstants.Drive.supplyEnabled;
        currentLimitsConfigs.SupplyCurrentThreshold = DriveConstants.Drive.supplyThreshold;
        //TODO: sparkMax.enableVoltageCompensation(DriveConstants.Drive.voltageCompSaturation);
        motorOutputConfigs.NeutralMode = NeutralModeValue.Coast;
        config.apply(slot0Configs);
        config.apply(closedLoopRampsConfigs);
        config.apply(openLoopRampsConfigs);
        config.apply(currentLimitsConfigs);
        config.apply(motorOutputConfigs);
        // Invert the left side modules so we can zero all modules with the bevel gears facing outward.
        // Without this code, all bevel gears would need to face right when the modules are zeroed.
        boolean isLeftSide = (pos == WheelPosition.FRONT_LEFT) || (pos == WheelPosition.BACK_LEFT);
        talonFX.setInverted(isLeftSide);

        // need rapid velocity feedback for control logic
        //TODO: CanBusUtil.fastVelocitySparkMax(driveMotor);
      }

      private void configRotation(TalonFX talonFX) {
        talonFX.getConfigurator().apply(new TalonFXConfiguration());
        Slot0Configs slot0Configs = new Slot0Configs();
        ClosedLoopRampsConfigs closedLoopRampsConfigs = new ClosedLoopRampsConfigs();
        VoltageConfigs voltageConfigs = new VoltageConfigs();
        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
        encoder.setInverted(true);
        talonFX.setInverted(true);

        TalonFXConfigurator config = talonFX.getConfigurator();
        slot0Configs.kP = DriveConstants.Rotation.kP;
        slot0Configs.kD = DriveConstants.Rotation.kD;
        closedLoopRampsConfigs.VoltageClosedLoopRampPeriod = DriveConstants.Rotation.configCLosedLoopRamp;
        //TODO: config.setSmartMotionAllowedClosedLoopError(DriveConstants.Rotation.allowableClosedloopError,0);
        voltageConfigs.PeakForwardVoltage = DriveConstants.Rotation.maxPower;
        voltageConfigs.PeakReverseVoltage = -DriveConstants.Rotation.maxPower;
        motorOutputConfigs.NeutralMode = NeutralModeValue.Coast;// Allow robot to be moved prior to enabling
        //TODO: sparkMax.enableVoltageCompensation(DriveConstants.Rotation.configVoltageCompSaturation); 
        talonFX.setSmartCurrentLimit(DriveConstants.Rotation.stallLimit, DriveConstants.Rotation.freeLimit); 
        encoder.setPositionConversionFactor(360);  // convert encoder position duty cycle to degrees
        
        config.setFeedbackDevice(encoder);
        config.setPositionPIDWrappingEnabled(true);
        config.setPositionPIDWrappingMinInput(0);
        config.setPositionPIDWrappingMaxInput(360);
    
        // need rapid position feedback for steering control
        //TODO: CanBusUtil.fastPositionSparkMaxAbs(turningMotor);
      }

      // Below are the implementations of the methods in SwerveModuleIO.java
      @Override
      public void updateInputs(SwerveModuleIOInputs inputs) {
        //drive inputs
        inputs.driveRotations = driveMotor.getPosition().getValue();
        inputs.driveRotationsPerSec = driveMotor.getVelocity().getValue()/60;
        inputs.driveAppliedVolts = driveMotor.getAppliedOutput() * driveMotor.getSupplyVoltage().getValue();
        inputs.driveCurrentAmps = driveMotor.getSupplyCurrent().getValue();
        //turn inputs
        inputs.turnVelocityDegPerSec = Units.rotationsToDegrees(turningMotor.getVelocity().getValue());
        inputs.turnAppliedVolts = turningMotor * turningMotor.getSupplyVoltage().getValue();
        inputs.turnCurrentAmps = turningMotor.getSupplyCurrent().getValue();
        inputs.turnDegrees = turningMotor.getPosition().getValue();

        inputs.calculatedFF = calculatedFeedForwardValue;
    }

    // PID methods for turn motor
    @Override
    public void setTurnAngle(double desiredAngle) {
           PositionVoltage positionVoltage = new PositionVoltage(desiredAngle);
      turningMotor.setControl(positionVoltage);
    }

    // PID method for drive motor
    @Override
    public void setDriveVoltage(double desiredMotorRPM) {
      // convert RPM to RPS
      double desiredMotorRPS = Math.abs(desiredMotorRPM / 60);
      int i;
      
      // If motor RPS less than element 1 of FF speed threshold, i defaults to 0
      // due to for loop iterating i at the end.
      // Greater than or equals to not used in order to protect against erroneous shuffleboard input
      // where element 0 of FF velocity threshold is changed from 0.
      for (i = feedForwardRPSThreshold.length - 1; i > 0; i--) { 
        if (desiredMotorRPS >= feedForwardRPSThreshold[i]) {
          break;
        }
      }

      // Linear extrapolation to account for edge case where requested speed 
      // is greater than max threshold speed.
      // Feed Forward calculated by determining linear equation between last two points.
      if (i == feedForwardRPSThreshold.length - 1) {
        int lastElement = i;
        int secondToLastElement = i - 1;
        // Slope between last point and second to last point used to predict corresponding 
        // Feed Forward value for requested RPS values beyond max speed threshold
        double slope = (feedForwardVolts[lastElement] - feedForwardVolts[secondToLastElement]) / 
                        (feedForwardRPSThreshold[lastElement] - feedForwardRPSThreshold[secondToLastElement]);

        calculatedFeedForwardValue = slope * (desiredMotorRPS - feedForwardRPSThreshold[lastElement]) 
                                      + feedForwardVolts[lastElement];
      }
      // Linear interpolation to calculate a more precise Feed Forward value for 
      // points between established thresholds.
      // Calculated through weighted average.
      else {
        int upperBound = i + 1;
        int lowerBound = i;
        // Calculated weight based on distance between lower bound value and desired speed value
        double weight = (desiredMotorRPS - feedForwardRPSThreshold[upperBound]) /
                    (feedForwardRPSThreshold[lowerBound] - feedForwardRPSThreshold[upperBound]);
        
        calculatedFeedForwardValue = (weight * feedForwardVolts[lowerBound]) + 
                                      ((1 - weight) * feedForwardVolts[upperBound]);
      }

      // make sure wheel RPS shuffleboard inputs are in ascending order
      if (calculatedFeedForwardValue < 0) {
        calculatedFeedForwardValue = 0;
        kSVolts = 0;
      }

      // need to reverse Feed Forward value sign if speed is negative
      calculatedFeedForwardValue = calculatedFeedForwardValue * Math.signum(desiredMotorRPM);

      // convert speed to volts while accounting for volts required to overcome static friction
      double desiredVolts = (kSVolts * Math.signum(desiredMotorRPM)) + (calculatedFeedForwardValue * desiredMotorRPS);
      
      // send requested voltage to SparkMAX
      
      driveMotor.setVoltage(desiredVolts);
      /**if (error != REVLibError.kOk) {
        DriverStation.reportError("Drive motor " + driveMotor.getDeviceId() + " error " + error.name() + " while sending requested voltage", false);
      }**/
    }

    @Override
    public void setFeedForwardSpeedThreshold(double[] newFeedForwardRPSThreshold) {
      for (int i = 0; i < feedForwardRPSThreshold.length; i++) {
        feedForwardRPSThreshold[i] = newFeedForwardRPSThreshold[i];
      }
    }
    
    @Override
    public void updateFeedForward(double[] newFeedForwardVolts) {
      for (int i = 0; i < feedForwardVolts.length; i++) {
          feedForwardVolts[i] = newFeedForwardVolts[i];
      }
    }

    @Override
    public void updateVoltsToOvercomeFriction(double newkSVolts) {
      kSVolts = newkSVolts;
    }

    @Override
    public void setBrakeMode() {
      MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
      motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
      driveMotor.getConfigurator().refresh(motorOutputConfigs);
      turningMotor.getConfigurator().refresh(motorOutputConfigs);
    }

    @Override
    public void setCoastMode() {
      MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
      motorOutputConfigs.NeutralMode = NeutralModeValue.Coast;
      driveMotor.getConfigurator().refresh(motorOutputConfigs);
      turningMotor.getConfigurator().refresh(motorOutputConfigs);
    }

    @Override
    public void setClosedRampRate(double period) {
      ClosedLoopRampsConfigs closedLoopRampsConfigs = new ClosedLoopRampsConfigs();
      closedLoopRampsConfigs.VoltageClosedLoopRampPeriod = period;
      driveMotor.getConfigurator().refresh(closedLoopRampsConfigs);
    }

    @Override
    public void setOpenRampRate(double period) {
      OpenLoopRampsConfigs openLoopRampsConfigs = new OpenLoopRampsConfigs();
      openLoopRampsConfigs.VoltageOpenLoopRampPeriod = period;
      driveMotor.getConfigurator().refresh(openLoopRampsConfigs);
    }

    @Override
    public void stopMotor() {
        driveMotor.stopMotor();
        turningMotor.stopMotor();
    }
    
}
