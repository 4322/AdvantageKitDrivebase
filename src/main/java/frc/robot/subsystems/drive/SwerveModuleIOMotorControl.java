package frc.robot.subsystems.drive;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.WheelPosition;
import frc.utility.CanBusUtil;
import frc.utility.OrangeMath;

public class SwerveModuleIOMotorControl implements SwerveModuleIO {
    //drive motor
    private TalonFX driveMotor;
    private MotorOutputConfigs  mOutputConfigs;

    //rotate motor
    private TalonFX rotateMotor;
    private CurrentLimitsConfigs currentLimitConfigs = new CurrentLimitsConfigs();

    public SwerveModuleIOMotorControl(WheelPosition wheelPos) {
        switch(wheelPos) {
            case FRONT_RIGHT:
                driveMotor = new TalonFX(DriveConstants.frontRightDriveID, DriveConstants.Drive.canivoreName);
                rotateMotor = new TalonFX(DriveConstants.frontRightDriveID2, DriveConstants.Drive.canivoreName);
                break;
            case FRONT_LEFT:
                driveMotor = new TalonFX(DriveConstants.frontLeftDriveID, DriveConstants.Drive.canivoreName);
                rotateMotor = new TalonFX(DriveConstants.frontLeftDriveID2, DriveConstants.Drive.canivoreName);
                break;
            case BACK_RIGHT:
                driveMotor = new TalonFX(DriveConstants.rearRightDriveID, DriveConstants.Drive.canivoreName);
                rotateMotor = new TalonFX(DriveConstants.rearRightDriveID2, DriveConstants.Drive.canivoreName);
                break;
            case BACK_LEFT: 
                driveMotor = new TalonFX(DriveConstants.rearLeftDriveID, DriveConstants.Drive.canivoreName);
                rotateMotor = new TalonFX(DriveConstants.rearLeftDriveID2, DriveConstants.Drive.canivoreName);
                break;
        }

        mOutputConfigs = new MotorOutputConfigs();

        switch (Constants.driveDegradedMode) {
            case normal:
              configDrive(driveMotor, wheelPos, false);
              configDrive(rotateMotor, wheelPos, false);
              break;
            case sideMotorsOnly:
              configDrive(driveMotor, wheelPos, false);
              configDrive(rotateMotor, wheelPos, true);
              break;
            case centerMotorsOnly:
              TalonFX driveTemp = driveMotor;
              driveMotor = rotateMotor;
              rotateMotor = driveTemp;
              configDrive(driveMotor, wheelPos, false);
              configDrive(rotateMotor, wheelPos, true);
              break;
        }
        // need rapid position/velocity feedback for control logic
        driveMotor.getPosition().setUpdateFrequency(OrangeMath.msAndHzConverter(CanBusUtil.nextFastStatusPeriodMs()), 
        Constants.controllerConfigTimeoutMs);
        driveMotor.getVelocity().setUpdateFrequency(OrangeMath.msAndHzConverter(CanBusUtil.nextFastStatusPeriodMs()), 
        Constants.controllerConfigTimeoutMs);
    }

    private void configDrive(TalonFX talon, WheelPosition pos, boolean coastOnly) {
        Slot0Configs slot0config = new Slot0Configs();
        slot0config.kP = DriveConstants.Drive.kP;
        slot0config.kI = DriveConstants.Drive.kI;
        slot0config.kD = DriveConstants.Drive.kD;
        slot0config.kV = DriveConstants.Drive.FeedForward.voltsAtMaxSpeed[0];
        
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
        
      }

      // Below are the implementations of the methods in SwerveModuleIO.java

      @Override
      public void updateInputs(SwerveModuleIOInputs inputs) {
        //drive inputs
        inputs.driveRotations = driveMotor.getPosition().getValue();
        inputs.driveRotationsPerSec = driveMotor.getVelocity().getValue();
        inputs.driveAppliedVolts = driveMotor.getSupplyVoltage().getValue();
        inputs.driveCurrentAmps = driveMotor.getStatorCurrent().getValue();

        inputs.rotateRotations = rotateMotor.getPosition().getValue();
        inputs.rotateRotationsPerSec = rotateMotor.getVelocity().getValue();
        inputs.rotateAppliedVolts = rotateMotor.getSupplyVoltage().getValue();
        inputs.rotateCurrentAmps = rotateMotor.getStatorCurrent().getValue();
    }

    // PID methods for turn motor
    @Override
    public void setTurnPIDReference(double value, ControlType ctrl) {
      
    }

    // PID method for drive motors
    @Override
    public void setDrivePIDTargetVel(VelocityVoltage request) {
      driveMotor.setControl(request);
    }

    @Override
    public void setBrakeMode() {
      mOutputConfigs.NeutralMode = NeutralModeValue.Brake;
      // the following calls reset follower mode or something else that makes the robot uncontrollable
      //driveMotor.getConfigurator().apply(mOutputConfigs);
      //driveMotor2.getConfigurator().apply(mOutputConfigs);
    }

    @Override
    public void setCoastMode() {
        mOutputConfigs.NeutralMode = NeutralModeValue.Coast;
        // the following calls reset follower mode or something else that makes the robot uncontrollable
        //driveMotor.getConfigurator().apply(mOutputConfigs);
        //driveMotor2.getConfigurator().apply(mOutputConfigs);
    }

    @Override
    public void setClosedRampRate(double period) {
      ClosedLoopRampsConfigs closedLoopConfig = new ClosedLoopRampsConfigs();

      closedLoopConfig.VoltageClosedLoopRampPeriod = period;

      driveMotor.getConfigurator().apply(closedLoopConfig);
      rotateMotor.getConfigurator().apply(closedLoopConfig);
    }

    @Override
    public void setOpenRampRate(double period) {
      OpenLoopRampsConfigs openLoopConfig = new OpenLoopRampsConfigs();

      openLoopConfig.VoltageOpenLoopRampPeriod = period;

      driveMotor.getConfigurator().apply(openLoopConfig);
      rotateMotor.getConfigurator().apply(openLoopConfig);
    }

    @Override
    public void stopMotor() {
        driveMotor.stopMotor();
        rotateMotor.stopMotor();
    }
    
}
