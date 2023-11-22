package frc.robot.subsystems.drive;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.WheelPosition;
import frc.utility.CanBusUtil;
import frc.utility.OrangeMath;

public class SwerveModuleIOTalonFX implements SwerveModuleIO {
    private TalonFX driveMotor;
    private MotorOutputConfigs  mOutputConfigs;
    private TalonFX driveMotor2;
    private CurrentLimitsConfigs currentLimitConfigs = new CurrentLimitsConfigs();

    public SwerveModuleIOTalonFX(WheelPosition wheelPos) {
        switch(wheelPos) {
            case FRONT_RIGHT:
                driveMotor = new TalonFX(DriveConstants.frontRightDriveID, DriveConstants.Drive.canivoreName);
                driveMotor2 = new TalonFX(DriveConstants.frontRightDriveID2, DriveConstants.Drive.canivoreName);
                break;
            case FRONT_LEFT:
                driveMotor = new TalonFX(DriveConstants.frontLeftDriveID, DriveConstants.Drive.canivoreName);
                driveMotor2 = new TalonFX(DriveConstants.frontLeftDriveID2, DriveConstants.Drive.canivoreName);
                break;
            case BACK_RIGHT:
                driveMotor = new TalonFX(DriveConstants.rearRightDriveID, DriveConstants.Drive.canivoreName);
                driveMotor2 = new TalonFX(DriveConstants.rearRightDriveID2, DriveConstants.Drive.canivoreName);
                break;
            case BACK_LEFT: 
                driveMotor = new TalonFX(DriveConstants.rearLeftDriveID, DriveConstants.Drive.canivoreName);
                driveMotor2 = new TalonFX(DriveConstants.rearRightDriveID2, DriveConstants.Drive.canivoreName);
                break;
        }
        mOutputConfigs = new MotorOutputConfigs();

        switch (Constants.driveDegradedMode) {
            case normal:
              configDrive(driveMotor, wheelPos, false);
              configDrive(driveMotor2, wheelPos, false);
              driveMotor2.setControl(new Follower(driveMotor.getDeviceID(), false));
              break;
            case sideMotorsOnly:
              configDrive(driveMotor, wheelPos, false);
              configDrive(driveMotor2, wheelPos, true);
              break;
            case centerMotorsOnly:
              TalonFX driveTemp = driveMotor;
              driveMotor = driveMotor2;
              driveMotor2 = driveTemp;
              configDrive(driveMotor, wheelPos, false);
              configDrive(driveMotor2, wheelPos, true);
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

    

}
