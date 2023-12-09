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
    private CANSparkMax driveMotor;

    //turning motor
    private CANSparkMax turningMotor;
    private SparkMaxAbsoluteEncoder encoder;

    public SwerveModuleIOMotorControl(WheelPosition wheelPos) {
        switch(wheelPos) {
            case FRONT_RIGHT:
                driveMotor = new CANSparkMax(DriveConstants.frontRightDriveID, MotorType.kBrushless);
                turningMotor = new CANSparkMax(DriveConstants.frontRightRotationID, MotorType.kBrushless);
                break;
            case FRONT_LEFT:
                driveMotor = new CANSparkMax(DriveConstants.frontLeftDriveID, MotorType.kBrushless);
                turningMotor = new CANSparkMax(DriveConstants.frontLeftRotationID, MotorType.kBrushless); 
                break;
            case BACK_RIGHT:
                driveMotor = new CANSparkMax(DriveConstants.rearRightDriveID, MotorType.kBrushless);
                turningMotor = new CANSparkMax(DriveConstants.rearRightRotationID, MotorType.kBrushless);
                break;
            case BACK_LEFT: 
                driveMotor = new CANSparkMax(DriveConstants.rearLeftDriveID, MotorType.kBrushless);
                turningMotor = new CANSparkMax(DriveConstants.rearLeftRotationID, MotorType.kBrushless);
                break;
        }

        encoder = turningMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
        encoder.setInverted(true);
        turningMotor.setInverted(true);
        driveMotor.setInverted(false); // NEED TO CHECK WHEN SETTING UP
        CanBusUtil.staggerSparkMax(turningMotor);
        CanBusUtil.staggerSparkMax(driveMotor);

        
        // need rapid position/velocity feedback for control logic
        driveMotor.getPosition().setUpdateFrequency(OrangeMath.msAndHzConverter(CanBusUtil.nextFastStatusPeriodMs()), 
        Constants.controllerConfigTimeoutMs);
        driveMotor.getVelocity().setUpdateFrequency(OrangeMath.msAndHzConverter(CanBusUtil.nextFastStatusPeriodMs()), 
        Constants.controllerConfigTimeoutMs);

        configRotation(turningMotor);
    }

    private void configDrive(CANSparkMax sparkMax, WheelPosition pos) {
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
        sparkMax.setInverted(isLeftSide);
    
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
        CanBusUtil.fastPositionSparkMaxAbs(turningMotor);
      }

      // Below are the implementations of the methods in SwerveModuleIO.java

      @Override
      public void updateInputs(SwerveModuleIOInputs inputs) {
        //drive inputs
        inputs.driveRotations = driveMotor.getPosition().getValue();
        inputs.driveRotationsPerSec = driveMotor.getVelocity().getValue();
        inputs.driveAppliedVolts = driveMotor.getSupplyVoltage().getValue();
        inputs.driveCurrentAmps = driveMotor.getStatorCurrent().getValue();
        
        //turn inputs
        inputs.turnVelocityDegPerSec = Units.rotationsToDegrees(encoder.getVelocity());
        inputs.turnAppliedVolts = turningMotor.getAppliedOutput() * turningMotor.getBusVoltage();
        inputs.turnCurrentAmps = turningMotor.getOutputCurrent();
        inputs.turnDegrees = encoder.getPosition();
    }

    // PID methods for turn motor
    @Override
    public void setTurnPIDTargetAngle(double desiredAngle) {
      turningMotor.getPIDController().setReference(desiredAngle, ControlType.kPosition);
    }

    // PID method for drive motors
    @Override
    public void setDrivePIDTargetVel(double desiredVelocity) {
      driveMotor.setControl(request);
    }

    @Override
    public void setBrakeMode() {
      driveMotor.setIdleMode(IdleMode.kBrake);
      turningMotor.setIdleMode(IdleMode.kBrake);
    }

    @Override
    public void setCoastMode() {
        driveMotor.setIdleMode(IdleMode.kCoast);
        turningMotor.setIdleMode(IdleMode.kCoast);
    }

    @Override
    public void setClosedRampRate(double period) {
      ClosedLoopRampsConfigs closedLoopConfig = new ClosedLoopRampsConfigs();

      closedLoopConfig.VoltageClosedLoopRampPeriod = period;

      driveMotor.getConfigurator().apply(closedLoopConfig);
    }

    @Override
    public void setOpenRampRate(double period) {
      OpenLoopRampsConfigs openLoopConfig = new OpenLoopRampsConfigs();

      openLoopConfig.VoltageOpenLoopRampPeriod = period;

      driveMotor.getConfigurator().apply(openLoopConfig);
    }

    @Override
    public void stopMotor() {
        driveMotor.stopMotor();
        turningMotor.stopMotor();
    }
    
}
