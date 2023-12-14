package frc.robot.subsystems.drive;

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

        
        configDrive(driveMotor, wheelPos);

        configRotation(turningMotor);
    }

    private void configDrive(CANSparkMax sparkMax, WheelPosition pos) {
        SparkMaxPIDController config = sparkMax.getPIDController();

        // PID config for 4 slotIDs
        for (int i = 0; i <= 3; i++) {
          config.setP(DriveConstants.Drive.kP, i);
          config.setI(DriveConstants.Drive.kI, i);
          config.setD(DriveConstants.Drive.kD, i);
          config.setFF(DriveConstants.Drive.FeedForward.voltsAtMaxSpeed[i], i);
          config.setOutputRange(0, 0, i); //TODO
        }

        


        sparkMax.setClosedLoopRampRate(DriveConstants.Drive.closedLoopRampSec);
        sparkMax.setOpenLoopRampRate(DriveConstants.Drive.openLoopRampSec);
        
        sparkMax.setIdleMode(IdleMode.kCoast); // Allow robot to be moved prior to enabling
        
        // Invert the left side modules so we can zero all modules with the bevel gears facing outward.
        // Without this code, all bevel gears would need to face right when the modules are zeroed.
        boolean isLeftSide = (pos == WheelPosition.FRONT_LEFT) || (pos == WheelPosition.BACK_LEFT);
        sparkMax.setInverted(isLeftSide);

        sparkMax.setSmartCurrentLimit(0, 0); // TODO
        

        // need rapid velocity feedback for control logic
        CanBusUtil.fastVelocitySparkMax(driveMotor);
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
        inputs.driveRotations = driveMotor.getEncoder().getPosition();
        inputs.driveRotationsPerSec = driveMotor.getEncoder().getVelocity()/60;
        inputs.driveAppliedVolts = driveMotor.getAppliedOutput() * driveMotor.getBusVoltage();
        inputs.driveCurrentAmps = driveMotor.getOutputCurrent();
        
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
    public void setDrivePIDTargetVel(double desiredVelocity, double[] setFeedForward, double[] thresholdRotPerSec) {
      for (int i = 0; i <= 3; i++) {
        driveMotor.getPIDController().setFF(setFeedForward[i], i);
      }
      double wheelRotations = driveMotor.getEncoder().getPosition();

      if (wheelRotations <= thresholdRotPerSec[0]) {
        driveMotor.getPIDController().setReference(desiredVelocity, ControlType.kVelocity, 0);
      }
      else if (wheelRotations <= thresholdRotPerSec[1]) {
        driveMotor.getPIDController().setReference(desiredVelocity, ControlType.kVelocity, 1); 
      }
      else if (wheelRotations <= thresholdRotPerSec[2]) {
        driveMotor.getPIDController().setReference(desiredVelocity, ControlType.kVelocity, 2);
      }
      else if (wheelRotations <= thresholdRotPerSec[3]) {
        driveMotor.getPIDController().setReference(desiredVelocity, ControlType.kVelocity, 3);
      }
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
