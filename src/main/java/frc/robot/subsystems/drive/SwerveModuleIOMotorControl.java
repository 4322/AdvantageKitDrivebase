package frc.robot.subsystems.drive;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.WheelPosition;
import frc.utility.CanBusUtil;
import frc.utility.OrangeMath;

public class SwerveModuleIOMotorControl implements SwerveModuleIO {
    private CANSparkMax driveMotor;

    private CANSparkMax turningMotor;
    private SparkMaxAbsoluteEncoder encoder;
    
    private double[] feedForwardMetersPerSecThreshold = DriveConstants.Drive.FeedForward.feedForwardMetersPerSecThreshold.clone();
    private double[] feedForwardVoltsOverMetersPerSec = DriveConstants.Drive.FeedForward.voltsOverMetersPerSecAtSpeedThresholds.clone();
    private double kSVolts = DriveConstants.Drive.kS;

    private double calcFeedForwardVoltsOverMetersPerSec;
    private double desiredVolts;

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
        CanBusUtil.staggerSparkMax(turningMotor);
        CanBusUtil.staggerSparkMax(driveMotor);
        
        configDrive(driveMotor, wheelPos);

        configRotation(turningMotor);
    }

    private void configDrive(CANSparkMax sparkMax, WheelPosition pos) {
        sparkMax.restoreFactoryDefaults();

        SparkMaxPIDController config = sparkMax.getPIDController();

        config.setP(DriveConstants.Drive.kP, 0);
        config.setI(DriveConstants.Drive.kI, 0);
        config.setIZone(DriveConstants.Drive.kIZone, 0);
        config.setD(DriveConstants.Drive.kD, 0);
        config.setFF(DriveConstants.Drive.kF, 0);

        sparkMax.setClosedLoopRampRate(DriveConstants.Drive.closedLoopRampSec);
        sparkMax.setOpenLoopRampRate(DriveConstants.Drive.openLoopRampSec);
        sparkMax.setSmartCurrentLimit(DriveConstants.Drive.currentLimit);
        sparkMax.setSecondaryCurrentLimit(DriveConstants.Drive.secondaryCurrentLimit);
        sparkMax.enableVoltageCompensation(DriveConstants.Drive.voltageCompSaturation);
        sparkMax.setIdleMode(IdleMode.kCoast); // Allow robot to be moved prior to enabling
        sparkMax.getEncoder().setPositionConversionFactor(Math.PI * Constants.DriveConstants.Drive.wheelDiameterInches 
            * Constants.inchesToMeters / Constants.DriveConstants.Drive.gearRatio); // motor rotations to wheel meters
        sparkMax.getEncoder().setVelocityConversionFactor(Math.PI * Constants.DriveConstants.Drive.wheelDiameterInches 
            * Constants.inchesToMeters / 60 / Constants.DriveConstants.Drive.gearRatio); // motor RPM to wheel m/s
        
        // Invert the left side modules so we can zero all modules with the bevel gears facing outward.
        // Without this code, all bevel gears would need to face right when the modules are zeroed.
        boolean isLeftSide = (pos == WheelPosition.FRONT_LEFT) || (pos == WheelPosition.BACK_LEFT);
        sparkMax.setInverted(isLeftSide);

        // need rapid velocity feedback for control logic
        CanBusUtil.fastVelocitySparkMax(driveMotor);
      }

      private void configRotation(CANSparkMax sparkMax) {
        sparkMax.restoreFactoryDefaults();

        encoder.setInverted(true);
        turningMotor.setInverted(true);

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
        config.setFeedbackDevice(encoder);
        config.setPositionPIDWrappingEnabled(true);
        config.setPositionPIDWrappingMinInput(0);
        config.setPositionPIDWrappingMaxInput(360);
    
        // need rapid position feedback for steering control
        CanBusUtil.fastPositionSparkMaxAbs(turningMotor);
      }

      // Below are the implementations of the methods in SwerveModuleIO.java
      @Override
      public void updateInputs(SwerveModuleIOInputs inputs) {
        //drive inputs
        inputs.driveMeters = driveMotor.getEncoder().getPosition();
        inputs.driveMetersPerSec = driveMotor.getEncoder().getVelocity();
        inputs.driveAppliedVolts = driveMotor.getAppliedOutput() * driveMotor.getBusVoltage();
        inputs.driveCurrentAmps = driveMotor.getOutputCurrent();
        
        //turn inputs
        inputs.turnVelocityDegPerSec = Units.rotationsToDegrees(encoder.getVelocity());
        inputs.turnAppliedVolts = turningMotor.getAppliedOutput() * turningMotor.getBusVoltage();
        inputs.turnCurrentAmps = turningMotor.getOutputCurrent();
        inputs.turnDegrees = encoder.getPosition();

        inputs.calculatedFF = calcFeedForwardVoltsOverMetersPerSec;
        inputs.calculatedVolts = desiredVolts;
    }

    // set turn angle of wheel using hardware PID
    @Override
    public void setTurnAngle(double desiredAngle) {
      turningMotor.getPIDController().setReference(desiredAngle, ControlType.kPosition);
    }

    // set drive motor voltage based on desired wheel m/s
    @Override
    public void setDriveVoltage(double desiredWheelMetersPerSec) {
      double desiredWheelMetersPerSecAbs = Math.abs(desiredWheelMetersPerSec);
      int i;
      
      // If wheel m/s less than element 1 of FF speed threshold, i defaults to 0
      // due to for loop iterating i at the end.
      // Greater than or equals to not used in order to protect against erroneous shuffleboard input
      // where element 0 of FF velocity threshold is changed from 0.
      for (i = feedForwardMetersPerSecThreshold.length - 1; i > 0; i--) { 
        if (desiredWheelMetersPerSecAbs >= feedForwardMetersPerSecThreshold[i]) {
          break;
        }
      }

      // Linear extrapolation to account for edge case where requested wheel speed 
      // is greater than max threshold speed.
      // Feed Forward calculated by determining linear equation between last two points.
      if (i == feedForwardMetersPerSecThreshold.length - 1) {
        int lastElement = i;
        int secondToLastElement = i - 1;
        // Slope between last point and second to last point used to predict corresponding 
        // Feed Forward value for requested m/s values beyond max speed threshold
        double slope = (feedForwardVoltsOverMetersPerSec[lastElement] - feedForwardVoltsOverMetersPerSec[secondToLastElement]) / 
                        (feedForwardMetersPerSecThreshold[lastElement] - feedForwardMetersPerSecThreshold[secondToLastElement]); // TODO: fix potential divide by 0

        calcFeedForwardVoltsOverMetersPerSec = slope * (desiredWheelMetersPerSecAbs - feedForwardMetersPerSecThreshold[lastElement]) 
                                      + feedForwardVoltsOverMetersPerSec[lastElement];
      }
      // Linear interpolation to calculate a more precise Feed Forward value for 
      // points between established thresholds.
      // Calculated through weighted average.
      else {
        int upperBound = i + 1;
        int lowerBound = i;
        // Calculated weight based on distance between lower bound value and desired wheel speed value
        double weight = (desiredWheelMetersPerSecAbs - feedForwardMetersPerSecThreshold[upperBound]) /
                    (feedForwardMetersPerSecThreshold[lowerBound] - feedForwardMetersPerSecThreshold[upperBound]);  // TODO: fix potential divide by 0
        
        calcFeedForwardVoltsOverMetersPerSec = (weight * feedForwardVoltsOverMetersPerSec[lowerBound]) + 
                                      ((1 - weight) * feedForwardVoltsOverMetersPerSec[upperBound]);
      }

      // make sure wheel m/s shuffleboard inputs are in ascending order
      if (calcFeedForwardVoltsOverMetersPerSec < 0) {
        calcFeedForwardVoltsOverMetersPerSec = 0;
        kSVolts = 0;
      }

      // convert speed to volts while accounting for volts required to overcome static friction
      desiredVolts = Math.signum(desiredWheelMetersPerSec) * (kSVolts + (calcFeedForwardVoltsOverMetersPerSec * desiredWheelMetersPerSecAbs));
      
      // send requested voltage to SparkMAX
      REVLibError error = driveMotor.getPIDController().setReference(desiredVolts, ControlType.kVoltage, 0);
      if (error != REVLibError.kOk) {
        DriverStation.reportError("Drive motor " + driveMotor.getDeviceId() + " error " + error.name() + " while sending requested voltage", false);
      }
    }

    @Override
    public void setFeedForwardSpeedThreshold(double[] newFeedForwardMetersPerSecThreshold) {
      for (int i = 0; i < feedForwardMetersPerSecThreshold.length; i++) {
        feedForwardMetersPerSecThreshold[i] = newFeedForwardMetersPerSecThreshold[i];
      }
    }
    
    @Override
    public void updateFeedForward(double[] newFeedForwardVolts) {
      for (int i = 0; i < feedForwardVoltsOverMetersPerSec.length; i++) {
          feedForwardVoltsOverMetersPerSec[i] = newFeedForwardVolts[i];
      }
    }

    @Override
    public void updateVoltsToOvercomeFriction(double newkSVolts) {
      kSVolts = newkSVolts;
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
      driveMotor.setClosedLoopRampRate(period);
    }

    @Override
    public void setOpenRampRate(double period) {
      driveMotor.setOpenLoopRampRate(period);
    }

    @Override
    public void stopMotor() {
        driveMotor.stopMotor();
        turningMotor.stopMotor();
      // for logging purposes
        desiredVolts = 0; 
        calcFeedForwardVoltsOverMetersPerSec = 0;
    }
}
