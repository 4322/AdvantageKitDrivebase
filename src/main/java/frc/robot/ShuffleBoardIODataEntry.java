package frc.robot;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Relay.InvalidValueException;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveConstants.Manual.ControllerType;
import frc.robot.subsystems.drive.SwerveModule;

public class ShuffleBoardIODataEntry implements ShuffleBoardIO {
  private ShuffleboardTab customizationTab;
  private GenericEntry closedRampRate;
  private GenericEntry openRampRate;
  private GenericEntry maxManualRotationEntry;
  private GenericEntry slowMovingAutoRotateEntry;
  private GenericEntry fastMovingAutoRotateEntry;
  private GenericEntry fastMovingFtPerSecEntry;
  private GenericEntry psuedoAutoRotateCheckbox;
  private SendableChooser<Integer> driveInputScaling;
  private SendableChooser<ControllerType> driveControlType;
  private double lastClosedRampRate = DriveConstants.Drive.closedLoopRampSec;
  private double lastOpenRampRate = DriveConstants.Drive.openLoopRampSec;
  private SwerveModule[] swerveModules = new SwerveModule[4];

  public ShuffleBoardIODataEntry() {
    customizationTab = Shuffleboard.getTab("Drivebase Customization");
    psuedoAutoRotateCheckbox = customizationTab.add("Psuedo Auto Rotate", Constants.psuedoAutoRotateEnabled)
    .withWidget(BuiltInWidgets.kToggleButton).withPosition(0, 0).withSize(2, 1).getEntry();
    
    driveInputScaling = new SendableChooser<Integer>();
    driveInputScaling.addOption("Linear", 1);
    driveInputScaling.setDefaultOption("Quadratic", 2);
    driveInputScaling.addOption("Cubic", 3);

    customizationTab.add("Input Scaling", driveInputScaling).withWidget(BuiltInWidgets.kSplitButtonChooser)
        .withPosition(2, 0).withSize(3, 1);

    driveControlType = new SendableChooser<ControllerType>();
    driveControlType.addOption("Joysticks", Constants.DriveConstants.Manual.ControllerType.JOYSTICKS);
    driveControlType.setDefaultOption("Xbox Left Drive", Constants.DriveConstants.Manual.ControllerType.XBOXLEFTDRIVE);
    driveControlType.addOption("Xbox Right Drive", Constants.DriveConstants.Manual.ControllerType.XBOXRIGHTDRIVE);

    customizationTab.add("Drive Control", driveControlType).withWidget(BuiltInWidgets.kSplitButtonChooser)
      .withPosition(5, 0).withSize(3, 1);

    maxManualRotationEntry = customizationTab.add("Max Manual Rotate Power", 
        Constants.DriveConstants.Manual.maxManualRotation)
        .withPosition(0, 1).withSize(2, 1).getEntry();

    slowMovingAutoRotateEntry = customizationTab.add("Slow Moving Auto Rotate Power", 
        Constants.DriveConstants.Auto.slowMovingAutoRotate)
        .withPosition(2, 1).withSize(2, 1).getEntry();

    fastMovingAutoRotateEntry = customizationTab.add("Fast Moving Auto Rotate Power", 
        Constants.DriveConstants.Auto.fastMovingAutoRotate)
        .withPosition(4, 1).withSize(2, 1).getEntry();

    fastMovingFtPerSecEntry = customizationTab.add("Fast Moving Ft Per Sec", 
        Constants.DriveConstants.Auto.fastMovingFtPerSec)
        .withPosition(6, 1).withSize(2, 1).getEntry();
        
    closedRampRate = customizationTab.add("Acc Ramp Rate", lastClosedRampRate)
        .withPosition(0, 2).withSize(1, 1).getEntry();

    openRampRate = customizationTab.add("Stop Ramp Rate", lastOpenRampRate)
        .withPosition(1, 2).withSize(1, 1).getEntry();  
  }

  @Override
  public void updateInputs(ShuffleBoardIOInputs inputs) {
    inputs.psuedoAutoRotateCheckboxEnabled = psuedoAutoRotateCheckbox.getBoolean(Constants.psuedoAutoRotateEnabled);
    inputs.inputScaling = driveInputScaling.getSelected().toString();
    inputs.driveControllerType = getString(driveControlType.getSelected());
    inputs.maxManualRotatePower = maxManualRotationEntry.getDouble(Constants.DriveConstants.Manual.maxManualRotation);
    inputs.slowMovingAutoRotatePower = slowMovingAutoRotateEntry.getDouble(Constants.DriveConstants.Auto.slowMovingAutoRotate);
    inputs.fastMovingAutoRotatePower = fastMovingAutoRotateEntry.getDouble(Constants.DriveConstants.Auto.fastMovingAutoRotate);
    inputs.fastMovingFtPerSec = fastMovingFtPerSecEntry.getDouble(Constants.DriveConstants.Auto.fastMovingFtPerSec);
    inputs.accelerationRampRate = closedRampRate.getDouble(lastClosedRampRate);
    inputs.openRampRate = openRampRate.getDouble(lastOpenRampRate);
  }

  public void changeRampRate() {
    double newRampRate = closedRampRate.getDouble(lastClosedRampRate);
    if (lastClosedRampRate != newRampRate) {
      lastClosedRampRate = newRampRate;
      for (SwerveModule module : swerveModules) {
        module.setClosedRampRate(newRampRate);
      }
    }
  }

  private String getString(ControllerType controllerType){
     switch (controllerType) {
      case NONE:
        return "NONE";
      case XBOXLEFTDRIVE:
        return "XBOXLEFTDRIVE";
      case XBOXRIGHTDRIVE:
        return "XBOXRIGHTDRIVE";
      case JOYSTICKS:
        return "JOYSTICKS";
      default:
        return "";
    }
  }
}
