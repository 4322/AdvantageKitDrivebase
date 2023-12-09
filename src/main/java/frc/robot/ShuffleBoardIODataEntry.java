package frc.robot;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants.ControllerTypeStrings;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.InputScalingStrings;

public class ShuffleBoardIODataEntry implements ShuffleBoardIO {
  private ShuffleboardTab customizationTab;
  private GenericEntry closedRampRate;
  private GenericEntry openRampRate;
  private GenericEntry maxManualRotationEntry;
  private GenericEntry slowMovingAutoRotateEntry;
  private GenericEntry fastMovingAutoRotateEntry;
  private GenericEntry fastMovingFtPerSecEntry;
  private GenericEntry psuedoAutoRotateCheckbox;
  private SendableChooser<String> driveInputScaling;
  private SendableChooser<String> driveControlType;

  public ShuffleBoardIODataEntry() {

    if(Constants.debug) {
        //new shuffleboard tabs
      customizationTab = Shuffleboard.getTab("Drivebase Customization");
      
      //widgets for customizationTab
      psuedoAutoRotateCheckbox = customizationTab.add("Psuedo Auto Rotate", Constants.psuedoAutoRotateEnabled)
      .withWidget(BuiltInWidgets.kToggleButton).withPosition(0, 0).withSize(2, 1).getEntry();
      
      driveInputScaling = new SendableChooser<String>();
      driveInputScaling.addOption(InputScalingStrings.linear, InputScalingStrings.linear);
      driveInputScaling.setDefaultOption(InputScalingStrings.quadratic, InputScalingStrings.quadratic);
      driveInputScaling.addOption(InputScalingStrings.cubic, InputScalingStrings.cubic);

      customizationTab.add("Input Scaling", driveInputScaling).withWidget(BuiltInWidgets.kSplitButtonChooser)
          .withPosition(2, 0).withSize(3, 1);

      driveControlType = new SendableChooser<String>();
      driveControlType.addOption(ControllerTypeStrings.joysticks, ControllerTypeStrings.joysticks);
      driveControlType.setDefaultOption(ControllerTypeStrings.xboxLeftDrive, ControllerTypeStrings.xboxLeftDrive);
      driveControlType.addOption(ControllerTypeStrings.xboxRightDrive, ControllerTypeStrings.xboxRightDrive);

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
          
      closedRampRate = customizationTab.add("Acc Ramp Rate", DriveConstants.Drive.closedLoopRampSec)
          .withPosition(0, 2).withSize(1, 1).getEntry();

      openRampRate = customizationTab.add("Stop Ramp Rate", DriveConstants.Drive.openLoopRampSec)
          .withPosition(1, 2).withSize(1, 1).getEntry();  
    }
  }
    

  @Override
  public void updateInputs(ShuffleBoardIOInputs inputs) {
    if (Constants.shuffleboardEnabled) {
      inputs.psuedoAutoRotateEnabled = psuedoAutoRotateCheckbox.getBoolean(Constants.psuedoAutoRotateEnabled);
      inputs.inputScaling = driveInputScaling.getSelected();
      inputs.driveControllerType = driveControlType.getSelected();
      inputs.maxManualRotatePower = maxManualRotationEntry.getDouble(Constants.DriveConstants.Manual.maxManualRotation);
      inputs.slowMovingAutoRotatePower = slowMovingAutoRotateEntry.getDouble(Constants.DriveConstants.Auto.slowMovingAutoRotate);
      inputs.fastMovingAutoRotatePower = fastMovingAutoRotateEntry.getDouble(Constants.DriveConstants.Auto.fastMovingAutoRotate);
      inputs.fastMovingFtPerSec = fastMovingFtPerSecEntry.getDouble(Constants.DriveConstants.Auto.fastMovingFtPerSec);
      inputs.accelerationRampRate = closedRampRate.getDouble(DriveConstants.Drive.closedLoopRampSec);
      inputs.stoppedRampRate = openRampRate.getDouble(DriveConstants.Drive.openLoopRampSec);
    }
    else { 
      //if shuffleboard not enabled, don't want values to be 0
      inputs.psuedoAutoRotateEnabled = Constants.psuedoAutoRotateEnabled;
      inputs.inputScaling = Constants.driveInputScaling;
      inputs.driveControllerType = Constants.controllerType;
      inputs.maxManualRotatePower = Constants.DriveConstants.Manual.maxManualRotation;
      inputs.slowMovingAutoRotatePower = Constants.DriveConstants.Auto.slowMovingAutoRotate;
      inputs.fastMovingAutoRotatePower = Constants.DriveConstants.Auto.fastMovingAutoRotate;
      inputs.fastMovingFtPerSec = Constants.DriveConstants.Auto.fastMovingFtPerSec;
      inputs.accelerationRampRate = DriveConstants.Drive.closedLoopRampSec;
      inputs.stoppedRampRate = DriveConstants.Drive.openLoopRampSec;
    }
  }
}