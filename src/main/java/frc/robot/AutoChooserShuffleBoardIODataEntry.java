package frc.robot;

import java.util.ArrayList;
import java.util.Arrays;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.utility.Auto;
import frc.utility.OrangeSendableChooser;

public class AutoChooserShuffleBoardIODataEntry implements AutoChooserShuffleBoardIO{
  private ShuffleboardTab customizationTab;
  private SendableChooser<Integer> positionChooser;
  private ArrayList<Auto> autoArrayList;
  private OrangeSendableChooser<Command> autoChooser = new OrangeSendableChooser<Command>();
  private final PathPlannerManager ppManager;
  private final Drive drive = new Drive();
  private int selectedPosition;
  

  public AutoChooserShuffleBoardIODataEntry() {
    // Set up auto routines
    ppManager = new PathPlannerManager(drive);
    
    if (Constants.autoChooserEnabled) {
      // new ShuffleBoard Tab
      customizationTab = Shuffleboard.getTab("Autos");

      // Widgets for Autos
      positionChooser = new SendableChooser<Integer>();
      positionChooser.addOption("1", 1);
      positionChooser.addOption("2", 2);
      positionChooser.addOption("3", 3);
      positionChooser.addOption("4", 4);
      positionChooser.addOption("5", 5);
      positionChooser.addOption("6", 6);
      positionChooser.addOption("7", 7);
      positionChooser.addOption("8", 8);
      positionChooser.addOption("9", 9);

      customizationTab.add("Position Chooser", positionChooser).withWidget(BuiltInWidgets.kSplitButtonChooser)
        .withPosition(0, 0).withSize(9, 2);

      customizationTab.add("Auto Chooser", autoChooser).withWidget(BuiltInWidgets.kSplitButtonChooser)
        .withPosition(0, 2).withSize(9, 2);
        
      loadAutos();
    }
  }

  public void loadAutos() {
    // Reset autoArrayList and selectedPosition
    autoArrayList.clear();
    selectedPosition = 0;

    autoArrayList.add(new Auto(
    "Move Forward", 
    ppManager.loadAuto("Move Forward", false),
    Arrays.asList(1, 2, 3, 4, 5, 6, 7, 8, 9)));
  }

  public void updateChoosers() {
    if (positionChooser.getSelected() != null) {
      if (positionChooser.getSelected() != selectedPosition) {
        selectedPosition = positionChooser.getSelected();
        autoChooser.removeAllOptions();
        for (Auto auto : autoArrayList) {
          if (auto.positions.contains(selectedPosition)) {
            autoChooser.addOption(auto.name, auto.command);
          }
        }
      }
    }
  }

  @Override
  public void updateInputs(AutoChooserShuffleBoardIOInputs inputs) {
    if (Constants.autoChooserEnabled) {
      inputs.position = positionChooser.getSelected();
      inputs.auto = autoChooser.getSelected();
    }
  }
}
