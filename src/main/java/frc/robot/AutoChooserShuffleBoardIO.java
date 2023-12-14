package frc.robot;

import java.util.ArrayList;
import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.utility.Auto;

public interface AutoChooserShuffleBoardIO {
  @AutoLog
  public static class AutoChooserShuffleBoardIOInputs {
    public ArrayList<Auto> autoArrayList;
    public int position;
  }

  public default void updateInputs(AutoChooserShuffleBoardIOInputs inputs) {}

  public default void loadAutos() {}

  public default void updateChoosers() {}
}
