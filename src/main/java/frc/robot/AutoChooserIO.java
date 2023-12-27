package frc.robot;

import java.util.ArrayList;
import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.utility.Auto;

public interface AutoChooserIO {
  @AutoLog
  public static class AutoChooserIOInputs {
    public int startingGridPosition;
    public Command autoCommand;
  }

  public default void updateInputs(AutoChooserIOInputs inputs) {}
  
  public default void updateChoosers() {}
}
