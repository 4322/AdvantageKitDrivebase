// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import java.util.ArrayList;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DriveManual;
import frc.robot.commands.DriveStop;
import frc.robot.commands.ResetFieldCentric;
import frc.robot.subsystems.drive.Drive;
import frc.utility.Auto;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private Timer disableTimer = new Timer();

  // Define controllers
  public static CommandXboxController xbox;
  public static Joystick driveStick;
  public static Joystick rotateStick;

  private JoystickButton driveButtonThree;
  private JoystickButton driveButtonSeven;
  private JoystickButton driveButtonTwelve;

  private ArrayList<Auto> autoArrayList = new ArrayList<Auto>();
  private SendableChooser<Integer> positionChooser = new SendableChooser<Integer>();

  private final Drive drive = new Drive(); 

  private final PathPlannerManager ppManager;

  private final DriveManual driveManualDefault = new DriveManual(drive, DriveManual.AutoPose.none);
  private final DriveStop driveStop = new DriveStop(drive);

  private int selectedPostion = 0;
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    drive.init();

    configureButtonBindings();

    if (Constants.driveEnabled) {
      drive.setDefaultCommand(driveManualDefault);
    }

    // Set up auto routines
    ppManager = new PathPlannerManager(drive);

    loadAutos();

    positionChooser.addOption("1", 1);
    positionChooser.addOption("2", 2);
    positionChooser.addOption("3", 3);
    positionChooser.addOption("4", 4);
    positionChooser.addOption("5", 5);
    positionChooser.addOption("6", 6);
    positionChooser.addOption("7", 7);
    positionChooser.addOption("8", 8);
    positionChooser.addOption("9", 9);
  }

  private void loadAutos() {

    // Reset autoArrayList and selectedPosition
    autoArrayList.clear();
    selectedPosition = 0;

    autoArrayList.add(new Auto("Do Nothing", null, null));

  }
  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    if (Constants.joysticksEnabled) {
      driveStick = new Joystick(0);
      rotateStick = new Joystick(1);
      
      driveButtonThree = new JoystickButton(driveStick, 3);
      driveButtonSeven = new JoystickButton(driveStick, 7);
      driveButtonTwelve = new JoystickButton(driveStick, 12);

      driveButtonThree.onTrue(new DriveManual(drive, DriveManual.AutoPose.usePresetAuto));
      driveButtonSeven.onTrue(new ResetFieldCentric(drive, 0, true));
      driveButtonTwelve.onTrue(driveStop);
    }

    if (Constants.xboxEnabled) {
      xbox = new CommandXboxController(2);
      xbox.povUp().onTrue(new ResetFieldCentric(drive, 0, true));
      xbox.rightBumper().onTrue(new DriveManual(drive, DriveManual.AutoPose.usePresetAuto));
      xbox.povDown().onTrue(driveStop);
    }
  }

  public void disabledPeriodic() {
    if (disableTimer.hasElapsed(Constants.DriveConstants.disableBreakSec)) {
      if (Constants.driveEnabled) {
        drive.setCoastMode(); // robot has stopped, safe to enter coast mode
      }
      disableTimer.stop();
      disableTimer.reset();
    }
  }

  public void enableSubsystems() {
    drive.setBrakeMode();
    disableTimer.stop();
    disableTimer.reset();
  }

  public void disableSubsystems() {
    driveStop.schedule();  // interrupt all drive commands
    disableTimer.reset();
    disableTimer.start();
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
