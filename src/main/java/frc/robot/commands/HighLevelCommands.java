package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/**
 * This class is used to automate the 'superstructure' of our robot - this is where any automation
 * should go where commands are built up and abstracted away into a single command.
 *
 * <p>To a degree, each of these commands is a "state" activated and cordinates the superstructure
 * (and drivesubsystem) accordingly.
 */
public class HighLevelCommands {

  public static Command exampleCommandDynamicAuton(Integer exampleParam) {
    return Commands.print("EA Sports: It's in the game! Example Param: " + exampleParam);
  }
}
