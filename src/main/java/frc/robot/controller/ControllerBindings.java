package frc.robot.controller;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * Interface for binding commands to a controller.
 */
public interface ControllerBindings {
  /**
  * Bind your commands to the controller here.
  * @param controller The controller to bind to.
  */
  void bind(CommandXboxController controller);

  /**
  * Unbind your commands from the controller here, and do any other necessary cleanup.
  * @param controller The controller to unbind from.
  */
  void unbind(CommandXboxController controller);
}
