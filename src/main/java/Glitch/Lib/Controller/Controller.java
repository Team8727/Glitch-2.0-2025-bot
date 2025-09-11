package Glitch.Lib.Controller;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * Wrapper for a controller that can have a set of bindings applied to it.
 */
public class Controller {
  public enum Operator {
    MAIN,
    ASSIST,
  }

  private final Operator m_operator;

  private CommandXboxController m_controller;
  private ControllerBindings m_currentBindings;

  public Controller(Operator operator) {
    m_operator = operator;
    initController();
  }

  private int getPort() {
    switch (m_operator) {
      case MAIN:
        return 0;
      case ASSIST:
        return 1;
      default:
        throw new IllegalStateException("Unknown operator: " + m_operator);
    }
  }

  private void initController() {
        m_controller = new CommandXboxController(getPort());
    }

  public void applyBindings(ControllerBindings bindings) {
    initController();

    if (bindings != null) {
      bindings.bind(m_controller);
    }

    m_currentBindings = bindings;
  }

  public void clearBindings() {
    applyBindings(null);
  }

  public CommandXboxController getController() {
    return m_controller;
  }
}
