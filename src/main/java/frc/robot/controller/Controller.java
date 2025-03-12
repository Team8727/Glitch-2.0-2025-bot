package frc.robot.controller;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * Wrapper for a controller that can have a set of bindings applied to it.
 */
public class Controller {
    public enum Operator {
        MAIN,
        ASSIST
    }

    private final Operator m_operator;

    private CommandXboxController m_controller;
    private ControllerBindings m_currentBindings;

    public Controller(Operator operator) {
        m_operator = operator;
        initController();
    }

    private int getPort() {
        final int mainPort = 0;
        final int assistPort = 1;
        return m_operator == Operator.MAIN ? mainPort : assistPort;
    }

    private void initController() {
        m_controller = new CommandXboxController(getPort());
    }

    public void applyBindings(ControllerBindings bindings) {
        if (m_currentBindings != null) {
            m_currentBindings.unbind(m_controller);
        }

        initController();

        if (bindings != null) {
            bindings.bind(m_controller);
        }

        m_currentBindings = bindings;
    }

    public void clearBindings() {
        applyBindings(null);
    }
}
