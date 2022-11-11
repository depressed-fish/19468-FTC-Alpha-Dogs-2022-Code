package org.firstinspires.ftc.teamcode;

public class PIDController {
    private double m_kp;
    private double m_ki;
    private double m_kd;
    private double m_period;
    private double m_positionTolerance;
    private double m_velocityTolerance;
    private double m_positionError;
    private double m_velocityError;
    private double m_prevError;
    private double m_totalError;
    private double m_setpoint;
    private double m_measurement;
    private double m_minimumIntegral = -1.0;
    private double m_maximumIntegral = 1.0;

    public PIDController(double kp, double ki, double kd) {
        m_kp = kp;
        m_ki = ki;
        m_kd = kd;
        m_period = 0.2;
    }
    /**
     * Sets the PID Controller gain parameters.
     *
     * <p>Set the proportional, integral, and differential coefficients.
     *
     * @param kp The proportional coefficient.
     * @param ki The integral coefficient.
     * @param kd The derivative coefficient.
     */
    public void setPID(double kp, double ki, double kd) {
        m_kp = kp;
        m_ki = ki;
        m_kd = kd;
    }

    /**
     * Sets the Proportional coefficient of the PID controller gain.
     *
     * @param kp proportional coefficient
     */
    public void setP(double kp) {
        m_kp = kp;
    }

    /**
     * Sets the Integral coefficient of the PID controller gain.
     *
     * @param ki integral coefficient
     */
    public void setI(double ki) {
        m_ki = ki;
    }

    /**
     * Sets the Differential coefficient of the PID controller gain.
     *
     * @param kd differential coefficient
     */
    public void setD(double kd) {
        m_kd = kd;
    }

    /**
     * Get the Proportional coefficient.
     *
     * @return proportional coefficient
     */
    public double getP() {
        return m_kp;
    }

    /**
     * Get the Integral coefficient.
     *
     * @return integral coefficient
     */
    public double getI() {
        return m_ki;
    }

    /**
     * Get the Differential coefficient.
     *
     * @return differential coefficient
     */
    public double getD() {
        return m_kd;
    }

    /**
     * Returns the period of this controller.
     *
     * @return the period of the controller.
     */
    public double getPeriod() {
        return m_period;
    }

    /**
     * Sets the setpoint for the PIDController.
     *
     * @param setpoint The desired setpoint.
     */
    public void setSetpoint(double setpoint) {
        m_setpoint = setpoint;
    }

    /**
     * Returns the current setpoint of the PIDController.
     *
     * @return The current setpoint.
     */
    public double getSetpoint() {
        return m_setpoint;
    }
    /**
     * Sets the error which is considered tolerable for use with atSetpoint().
     *
     * @param positionTolerance Position error which is tolerable.
     */
    public void setTolerance(double positionTolerance) {
        setTolerance(positionTolerance, Double.POSITIVE_INFINITY);
    }

    /**
     * Sets the error which is considered tolerable for use with atSetpoint().
     *
     * @param positionTolerance Position error which is tolerable.
     * @param velocityTolerance Velocity error which is tolerable.
     */
    public void setTolerance(double positionTolerance, double velocityTolerance) {
        m_positionTolerance = positionTolerance;
        m_velocityTolerance = velocityTolerance;
    }
    /**
     * Returns true if the error is within the tolerance of the setpoint.
     *
     * <p>This will return false until at least one input value has been computed.
     *
     * @return Whether the error is within the acceptable bounds.
     */
    public boolean atSetpoint() {
        double positionError;

        positionError = m_setpoint - m_measurement;

        double velocityError = (positionError - m_prevError) / m_period;

        return Math.abs(positionError) < m_positionTolerance
                && Math.abs(velocityError) < m_velocityTolerance;
    }
    /**
     * Returns the next output of the PID controller.
     *
     * @param measurement The current measurement of the process variable.
     * @return The next controller output.
     */
    public double calculate(double measurement) {
        m_measurement = measurement;
        m_prevError = m_positionError;

        m_positionError = m_setpoint - measurement;

        m_velocityError = (m_positionError - m_prevError) / m_period;

        if (m_ki != 0) {
            m_totalError =
                    Math.max(m_minimumIntegral / m_ki, Math.min(m_maximumIntegral / m_ki, m_totalError + m_positionError * m_period));

        }

        return m_kp * m_positionError + m_ki * m_totalError + m_kd * m_velocityError;
    }

    /** Resets the previous error and the integral term. */
    public void reset() {
        m_prevError = 0;
        m_totalError = 0;
    }
}
