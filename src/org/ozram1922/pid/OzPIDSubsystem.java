package org.ozram1922;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.tables.ITable;

/**
 * This class is designed to handle the case where there is a {@link Subsystem}
 * which uses a single {@link OzVDPIDController} almost constantly (for instance,
 * an elevator which attempts to stay at a constant height).
 *
 * <p>It provides some convenience methods to run an internal {@link OzVDPIDController}.
 * It also allows access to the internal {@link OzVDPIDController} in order to give total control
 * to the programmer.</p>
 *
 * @author Joe Grinstead
 */

 //TODO: Fix the documentation so its OURS, not just clearly copy-paste'd from the WPILib code
public abstract class OzPIDSubsystem extends Subsystem implements Sendable {

    /** The internal {@link OzVDPIDController} */
    private OzVDPIDController controller;
    /** An output which calls {@link PIDCommand#usePIDOutput(double)} */
    private PIDOutput output = new PIDOutput() {

        public void pidWrite(double output) {
            usePIDOutput(output);
        }
    };
    /** A source which calls {@link PIDCommand#returnPIDInput()} */
    private PIDSource source = new PIDSource() {

        public double pidGet() {
            return returnPIDInput();
        }
    };

    /**
     * Instantiates a {@link OzPIDSubsystem} that will use the given p, i and d values.
     * @param name the name
     * @param p the proportional value
     * @param i the integral value
     * @param d the derivative value
     */
    public OzPIDSubsystem(String name, double p, double i, double d) {
        super(name);
        controller = new OzVDPIDController(p, i, d, source, output);
    }

    /**
     * Instantiates a {@link OzPIDSubsystem} that will use the given p, i and d values.
     * @param name the name
     * @param p the proportional value
     * @param i the integral value
     * @param d the derivative value
     * @param f the feed forward value
     */
    public OzPIDSubsystem(String name, double p, double i, double d, double f) {
        super(name);
        controller = new OzVDPIDController(p, i, d, f, source, output);
    }

    /**
     * Instantiates a {@link OzPIDSubsystem} that will use the given p, i and d values.  It will also space the time
     * between PID loop calculations to be equal to the given period.
     * @param name the name
     * @param p the proportional value
     * @param i the integral value
     * @param d the derivative value
     * @param period the time (in seconds) between calculations
     */
    public OzPIDSubsystem(String name, double p, double i, double d, double f, double period) {
        super(name);
        controller = new OzVDPIDController(p, i, d, f, source, output, period);
    }

    /**
     * Instantiates a {@link OzPIDSubsystem} that will use the given p, i and d values.
     * It will use the class name as its name.
     * @param p the proportional value
     * @param i the integral value
     * @param d the derivative value
     */
    public OzPIDSubsystem(double p, double i, double d) {
        controller = new OzVDPIDController(p, i, d, source, output);
    }

    /**
     * Instantiates a {@link OzPIDSubsystem} that will use the given p, i and d values.
     * It will use the class name as its name.
     * It will also space the time
     * between PID loop calculations to be equal to the given period.
     * @param p the proportional value
     * @param i the integral value
     * @param d the derivative value
     * @param f the feed forward coefficient
     * @param period the time (in seconds) between calculations
     */
    public OzPIDSubsystem(double p, double i, double d, double period, double f) {
        controller = new OzVDPIDController(p, i, d, f, source, output, period);
    }

    /**
     * Instantiates a {@link OzPIDSubsystem} that will use the given p, i and d values.
     * It will use the class name as its name.
     * It will also space the time
     * between PID loop calculations to be equal to the given period.
     * @param p the proportional value
     * @param i the integral value
     * @param d the derivative value
     * @param period the time (in seconds) between calculations
     */
    public OzPIDSubsystem(double p, double i, double d, double period) {
        controller = new OzVDPIDController(p, i, d, source, output, period);
    }

    /**
     * Returns the {@link OzVDPIDController} used by this {@link OzPIDSubsystem}.
     * Use this if you would like to fine tune the pid loop.
     *
     * @return the {@link OzVDPIDController} used by this {@link OzPIDSubsystem}
     */
    public OzVDPIDController getPIDController() {
        return controller;
    }


    /**
     * Adds the given value to the setpoint.
     * If {@link OzPIDSubsystem#setInputRange(double, double) setInputRange(...)} was used,
     * then the bounds will still be honored by this method.
     * @param deltaSetpoint the change in the setpoint
     */
    public void setSetpointRelative(double deltaSetpoint) {
        setSetpoint(getPosition() + deltaSetpoint);
    }

    /**
     * Sets the setpoint to the given value.  If {@link OzPIDSubsystem#setInputRange(double, double) setInputRange(...)}
     * was called,
     * then the given setpoint
     * will be trimmed to fit within the range.
     * @param setpoint the new setpoint
     */
    public void setSetpoint(double setpoint) {
        controller.setSetpoint(setpoint);
    }

    /**
     * Returns the setpoint.
     * @return the setpoint
     */
    public double getSetpoint() {
        return controller.getSetpoint();
    }

    /**
     * Returns the current position
     * @return the current position
     */
    public double getPosition() {
        return returnPIDInput();
    }

    /**
     * Sets the maximum and minimum values expected from the input.
     *
     * @param minimumInput the minimum value expected from the input
     * @param maximumInput the maximum value expected from the output
     */
    public void setInputRange(double minimumInput, double maximumInput) {
        controller.setInputRange(minimumInput, maximumInput);
    }

    /**
     * Sets the maximum and minimum values to write.
     *
     * @param minimumOutput the minimum value to write to the output
     * @param maximumOutput the maximum value to write to the output
     */
    public void setOutputRange(double minimumOutput, double maximumOutput) {
        controller.setOutputRange(minimumOutput, maximumOutput);
    }

    /**
    * Set the absolute error which is considered tolerable for use with
    * OnTarget. The value is in the same range as the PIDInput values.
    * @param t the absolute tolerance
    */
    public void setAbsoluteTolerance(double t) {
        controller.setAbsoluteTolerance(t);
    }

    /**
    * Set the percentage error which is considered tolerable for use with
    * OnTarget. (Value of 15.0 == 15 percent)
    * @param p the percent tolerance
    */
    public void setPercentTolerance(double p) {
        controller.setPercentTolerance(p);
    }

    /**
     * Return true if the error is within the percentage of the total input range,
     * determined by setTolerance. This assumes that the maximum and minimum input
     * were set using setInput.
     * @return true if the error is less than the tolerance
     */
    public boolean onTarget() {
        return controller.onTarget();
    }

    /**
     * Returns the input for the pid loop.
     *
     * <p>It returns the input for the pid loop, so if this Subsystem was based
     * off of a gyro, then it should return the angle of the gyro</p>
     *
     * <p>All subclasses of {@link OzPIDSubsystem} must override this method.</p>
     *
     * @return the value the pid loop should use as input
     */
    protected abstract double returnPIDInput();

    /**
     * Uses the value that the pid loop calculated.  The calculated value is the "output" parameter.
     * This method is a good time to set motor values, maybe something along the lines of <code>driveline.tankDrive(output, -output)</code>
     *
     * <p>All subclasses of {@link OzPIDSubsystem} must override this method.</p>
     *
     * @param output the value the pid loop calculated
     */
    protected abstract void usePIDOutput(double output);

    /**
     * Enables the internal {@link OzVDPIDController}
     */
    public void enable() {
        controller.enable();
    }

    /**
     * Disables the internal {@link OzVDPIDController}
     */
    public void disable() {
        controller.disable();
    }

    public String getSmartDashboardType() {
        return "OzPIDSubsystem";
    }
    public void initTable(ITable table) {
        controller.initTable(table);
        super.initTable(table);
    }
}
