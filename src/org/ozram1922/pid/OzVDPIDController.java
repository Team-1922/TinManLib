package org.ozram1922;


import java.util.TimerTask;

import edu.wpi.first.wpilibj.livewindow.LiveWindowSendable;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.tables.ITable;
import edu.wpi.first.wpilibj.tables.ITableListener;
import edu.wpi.first.wpilibj.util.BoundaryException;

//TODO: THis will run its own thread controlling several OzPIDController instances, depending on which target it needs to meet
public class OzVDPIDController implements LiveWindowSendable, OzController
{

  public static final double msDefaultPeriod = .05;
  private static int msInstances = 0;
  //private double mP;     // factor for "proportional" control
  //private double mI;     // factor for "integral" control
  //private double mD;     // factor for "derivative" control
  //private double mF;                 // factor for feedforward term
  private double mMaximumOutput = 1.0; // |maximum output|
  private double mMinimumOutput = -1.0;  // |minimum output|
  private double mMaximumInput = 0.0;    // maximum input - limit setpoint to this
  private double mMinimumInput = 0.0;    // minimum input - limit setpoint to this
  private boolean mContinuous = false; // do the endpoints wrap around? eg. Absolute encoder
  private boolean mEnabled = false;    //is the pid controller enabled
  //private double mPrevError = 0.0; // the prior sensor input (used to compute velocity)
  //private double mTotalError = 0.0; //the sum of the errors for use in the integral calc
  //private Tolerance mTolerance;  //the tolerance object used to check if on target
  //private double mSetpoint = 0.0;
  private CompoundSetpoint mSetpoint;
  //private double mError = 0.0;
  //private double mResult = 0.0;
  private double mPeriod = msDefaultPeriod;
  PIDSource mPidInput;
  PIDOutput mPidOutput;
  java.util.Timer mControlLoop;
  private boolean mFreed = false;
  private boolean mUsingPercentTolerance;
  private OzPIDController[] mPIDControllers;
  private int mActiveControllerIndex = -1;

  public class CompoundSetpoint {
    public final double[] mSetpointList;//TODO: I don't know how to make an equivilent to 'const'

    public CompountSetpoint(double[] setpointList)
    {
      mSetpointList = setpointList;
    }
  }

  public class PIDCreateStruct{
    public final double mP;
    public final double mI;
    public final double mD;
    public final double mF;
    public final Tolerance mTolerance;
  }



  private class PIDTask extends TimerTask {

    private OzPIDController mController;

    public PIDTask(OzPIDController controller) {
      if (controller == null) {
        throw new NullPointerException("Given OzPIDController was null");
      }
      mController = controller;
    }

    @Override
    public void run() {
      mController.calculate();
    }
  }

  /**
  * Allocate a PID controller
  * @param maxDifferentialPower The maximum number of differential levels to control across
  * @param source The PIDSource object that is used to get values
  * @param output The PIDOutput object that is set to the output percentage
  * @param period the loop time for doing calculations. This particularly effects calculations of the
  * integral and differential terms. The default is 50ms.
  */
  public OzVDPIDController(int maxDifferentialPower, PIDCreateStruct[] pidInitInfo,
  PIDSource source, PIDOutput output,
  double period) {

    if (source == null) {
      throw new NullPointerException("Null PIDSource was given");
    }
    if (output == null) {
      throw new NullPointerException("Null PIDOutput was given");
    }

    mControlLoop = new java.util.Timer();



    mPidInput = source;
    mPidOutput = output;
    mPeriod = period;


    //TODO :Initialize PID Controllers here
    mPIDControllers = new OzPIDController[maxDifferentialPower];
    if(maxDifferentialPower != Array.getLength(pidInitInfo))
    {
      throw new IllegalArgumentException("PID Differentiation Power does not Match given length of PID info list");
    }
    for (int i = 0; i < maxDifferentialPower; ++i)
    {
      mPIDControllers[i] = new OzPIDController(pidInitInfo[i]);
    }

    mControlLoop.schedule(new PIDTask(this), 0L, (long) (mPeriod * 1000));

    msInstances++;
    //HLUsageReporting.reportPIDController(msInstances);
    mTolerance = new NullTolerance();
  }

  /**
  * Allocate a PID object with the given constants for P, I, D and period
  * @param maxDifferentialPower The maximum number of differential levels to control across
  * @param source the PIDSource object that is used to get values
  * @param output the PIDOutput object that is set to the output percentage
  * @param period the loop time for doing calculations. This particularly effects calculations of the
  * integral and differential terms. The default is 50ms.
  */
  public OzVDPIDController(int maxDifferentialPower,
  PIDSource source, PIDOutput output,
  double period) {
    this(maxDifferentialPower, 0.0, source, output, period);
  }

  /**
  * Allocate a PID object with the given constants for P, I, D, using a 50ms period.
  * @param maxDifferentialPower The maximum number of differential levels to control across
  * @param source The PIDSource object that is used to get values
  * @param output The PIDOutput object that is set to the output percentage
  */
  public OzVDPIDController(int maxDifferentialPower,
  PIDSource source, PIDOutput output) {
    this(maxDifferentialPower, source, output, msDefaultPeriod);
  }

  /**
  * Free the PID object
  */
  public void free() {
    mControlLoop.cancel();
    synchronized (this) {
      mFreed = true;
      mPidOutput = null;
      mPidInput = null;
      mControlLoop = null;
    }
    if(this.table!=null) table.removeTableListener(listener);
  }

  /**
  * Read the input, calculate the output accordingly, and write to the output.
  * This should only be called by the PIDTask
  * and is created during initialization.
  */
  private void calculate() {
    boolean enabled;
    PIDSource pidInput;

    synchronized (this) {
      if (mPidInput == null) {
        return;
      }
      if (mPidOutput == null) {
        return;
      }
      enabled = mEnabled; // take snapshot of these values...
      pidInput = mPidInput;
    }

    if (enabled) {
      double input;
      double result;
      PIDOutput pidOutput = null;
      synchronized (this)
      {
        input = pidInput.pidGet();
      }
      synchronized (this)
      {
        //TODO: calculate which PID controller should do the calculation, and reset it accordingly
        for(int n = 0; n < mPIDControllers; ++n)
        {
          //get the nth derivative of the input to see if the target is on target
          if(mPIDControllers[n].onTarget())
          {

          }
        }
      }

      result = mP * mError + mI * mTotalError + mD * (mError - mPrevError) + mSetpoint * mF;
      mPrevError = mError;

      if (result > mMaximumOutput) {
        result = mMaximumOutput;
      } else if (result < mMinimumOutput) {
        result = mMinimumOutput;
      }
      synchronized(this)
      {
        mPidOutput.pidWrite(result);
      }
    }

  }


  /**
  * Set the PID Controller gain parameters.
  * Set the proportional, integral, and differential coefficients.
  * @param p Proportional coefficient
  * @param i Integral coefficient
  * @param d Differential coefficient
  */
  public synchronized void setPID(double p, double i, double d) {
    mP = p;
    mI = i;
    mD = d;

    if (table != null) {
      table.putNumber("p", p);
      table.putNumber("i", i);
      table.putNumber("d", d);
    }
  }

  /**
  * Set the PID Controller gain parameters.
  * Set the proportional, integral, and differential coefficients.
  * @param p Proportional coefficient
  * @param i Integral coefficient
  * @param d Differential coefficient
  * @param f Feed forward coefficient
  */
  public synchronized void setPID(double p, double i, double d, double f) {
    mP = p;
    mI = i;
    mD = d;
    mF = f;

    if (table != null) {
      table.putNumber("p", p);
      table.putNumber("i", i);
      table.putNumber("d", d);
      table.putNumber("f", f);
    }
  }

  /**
  * Get the Proportional coefficient
  * @return proportional coefficient
  */
  public synchronized double getP() {
    return mP;
  }

  /**
  * Get the Integral coefficient
  * @return integral coefficient
  */
  public synchronized double getI() {
    return mI;
  }

  /**
  * Get the Differential coefficient
  * @return differential coefficient
  */
  public synchronized double getD() {
    return mD;
  }

  /**
  * Get the Feed forward coefficient
  * @return feed forward coefficient
  */
  public synchronized double getF() {
    return mF;
  }

  /**
  * Return the current PID result
  * This is always centered on zero and constrained the the max and min outs
  * @return the latest calculated output
  */
  public synchronized double get() {
    return mResult;
  }

  /**
  *  Set the PID controller to consider the input to be continuous,
  *  Rather then using the max and min in as constraints, it considers them to
  *  be the same point and automatically calculates the shortest route to
  *  the setpoint.
  * @param continuous Set to true turns on continuous, false turns off continuous
  */
  public synchronized void setContinuous(boolean continuous) {
    mContinuous = continuous;
  }

  /**
  *  Set the PID controller to consider the input to be continuous,
  *  Rather then using the max and min in as constraints, it considers them to
  *  be the same point and automatically calculates the shortest route to
  *  the setpoint.
  */
  public synchronized void setContinuous() {
    this.setContinuous(true);
  }

  /**
  * Sets the maximum and minimum values expected from the input and setpoint.
  *
  * @param minimumInput the minimum value expected from the input
  * @param maximumInput the maximum value expected from the input
  */
  public synchronized void setInputRange(double minimumInput, double maximumInput) {
    if (minimumInput > maximumInput) {
      throw new BoundaryException("Lower bound is greater than upper bound");
    }
    mMinimumInput = minimumInput;
    mMaximumInput = maximumInput;
    setSetpoint(mSetpoint);
  }

  /**
  * Sets the minimum and maximum values to write.
  *
  * @param minimumOutput the minimum percentage to write to the output
  * @param maximumOutput the maximum percentage to write to the output
  */
  public synchronized void setOutputRange(double minimumOutput, double maximumOutput) {
    if (minimumOutput > maximumOutput) {
      throw new BoundaryException("Lower bound is greater than upper bound");
    }
    mMinimumOutput = minimumOutput;
    mMaximumOutput = maximumOutput;
  }

  /**
  * Set the setpoint for the OzPIDController
  * @param setpoint the desired setpoint
  */
  //TODO: This has updated parameters, but NO updated logic
  public synchronized void setSetpoint(CompoundSetpoint setpoint) {
    if (mMaximumInput > mMinimumInput) {
      if (setpoint > mMaximumInput) {
        mSetpoint = mMaximumInput;
      } else if (setpoint < mMinimumInput) {
        mSetpoint = mMinimumInput;
      } else {
        mSetpoint = setpoint;
      }
    } else {
      mSetpoint = setpoint;
    }

    if (table != null)
    table.putNumber("setpoint", mSetpoint);
  }

  /**
  * Returns the current setpoint of the OzPIDController
  * @return the current setpoint
  */
  public synchronized double getSetpoint() {
    return mSetpoint;
  }

  /**
  * Returns the current difference of the input from the setpoint
  * @return the current error
  */
  public synchronized double getError() {
    //return mError;
    return getSetpoint() - mPidInput.pidGet();
  }

  /**
  * Return true if the error is within the percentage of the total input range,
  * determined by setTolerance. This assumes that the maximum and minimum input
  * were set using setInput.
  * @return true if the error is less than the tolerance
  */
  public synchronized boolean onTarget() {
    return mPIDControllers[0].onTarget(getNDerivInput());
  }

  /**
  * Begin running the OzPIDController
  */
  @Override
  public synchronized void enable() {
    mEnabled = true;

    if (table != null) {
      table.putBoolean("enabled", true);
    }
  }

  /**
  * Stop running the OzPIDController, this sets the output to zero before stopping.
  */
  @Override
  public synchronized void disable() {
    mPidOutput.pidWrite(0);
    mEnabled = false;

    if (table != null) {
      table.putBoolean("enabled", false);
    }
  }

  /**
  * Return true if OzPIDController is enabled.
  */
  public synchronized boolean isEnable() {
    return mEnabled;
  }

  /**
  * Reset the previous error,, the integral term, and disable the controller.
  */
  public synchronized void reset() {
    disable();
    mPrevError = 0;
    mTotalError = 0;
    mResult = 0;
  }

  @Override
  public String getSmartDashboardType() {
    return "OzVDPIDController";
  }

  private final ITableListener listener = new ITableListener() {
    @Override
    public void valueChanged(ITable table, String key, Object value, boolean isNew) {
      /*if (key.equals("p") || key.equals("i") || key.equals("d") || key.equals("f")) {
        if (getP() != table.getNumber("p", 0.0) || getI() != table.getNumber("i", 0.0) ||
        getD() != table.getNumber("d", 0.0) || getF() != table.getNumber("f", 0.0))
        setPID(table.getNumber("p", 0.0), table.getNumber("i", 0.0), table.getNumber("d", 0.0), table.getNumber("f", 0.0));
      } else if (key.equals("setpoint")) {
        if (getSetpoint() != ((Double) value).doubleValue())
        setSetpoint(((Double) value).doubleValue());
      } else if (key.equals("enabled")) {
        if (isEnable() != ((Boolean) value).booleanValue()) {
          if (((Boolean) value).booleanValue()) {
            enable();
          } else {
            disable();
          }
        }
      }*/
    }
  };
  private ITable table;
  @Override
  public void initTable(ITable table) {
    if(this.table!=null)
    this.table.removeTableListener(listener);
    this.table = table;
    if(table!=null) {
      //table.putNumber("p", getP());
      //table.putNumber("i", getI());
      //table.putNumber("d", getD());
      //table.putNumber("f", getF());
      //table.putNumber("setpoint", getSetpoint());
      table.putBoolean("enabled", isEnable());
      table.addTableListener(listener, false);
    }
  }

  /**
  * {@inheritDoc}
  */
  @Override
  public ITable getTable() {
    return table;
  }

  /**
  * {@inheritDoc}
  */
  @Override
  public void updateTable() {
  }

  /**
  * {@inheritDoc}
  */
  @Override
  public void startLiveWindowMode() {
    disable();
  }

  /**
  * {@inheritDoc}
  */
  @Override
  public void stopLiveWindowMode() {
  }
}
