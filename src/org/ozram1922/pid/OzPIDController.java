package org.ozram1922;

import java.util.TimerTask;

import edu.wpi.first.wpilibj.livewindow.LiveWindowSendable;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.tables.ITable;
import edu.wpi.first.wpilibj.tables.ITableListener;
import edu.wpi.first.wpilibj.util.BoundaryException;

/**
* Class implements a unit within a variable-differential PID Control Loop.
*
* Designed to be run from a separate thread on OzVDPIDController each cycle, NOT to be used on its own
*/

//TODO: Make this work as a single unit not to have its own thread
class OzPIDController implements LiveWindowSendable
{

  //public static final double msDefaultPeriod = .05;
  //private static int msInstances = 0;
  private double mP;     // factor for "proportional" control
  private double mI;     // factor for "integral" control
  private double mD;     // factor for "derivative" control
  private double mF;                 // factor for feedforward term
  //private double mMaximumOutput = 1.0; // |maximum output|
  //private double mMinimumOutput = -1.0;  // |minimum output|
  //private double mMaximumInput = 0.0;    // maximum input - limit setpoint to this
  //private double mMinimumInput = 0.0;    // minimum input - limit setpoint to this
  //private boolean mContinuous = false; // do the endpoints wrap around? eg. Absolute encoder
  //private boolean mEnabled = false;    //is the pid controller enabled
  private double mPrevError = 0.0; // the prior sensor input (used to compute velocity)
  private double mTotalError = 0.0; //the sum of the errors for use in the integral calc
  private Tolerance mTolerance;  //the tolerance object used to check if on target
  //private double mSetpoint = 0.0;
  private double mError = 0.0;
  private double mResult = 0.0;
  //private double mPeriod = msDefaultPeriod;
  //PIDSource mPidInput;
  //PIDOutput mPidOutput;
  //java.util.Timer mControlLoop;
  //private boolean mFreed = false;
  private boolean mUsingPercentTolerance;


  /**
  * Allocate a PID object with the given constants for P, I, D, and F
  * @param Kp the proportional coefficient
  * @param Ki the integral coefficient
  * @param Kd the derivative coefficient
  * @param Kf the feed forward term
  * @param source The PIDSource object that is used to get values
  * @param output The PIDOutput object that is set to the output percentage
  * @param period the loop time for doing calculations. This particularly effects calculations of the
  * integral and differential terms. The default is 50ms.
  */
  public OzPIDController(PIDCreateStruct initInfo)
  {
    mP = initInfo.mP;
    mI = initInfo.mI;
    mD = initInfo.mD;
    mF = initInfo.mF;

    mTolerance = initInfo.mTolerance;
  }
  /**
  * Tolerance is the type of tolerance used to specify if the PID controller is on target.
  * The various implementations of this class such as PercentageTolerance and AbsoluteTolerance
  * specify types of tolerance specifications to use.
  */
  public interface Tolerance {
    public boolean onTarget(double input);
  }

  public class PercentageTolerance implements Tolerance {
    double percentage;

    PercentageTolerance(double value) {
      percentage = value;
    }

    @Override
    public boolean onTarget(double input) {
      return (Math.abs(getError(input)) < percentage / 100
      * (mMaximumInput - mMinimumInput));
    }
  }

  public class AbsoluteTolerance implements Tolerance {
    double value;

    AbsoluteTolerance(double value) {
      this.value = value;
    }

    @Override
    public boolean onTarget(double input) {
      return Math.abs(getError(input)) < value;
    }
  }

  public class NullTolerance implements Tolerance {

    @Override
    public boolean onTarget(double input) {
      throw new RuntimeException("No tolerance value set when using OzPIDController.onTarget()");
    }
  }

  /**
  * Returns the current difference of the input from the setpoint
  * @return the current error
  */
  public synchronized double getError(double input) {
    //return mError;
    return getSetpoint() - input;
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
  *
  * returns the PID output
  * input represents the value of the pre-determined 'nth' derivative of the input
  */
  private double calculate(double input) {

    if (mEnabled)
    {
      mError = mSetpoint - input;
      if (mContinuous) {
        if (Math.abs(mError) > (mMaximumInput - mMinimumInput) / 2) {
          if (mError > 0) {
            mError = mError - mMaximumInput + mMinimumInput;
          } else {
            mError = mError + mMaximumInput - mMinimumInput;
          }
        }
      }

      if (mI != 0) {
        double potentialIGain = (mTotalError + mError) * mI;
        if (potentialIGain < mMaximumOutput) {
          if (potentialIGain > mMinimumOutput) {
            mTotalError += mError;
          } else {
            mTotalError = mMinimumOutput / mI;
          }
        } else {
          mTotalError = mMaximumOutput / mI;
        }
      }

      mResult = mP * mError + mI * mTotalError + mD * (mError - mPrevError) + mSetpoint * mF;
      mPrevError = mError;

      result = mResult;
    }

    return mResult;
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
* Set the setpoint for the OzPIDController
* @param setpoint the desired setpoint
*/
public synchronized void setSetpoint(double setpoint) {
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

/** Set the PID tolerance using a Tolerance object.
* Tolerance can be specified as a percentage of the range or as an absolute
* value. The Tolerance object encapsulates those options in an object. Use it by
* creating the type of tolerance that you want to use: setTolerance(new OzPIDController.AbsoluteTolerance(0.1))
* @param tolerance a tolerance object of the right type, e.g. PercentTolerance
* or AbsoluteTolerance
*/
private synchronized void setTolerance(Tolerance tolerance) {
  mTolerance = tolerance;
}

/**
* Set the absolute error which is considered tolerable for use with
* OnTarget.
* @param absvalue absolute error which is tolerable in the units of the input object
*/
public synchronized void setAbsoluteTolerance(double absvalue) {
  mTolerance = new AbsoluteTolerance(absvalue);
}

/**
* Set the percentage error which is considered tolerable for use with
* OnTarget. (Input of 15.0 = 15 percent)
* @param percentage percent error which is tolerable
*/
public synchronized void setPercentTolerance(double percentage) {
  mTolerance = new PercentageTolerance(percentage);
}

/**
* Return true if the error is within the percentage of the total input range,
* determined by setTolerance. This assumes that the maximum and minimum input
* were set using setInput.
* @return true if the error is less than the tolerance
*/
public synchronized boolean onTarget() {
  return mTolerance.onTarget();
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
  mPrevError = 0;
  mTotalError = 0;
  mResult = 0;
}

@Override
public String getSmartDashboardType() {
  return "OzPIDController";
}

private final ITableListener listener = new ITableListener() {
  @Override
  public void valueChanged(ITable table, String key, Object value, boolean isNew) {
    if (key.equals("p") || key.equals("i") || key.equals("d") || key.equals("f")) {
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
    }
  }
};
private ITable table;
@Override
public void initTable(ITable table) {
  if(this.table!=null)
  this.table.removeTableListener(listener);
  this.table = table;
  if(table!=null) {
    table.putNumber("p", getP());
    table.putNumber("i", getI());
    table.putNumber("d", getD());
    table.putNumber("f", getF());
    table.putNumber("setpoint", getSetpoint());
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
}

/**
* {@inheritDoc}
*/
@Override
public void stopLiveWindowMode() {
}
}
