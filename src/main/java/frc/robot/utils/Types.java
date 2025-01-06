package frc.robot.utils;


/**
 * @brief A class for creating convenience type wrappers to allow us to more easily pass around values.
 */
public final class Types {
  public static final class PidConstants {
    public  double p; 
    public  double i;
    public  double d;

    public PidConstants(double p, double i, double d) {
      this.p = p;
      this.i = i;
      this.d = d;
    }
  }

  public static final class FeedForwardConstants {
    public final double ks;
    public final double kv;
    public final double ka;
    public final double kg;

    public FeedForwardConstants(double ks, double kv, double ka) {
      this(ks, kv, ka, 0);
    }
    public FeedForwardConstants(double ks, double kv, double ka, double kg) {
      this.ks = ks;
      this.kv = kv;
      this.ka = ka;
      this.kg = kg;
    }
  }

  public static final class Limits {
    public final double low;
    public final double high;


    public Limits(double low, double high) {
      this.low = low;
      this.high = high;
    }
  }


  public enum LEDState{
    IN_RANGE,
    HAVE_NOTE,
    NO_NOTE,
    NONE
}
  public enum DirectionType{
    UP,
    DOWN
  };

  public enum PositionType{
    TOP,
    BOTTOM,
    LEFT,
    RIGHT
  }

  public enum SysidMechanism{
    NONE,
    INTAKE_TOP,
    INTAKE_BOTTOM,
    DRIVE,
    SHOOTER_LEFT,
    SHOOTER_RIGHT,
    KICKUP_RIGHT,
    ARM
  }

  public enum GoalType{
    SPEAKER,
    AMP
}

}
