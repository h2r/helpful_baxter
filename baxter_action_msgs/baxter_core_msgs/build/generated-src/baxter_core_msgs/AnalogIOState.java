package baxter_core_msgs;

public interface AnalogIOState extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "baxter_core_msgs/AnalogIOState";
  static final java.lang.String _DEFINITION = "time timestamp\nfloat64 value\nbool isInputOnly\n";
  org.ros.message.Time getTimestamp();
  void setTimestamp(org.ros.message.Time value);
  double getValue();
  void setValue(double value);
  boolean getIsInputOnly();
  void setIsInputOnly(boolean value);
}
