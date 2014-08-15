package baxter_core_msgs;

public interface AnalogIOStates extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "baxter_core_msgs/AnalogIOStates";
  static final java.lang.String _DEFINITION = "string[]         names\nAnalogIOState[] states";
  java.util.List<java.lang.String> getNames();
  void setNames(java.util.List<java.lang.String> value);
  java.util.List<baxter_core_msgs.AnalogIOState> getStates();
  void setStates(java.util.List<baxter_core_msgs.AnalogIOState> value);
}
