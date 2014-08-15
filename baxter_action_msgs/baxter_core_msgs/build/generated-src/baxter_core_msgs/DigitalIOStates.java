package baxter_core_msgs;

public interface DigitalIOStates extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "baxter_core_msgs/DigitalIOStates";
  static final java.lang.String _DEFINITION = "string[]         names\nDigitalIOState[] states";
  java.util.List<java.lang.String> getNames();
  void setNames(java.util.List<java.lang.String> value);
  java.util.List<baxter_core_msgs.DigitalIOState> getStates();
  void setStates(java.util.List<baxter_core_msgs.DigitalIOState> value);
}
