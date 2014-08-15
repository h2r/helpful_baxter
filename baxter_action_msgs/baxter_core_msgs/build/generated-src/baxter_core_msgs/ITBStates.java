package baxter_core_msgs;

public interface ITBStates extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "baxter_core_msgs/ITBStates";
  static final java.lang.String _DEFINITION = "# used when publishing multiple itbs\nstring[]         names\nITBState[] states";
  java.util.List<java.lang.String> getNames();
  void setNames(java.util.List<java.lang.String> value);
  java.util.List<baxter_core_msgs.ITBState> getStates();
  void setStates(java.util.List<baxter_core_msgs.ITBState> value);
}
