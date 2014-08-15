package baxter_core_msgs;

public interface NavigatorStates extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "baxter_core_msgs/NavigatorStates";
  static final java.lang.String _DEFINITION = "# used when publishing multiple navigators\nstring[]         names\nNavigatorState[] states\n";
  java.util.List<java.lang.String> getNames();
  void setNames(java.util.List<java.lang.String> value);
  java.util.List<baxter_core_msgs.NavigatorState> getStates();
  void setStates(java.util.List<baxter_core_msgs.NavigatorState> value);
}
