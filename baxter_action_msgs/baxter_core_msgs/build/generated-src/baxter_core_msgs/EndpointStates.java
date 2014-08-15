package baxter_core_msgs;

public interface EndpointStates extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "baxter_core_msgs/EndpointStates";
  static final java.lang.String _DEFINITION = "string[] names\nEndpointState[] states\n\n";
  java.util.List<java.lang.String> getNames();
  void setNames(java.util.List<java.lang.String> value);
  java.util.List<baxter_core_msgs.EndpointState> getStates();
  void setStates(java.util.List<baxter_core_msgs.EndpointState> value);
}
