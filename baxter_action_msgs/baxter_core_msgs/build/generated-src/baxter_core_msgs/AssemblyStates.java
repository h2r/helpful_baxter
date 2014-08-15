package baxter_core_msgs;

public interface AssemblyStates extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "baxter_core_msgs/AssemblyStates";
  static final java.lang.String _DEFINITION = "string[] names\nAssemblyState[] states";
  java.util.List<java.lang.String> getNames();
  void setNames(java.util.List<java.lang.String> value);
  java.util.List<baxter_core_msgs.AssemblyState> getStates();
  void setStates(java.util.List<baxter_core_msgs.AssemblyState> value);
}
