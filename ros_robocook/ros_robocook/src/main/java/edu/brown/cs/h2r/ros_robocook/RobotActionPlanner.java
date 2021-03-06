/*
 * Copyright (C) 2014 kofarrell.
 * 
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 * 
 * http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

package edu.brown.cs.h2r.ros_robocook;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import move_msgs.BaxterAction;
import move_msgs.MoveAction;
import move_msgs.MoveObject;
import move_msgs.MoveRegion;
import object_recognition_msgs.RecognizedObject;
import object_recognition_msgs.RecognizedObjectArray;

import org.apache.commons.logging.Log;
import org.ros.message.MessageFactory;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

import burlap.behavior.singleagent.Policy;
import burlap.oomdp.core.Domain;
import burlap.oomdp.core.State;
import edu.brown.cs.h2r.baking.Experiments.BaxterKitchen;
import edu.brown.cs.h2r.baking.Recipes.Brownies;
import edu.brown.cs.h2r.baking.Recipes.Recipe;
import geometry_msgs.Point;
import geometry_msgs.Vector3;



/*
 * 1). Subscribe to recognized objects (done)
 * 2). Update state accordingly (Done)
 * 3). Plan action from new state (Update plan method)
 * 4). Publish action (Done)
 */


/**
 * A simple {@link Subscriber} {@link NodeMain}.
 */
public class RobotActionPlanner extends AbstractNodeMain {
  private boolean initialized;
  private State currentState;
  MessageFactory messageFactory;
  Publisher<BaxterAction> actionPublisher;
  List<String> previouslySeenObjects;
  private static BaxterKitchen kitchen = new BaxterKitchen();
  private Recipe recipe;
  private Domain domain;
   @Override
  public GraphName getDefaultNodeName() {
    return GraphName.of("rosjava/listener");
  }

  @Override
  public void onStart(ConnectedNode connectedNode) {
     final Log log = connectedNode.getLog();
     final BaxterKitchen kitchen = new BaxterKitchen();
     
     this.recipe = new Brownies();
     
     this.domain = RobotActionPlanner.kitchen.generateDomain(this.recipe);
     this.messageFactory = connectedNode.getTopicMessageFactory();
     //kitchen.setDebug(true);
     this.setInitializedFalse();
     this.actionPublisher = connectedNode.newPublisher("/baxter_action", BaxterAction._TYPE);
     Subscriber<RecognizedObjectArray> subscriber = connectedNode.newSubscriber("/ar_objects", RecognizedObjectArray._TYPE);
     this.previouslySeenObjects = new ArrayList<String>();
     this.currentState = kitchen.generateInitialState(kitchen.generateDomain(this.recipe), this.recipe);

    subscriber.addMessageListener(new MessageListener<RecognizedObjectArray>() {
      @Override
      public void onNewMessage(RecognizedObjectArray array) {
    	 List<String> currentObjects = new ArrayList<String>();
    	 
    	 State previousState = RobotActionPlanner.this.currentState.copy();
       for (final RecognizedObject obj : array.getObjects()) {
          final String id = obj.getType().getKey();
          final String name = id;
          double x = obj.getPose().getPose().getPose().getPosition().getX();
          double y = obj.getPose().getPose().getPose().getPosition().getY();
          double z = obj.getPose().getPose().getPose().getPosition().getZ();
          // add object
          System.out.println("Adding object " + name);
          RobotActionPlanner.this.currentState = kitchen.addObjectInRobotsSpace(RobotActionPlanner.this.domain, RobotActionPlanner.this.currentState, name);
          currentObjects.add(name);
         }
       
       RobotActionPlanner.this.previouslySeenObjects.removeAll(currentObjects);
       for (String name : previouslySeenObjects) {
    	   RobotActionPlanner.this.currentState = kitchen.disposeObject(RobotActionPlanner.this.currentState, name);
       }
       RobotActionPlanner.this.previouslySeenObjects.clear();
       RobotActionPlanner.this.previouslySeenObjects.addAll(currentObjects);

       //if (!previousState.equals(RobotActionPlanner.this.currentState)) {
	       String[] actionParams = null;
	       try {
	    	   actionParams = kitchen.getRobotAction(RobotActionPlanner.this.domain, RobotActionPlanner.this.currentState, RobotActionPlanner.this.recipe);
	       } catch (Exception e){
	    	   System.err.println(e.getMessage());
	    	   e.printStackTrace();
	    	   
	    	   System.out.println(RobotActionPlanner.this.currentState.toString());
	       }
	       
	       if (actionParams != null) {
	    	   System.out.println(Arrays.toString(actionParams));
	   		
	    	   BaxterAction actionMsg = RobotActionPlanner.this.getRobotActionMsg(actionParams);
	    	   RobotActionPlanner.this.actionPublisher.publish(actionMsg);
	       }
	       System.out.println("");
       //}
       //else {
    //	   System.out.println("State has not changed. No Actions suggested");
      // }
     }

    });
  }

  public BaxterAction getRobotActionMsg(String[] actionParams)
  {
    String action = actionParams[0];
    BaxterAction actionMsg = this.messageFactory.newFromType(BaxterAction._TYPE);
    if (action == "move" || action == "hand")
    {
      actionMsg.setType(BaxterAction.MOVE);
      MoveAction moveMsg = this.messageFactory.newFromType(MoveAction._TYPE);

      MoveRegion regionMsg = this.getMoveRegion(actionParams[3]);
      MoveObject objectMsg = this.messageFactory.newFromType(MoveObject._TYPE);
      objectMsg.setName(actionParams[2]);
      moveMsg.setObject(objectMsg);
      moveMsg.setRegion(regionMsg);
      actionMsg.setMoveAction(moveMsg);
    }

    return actionMsg;
  }

  public MoveRegion getMoveRegion(String regionName)
  {
    MoveRegion region = this.messageFactory.newFromType(MoveRegion._TYPE);
    region.setName(regionName);
    region.setShape(MoveRegion.SHAPE_CIRCLE);
    
    Point origin = this.messageFactory.newFromType(Point._TYPE);
    origin.setX(1.0);
    origin.setY(1.0);
    origin.setZ(0.0);
    region.setOrigin(origin);


    Vector3 scale = this.messageFactory.newFromType(Vector3._TYPE);
    scale.setX(0.333);
    scale.setY(0.333);
    scale.setZ(0.2);  
    region.setScale(scale);

    return region;
  }

  public boolean getInitialized() {
    return this.initialized;
  }

  public void setInitialized() {
    this.initialized = true;
  }

  public void setInitializedFalse() {
    this.initialized = false;
  }
}
