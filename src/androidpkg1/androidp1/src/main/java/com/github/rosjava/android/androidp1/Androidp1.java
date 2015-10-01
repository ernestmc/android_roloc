/*
 * Copyright 2015 Ekumen
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package com.github.rosjava.android.androidp1;


import org.ros.android.RosActivity;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeListener;
import org.ros.node.NodeMain;
import org.ros.node.NodeMainExecutor;
import org.ros.node.parameter.ParameterTree;
import org.ros.RosCore;

import org.ros.android.android_sensors_driver.ImuPublisher;

import android.content.Context;
import android.hardware.SensorManager;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

import org.ros.rosjava_tutorial_native_node.LocalizationNativeNode;
import java.net.URI;
import java.util.Arrays;


public class Androidp1 extends RosActivity
{
    private RosCore myRoscore;
    private Log log = LogFactory.getLog(Androidp1.class);
    private NodeMainExecutor nodeMainExecutor = null;
    private URI masterUri;
    private String hostName;
    private LocalizationNativeNode localizationNativeNode;
    private ImuPublisher imuNode;
    final static String appName = "RoLoc_test1";
    
    public Androidp1()
    {
        super(appName, appName);
        
        //log.info("Creating a ROS master...");
        
        // create the ROS master listening at the specified port
        //myRoscore = RosCore.newPublic(11311);
        /*myRoscore = RosCore.newPrivate();
        
        myRoscore.start();
        try {
            myRoscore.awaitStart();
        } catch (Exception e) {
            throw new RuntimeException(e);
        }*/
    }
    
    @Override
    protected void init(NodeMainExecutor nodeMainExecutor)
    {
        log.info("Androidp1 init");
        
        // Store a reference to the NodeMainExecutor and unblock any processes that were waiting
        // for this to start ROS Nodes
        this.nodeMainExecutor = nodeMainExecutor;
        masterUri = getMasterUri();
        //masterUri = myRoscore.getUri();
        hostName = getRosHostname();

        log.info(masterUri);

        // Load ROS parameters in the parameter server
        //NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic(hostName);
        //nodeConfiguration.setMasterUri(masterUri);

        /*parameterLoaderNode = new ParameterLoaderNode(getResources().openRawResource(R.raw.ros_parameters), this.getExternalCacheDir(), true);
        nodeConfiguration.setNodeName("parameter_loader"); //FIXME: There is a naming collision with ParameterLoaderNode.java
        nodeMainExecutor.execute(parameterLoaderNode, nodeConfiguration);*/
        
        startParameterLoader();
        startImu();
        startLocalization();        
    }
    
    public void onStart(final ConnectedNode connectedNode)
    {
        // Load start-up parameters
        /*ParameterTree parameterTree = connectedNode.getParameterTree();
        
        configRobotLocalization(parameterTree);*/
        
    }
    
    // Create a native robot_localization node
    private void startLocalization()
    {
        log.info("Starting localization...");
        
        NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic(hostName);
        
        nodeConfiguration.setMasterUri(masterUri);
        nodeConfiguration.setNodeName("RolocNode");
        
        localizationNativeNode = new LocalizationNativeNode();
        
        nodeMainExecutor.execute(localizationNativeNode, nodeConfiguration);
    }
    
    // Create IMU publisher node
    private void startImu()
    {
        log.info("Starting imu...");
        
        NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic(hostName);
        SensorManager mSensorManager = (SensorManager)this.getSystemService(SENSOR_SERVICE);

        nodeConfiguration.setMasterUri(masterUri);
        nodeConfiguration.setNodeName("ImuNode");
        
        imuNode = new ImuPublisher(mSensorManager);
        
        log.info("About to execute ImuNode...");
        nodeMainExecutor.execute(imuNode, nodeConfiguration);
    }
    
    // Load the parameter loader node. It dies as soon as it loads the parameters.
    private void startParameterLoader()
    {
        NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic(hostName);

        nodeConfiguration.setMasterUri(masterUri);
        nodeConfiguration.setNodeName("parameter_loader_node");
        
        ParameterLoaderNode paramNode = new ParameterLoaderNode();
        
        nodeMainExecutor.execute(paramNode, nodeConfiguration);
    }
    
    /*private void configRobotLocalization(ParameterTree paramTree)
    {
        boolean imuConfig[] = {false, false, false,
                         true, true, true,
                         false, false, false,
                         false, false, false,
                         true, true, true};
        
        paramTree.set("two_d_mode", true);
        paramTree.set("base_link_frame", "imu");
        
        paramTree.set("imu0", "/android/imu");        
        paramTree.set("imu0_config", Arrays.asList(imuConfig));
    }*/
    
    /*public void stopLocalization()
    {
        this.nodeMainExecutor.shutdownNodeMain(localizationNativeNode);
    }*/
    

}
