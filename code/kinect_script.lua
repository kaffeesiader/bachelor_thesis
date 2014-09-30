if (simGetScriptExecutionCount()==0) then
	depthCam=simGetObjectHandle('kinect_visionSensor')
	depthView=simFloatingViewAdd(0.9,0.9,0.2,0.2,0)
	simAdjustView(depthView,depthCam,64)

	-- publish depth image
	cmd=simros_strmcmd_get_depth_sensor_data
	topic='/simulation/kinect1/sensoring/depth_image'
	-- uncomment the following line to dissable the depth image publisher
	simExtROS_enablePublisher(topic,1,cmd,depthCam,-1,'')
	-- publish rgba image
	cmd=simros_strmcmd_get_vision_sensor_image
	topic='/simulation/kinect1/sensoring/rgb_image'
	-- uncomment the following line to dissable the rgb image publisher
	simExtROS_enablePublisher(topic,1,cmd,depthCam,-1,'')
end

simHandleChildScript(sim_handle_all_except_explicit)
