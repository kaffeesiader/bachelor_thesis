void controller_manager_thread() {

	ros::Rate r(100);

	ros::Time lastTimeStamp = ros::Time::now();
	ros::Time hwTime;

	// create controller manager instance
	ControllerManager cm(hw_.get());

	ROS_INFO("Entering controller manager callback loop...");

	while (ros::ok()) {
		ros::Time currentTimeStamp = ros::Time::now();
		ros::Duration duration = currentTimeStamp - lastTimeStamp;

		hwTime += duration;
		// update the controllers based on time since last iteration
		cm.update(currentTimeStamp, duration);
		// force hw to publish the commanded values
		hw_->publish();

		lastTimeStamp = currentTimeStamp;
		r.sleep();
	}

	ROS_INFO("Controller manager callback loop shut down");
}