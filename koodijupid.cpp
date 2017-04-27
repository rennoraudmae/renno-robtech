      if(ev->code != BTN_LEFT)
	ROS_WARN("Unexpected key event; ev->code = 0x%04x\n", ev->code);
      else
      {
	// reads EV_KEY value, converts it to bool
	pressed_ = (bool)ev->value;
	
	//// Composing a ros_message
	ros_message.integral = integral_;
	ros_message.is_pressed = pressed_;
	
	//// Publish ros_message
	ros_publisher.publish( ros_message );
	////printf("Button was %s\n", ev->value? "pressed":"released");
      }



            if(ev->code != BTN_LEFT)
      	ROS_WARN("PUSH THE BUTTON");
      else
      {
      	pressed_ = (bool)ev->value;
      	//compose ROS message
      	ros_message.is_pressed = pressed_;
      	ros_publisher.publish( ros_message );
      }

	//signed char dir = (signed char)ev->value;
	
	// Sums consecutive dir values to find integral
	//integral_ += (long long)dir;
	
	// Composing a ros_message
	//ros_message.EV_REL= ev->value;
	////ros_message.REL_Y = REL_Y;
	//ros_message.is_pressed = pressed_;

	//// Publish ros_message
	
	//ros_publisher.publish( ros_message );
	//printf("Button was rotated %d units; Shift from start is now %d units\n", (int)ev->value, total_shift);
   //   }



      # TRUE while moving on X axis, FALSE otherwise

      # Dirction values can be -1 for counter-clockwise, 0 for no, and 1 clockwise rotation
int8 direction

# Sum of direction values since the launch of the node
int64 integral