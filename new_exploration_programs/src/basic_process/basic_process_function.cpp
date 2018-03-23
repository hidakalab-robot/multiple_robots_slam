#include <new_exploration_programs/basic_process.h>

void BasicProcess::odom_marking(float x, float y)
{
	geometry_msgs::Point marking_point;
	marking_point.x = x;
	marking_point.y = y;
	marking_point.z = which_exp;
	which_pub.publish(marking_point);
}

void BasicProcess::bumper_callback(const kobuki_msgs::BumperEvent::ConstPtr& bumper_msg)
{
	if(bumper_msg -> state == 1){
		bumper_hit = true;
		which_bumper = bumper_msg -> bumper;
		std::cout << "bumper_hit" << std::endl;
	}
	else{
		bumper_hit = false;
		std::cout << "no_bumper_hit" << std::endl;
	}
}

void BasicProcess::display_goal_angle(float x, float y)
{
	uint32_t arrow = visualization_msgs::Marker::ARROW;
	visualization_msgs::Marker marker1;
	marker1.header.frame_id = "map";
	marker1.header.stamp = ros::Time::now();
	marker1.ns = "basic_shapes";
	marker1.id = 0;
	marker1.type = arrow;
	marker1.action = visualization_msgs::Marker::ADD;
	geometry_msgs::Point pgo0;
	geometry_msgs::Point pgo1;

	//float xo,yo;
	//double zo;

	//odom_get(&xo, &yo, &zo);
	odom_queue.callOne(ros::WallDuration(1));

	pgo0.x = odom_x;
	pgo0.y = odom_y;
	pgo0.z = 0.0;

	pgo1.x = x;
	pgo1.y = y;
	pgo1.z = 0.0;

	marker1.points.push_back(pgo0);
	marker1.points.push_back(pgo1);

	marker1.scale.x = 0.1;
	marker1.scale.y = 0.5;
	marker1.scale.z = 0.0;

	marker1.color.r = 0.0f;
	marker1.color.g = 1.0f;
	marker1.color.b = 0.0f;
	marker1.color.a = 1.0;

	marker1.lifetime = ros::Duration(0);

	marker_pub.publish(marker1);
}


void BasicProcess::display_gravity(float x, float y)
{
	uint32_t line = visualization_msgs::Marker::LINE_STRIP;
	visualization_msgs::Marker marker2;
	marker2.header.frame_id = "map";
	marker2.header.stamp = ros::Time::now();
	marker2.ns = "basic_shapes";
	marker2.id = 1;
	marker2.type = line;
	marker2.action = visualization_msgs::Marker::ADD;
	marker2.pose.orientation.w = 1.0;
	geometry_msgs::Point pgr0;
	geometry_msgs::Point pgr1;

//	float xo,yo;
//	double zo;

	//odom_get(&xo, &yo, &zo);
	odom_queue.callOne(ros::WallDuration(1));

	pgr0.x = odom_x;
	pgr0.y = odom_y;
	pgr0.z = 0.0;

	pgr1.x = x;
	pgr1.y = y;
	pgr1.z = 0.0;

	marker2.points.push_back(pgr0);
	marker2.points.push_back(pgr1);

	marker2.scale.x = 0.1;

	marker2.color.r = 1.0f;
	marker2.color.g = 0.0f;
	marker2.color.b = 0.0f;
	marker2.color.a = 1.0;

	marker2.lifetime = ros::Duration(0);

	marker_pub.publish(marker2);
}

void BasicProcess::vel_curve_VFH(float theta,float t,float v)////frontierでもodomlogを取るようにする
{
	float theta_rho;

	pre_theta = theta;

	theta_rho = 2*theta;
	omega = theta_rho/t;

	vel.linear.x = v;
	vel.angular.z = omega;

	vel_pub.publish(vel);
	std::cout << "障害物を回避しながら移動中♪" << std::endl;

	//float xo,yo;
	//double zo;

	//odom_get(&xo, &yo, &zo);
	odom_queue.callOne(ros::WallDuration(1));

	odom_log_x.push_back(odom_x);
	odom_log_y.push_back(odom_y);

	odom_marking(odom_x,odom_y);
}

void BasicProcess::approx(std::vector<float> &scan)
{
	float depth,depth1,depth2;
	depth=0;
	depth1=0;
	depth2=0;

	for(int j=0,count=0;j<scan.size()-1;j++){
		depth=scan[j];
		//|val|nan|のとき
		//ROS_INFO("(%d,%d):",i,j);

		if(!isnan(depth) && isnan(scan[j+1])){
			depth1=depth;
			count++;
			//ROS_INFO("!nan&nan(%d,%d)",j,j+1);
		}

		if(isnan(depth)){
	//|nan|nan|の区間
			if(isnan(scan[j+1])){
				count++;
				//ROS_INFO("nan&nan(%d,%d)",j,j+1);
			}
	//|nan|val|のとき
			else{
				//ROS_INFO("nan&!nan(%d,%d)",j,j+1);
				depth2=scan[j+1];
	//左端がnanのとき
				if(isnan(depth1)){
					for(int k=0;k<count+1;k++)
						scan[j-k]=0.01;//depth2;
				}
				else{
					for(int k=0;k<count;k++)
						scan[j-k]=depth2-(depth2-depth1)/(count+1)*(k+1);
				}
				//ROS_INFO("nan|val|:nancount=%d",count);
				count=0;
			}
		}
	//右端がnanのとき
		if(j==(scan.size()-1)-1 &&isnan(scan[j+1])){
			for(int k=0;k<count;k++)
				scan[j+1-k]=0.01;//depth1;
			//ROS_INFO("val|nan|nan|:nancount=%d",count);
			count=0;
		}
	}
	if(isnan(scan[0])){
		scan[0] = scan[1] - (scan[2] - scan[1]);
		if(scan[0] < 0){
			scan[0] = 0;
		}
	}
}

//void BasicProcess::loop_callback(const geometry_msgs::Point::ConstPtr& loop_msg)//loop-closureを数える
void BasicProcess::loop_callback(const std_msgs::Int8::ConstPtr& loop_msg)
{
	//loop_count = loop_msg -> x;
	loop_count = loop_msg -> data;
}

void BasicProcess::get_loop(int *loop)
{
	loop_queue.callOne(ros::WallDuration(1));
	*loop = loop_count;
}


void BasicProcess::odom_callback(const geometry_msgs::Point::ConstPtr& odom_msg)
{
	odom_x = odom_msg -> x;
	odom_y = odom_msg -> y;
	yaw = odom_msg -> z;
}


void BasicProcess::get_odom(float *x, float *y, double *z)
{
	odom_queue.callOne(ros::WallDuration(1));
	*x = odom_x;
	*y = odom_y;
	*z = yaw;
}

void BasicProcess::map_callback(const nav_msgs::OccupancyGrid::ConstPtr& map_msg)
{
	map_info = map_msg->info;
	map_data = map_msg->data;
}

void BasicProcess::get_map(nav_msgs::MapMetaData *info, std::vector<int8_t> *data)
{
	map_queue.callOne(ros::WallDuration(1));
	*info = map_info;
	*data = map_data;
}

void BasicProcess::scan_callback(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
	ranges = scan_msg->ranges;
	angle_min = scan_msg->angle_min;
	angle_max = scan_msg->angle_max;
	angle_increment = scan_msg->angle_increment;
}

void BasicProcess::get_scan(std::vector<float> *r, float *min, float *max, float *inc)
{
	scan_queue.callOne(ros::WallDuration(1));
	*r = ranges;
	*min = angle_min;
	*max = angle_max;
	*inc = angle_increment;
}

void BasicProcess::get_bumper(bool *hit, int *which)
{
	bumper_queue.callOne(ros::WallDuration(1));
	*hit = bumper_hit;
	*which = which_bumper;
}

void BasicProcess::get_odomlog(std::vector<float> *logx, std::vector<float> *logy)
{
	*logx = odom_log_x;
	*logy = odom_log_y;
}

void BasicProcess::get_vel(float *x, float *z)
{
	*x = vel.linear.x;
	*z = vel.angular.z;
}

void BasicProcess::set_vel(float x, float z)
{
	vel.linear.x = x;
	vel.angular.z = z;
}

void BasicProcess::pub_vel(void)
{
	vel_pub.publish(vel);
}

void BasicProcess::one_rotation(void)
{
	float pre_yaw = 0;
	int rotate_count = 0;

	vel.linear.x = 0;
	vel.angular.z = 0.3;

	while(rotate_count < 2){
		vel_pub.publish(vel);
		odom_queue.callOne(ros::WallDuration(1));
		if(yaw*pre_yaw < 0){
			rotate_count++;
		}
		pre_yaw = yaw;
	}
}

void BasicProcess::pub_led1(void)
{
	led1_pub.publish(led1);
}

void BasicProcess::pub_led2(void)
{
	led2_pub.publish(led2);
}

void BasicProcess::set_led1(int val)
{
	led1.value = val;
}

void BasicProcess::get_led1(int *val)
{
	*val = led1.value;
}

void BasicProcess::set_led2(int val)
{
	led2.value = val;
}

void BasicProcess::get_led2(int *val)
{
	*val = led2.value;
}

void BasicProcess::set_whichexp(float w)
{
	which_exp = w;
}

void BasicProcess::get_rotatevel(float *r)
{
	*r = rotate_vel;
}

void BasicProcess::get_scanthreshold(float *s)
{
	*s = scan_threshold;
}

void BasicProcess::get_pretheta(float *p)
{
	*p = pre_theta;
}

void BasicProcess::get_omega(float *o)
{
	*o = omega;
}
