/*
 *	occupancygrid_store.cpp
 */

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>

/*global variables*/
nav_msgs::OccupancyGrid grid_now;
nav_msgs::OccupancyGrid grid_store;
nav_msgs::Odometry odom;
ros::Time time_odom_now;
ros::Time time_odom_last;
bool first_callback_odom = true;
double theta = 0.0;
double delta_x = 0.0;
double delta_y = 0.0;

bool cell_is_inside(nav_msgs::OccupancyGrid grid, double x, double y)
{
	if(fabs(x)>grid.info.width*grid.info.resolution/2.0)	return false;
	if(fabs(y)>grid.info.height*grid.info.resolution/2.0)	return false;
	return true;
}

void index_to_point(nav_msgs::OccupancyGrid grid, int index, int& x, int& y)
{
	x = index%grid.info.width - grid.info.width/2.0;
	y = index/grid.info.width - grid.info.height/2.0;
	// std::cout << "index = " << index << std::endl;
	// std::cout << "x = " << x << std::endl;
	// std::cout << "y = " << y << std::endl;
}

int point_to_index(nav_msgs::OccupancyGrid grid, int x, int y)
{
	// std::cout << "- POINT TO INDEX -" << std::endl;
	int x_ = x + grid.info.width/2.0;
	int y_ = y + grid.info.height/2.0;
	return	y_*grid.info.width + x_;
}

void filter(nav_msgs::OccupancyGrid grid)
{
	const int range = 2;
	for(int i=0;i<grid.info.width*grid.info.height;i++){
		if(grid.data[i]==-1){
			// std::cout << "-----" << std::endl;
			int x, y;
			index_to_point(grid_store, i, x, y);
			int count_roadcell = 0;
			for(int j=-range;j<=range;j++){
				for(int k=-range;k<=range;k++){
					if(!cell_is_inside(grid_store, point_to_index(grid_store, x+j, y+k)))	break;
					if(grid.data[point_to_index(grid_store, x+j, x+k)]==0)	count_roadcell++;
				}
			}
			if(count_roadcell>(2*range+1)*(2*range+1)-1){
				// std::cout << "Grid is updated" << std::endl;
				grid.data[i] = 0;
			}
		}
	}
}

void index_to_meterpoint(nav_msgs::OccupancyGrid grid, int index, double& x, double& y)
{
	x = (index%grid.info.width - grid.info.width/2.0 + 0.5)*grid.info.resolution;
	y = (index/grid.info.width - grid.info.height/2.0 + 0.5)*grid.info.resolution;
}

int meterpoint_to_index(nav_msgs::OccupancyGrid grid, double x, double y)
{
	int x_ = x/grid.info.resolution + grid.info.width/2.0;
	int y_ = y/grid.info.resolution + grid.info.height/2.0;
	int index = y_*grid.info.width + x_;
	return index;
}

void move_cells(double dt)
{
	// std::cout << "MOVE CELLS" << std::endl;
	
	theta += odom.twist.twist.angular.z*dt;
	if(theta<-M_PI)	theta += 2*M_PI;
	if(theta>M_PI)	theta -= 2*M_PI;
	
	delta_x += odom.twist.twist.linear.x*dt * cos(theta);
	delta_y += odom.twist.twist.linear.x*dt * sin(theta);
	// std::cout << "theta = " << theta << std::endl;
	// std::cout << "delta_x = " << delta_x << std::endl;
	// std::cout << "delta_y = " << delta_y << std::endl;
	
	if(fabs(delta_x)>grid_store.info.resolution || fabs(delta_y)>grid_store.info.resolution){
		int delta_x_ = delta_x/grid_store.info.resolution;
		int delta_y_ = delta_y/grid_store.info.resolution;
		// std::cout << "delta_x_ = " << delta_x_ << std::endl;
		// std::cout << "delta_y_ = " << delta_y_ << std::endl;
		
		nav_msgs::OccupancyGrid tmp_grid = grid_store;
		for(size_t i=0;i<tmp_grid.data.size();i++)	tmp_grid.data[i] = -1;
		for(size_t i=0;i<grid_store.data.size();i++){
			double x, y;
			index_to_meterpoint(grid_store, i, x, y);
			// std::cout << i << " = " << meterpoint_to_index(grid_store, x, y) << std::endl;
			x -= delta_x_*grid_store.info.resolution;
			y -= delta_y_*grid_store.info.resolution;	
			// std::cout << "x = " << x << std::endl;
			// std::cout << "y = " << y << std::endl;
			int index_moved = meterpoint_to_index(grid_store, x, y);
			// std::cout << "index_moved = " << index_moved << std::endl;
			if(cell_is_inside(grid_store, x, y))	tmp_grid.data[index_moved] = grid_store.data[i];
		}
		delta_x -= delta_x_*grid_store.info.resolution;
		delta_y -= delta_y_*grid_store.info.resolution;

		grid_store = tmp_grid;
	}
}

void callback_odom(const nav_msgs::OdometryConstPtr& msg)
{
	// std::cout << "- CALLBACK ODOM -" << std::endl;
	odom = *msg;
	time_odom_now = ros::Time::now();
	double dt = (time_odom_now - time_odom_last).toSec();
	time_odom_last = time_odom_now;

	if(!first_callback_odom && !grid_store.data.empty())	move_cells(dt);

	first_callback_odom = false;
}

void grid_update(void)
{
	for(size_t i=0;i<grid_store.data.size();i++){
		if(grid_now.data[i]!=-1)	grid_store.data[i] = grid_now.data[i];
	}
}

void callback_grid(const nav_msgs::OccupancyGridConstPtr& msg)
{
	// std::cout << "- CALLBACK GRID -" << std::endl;
	
	grid_now = *msg;
	if(grid_store.data.empty())	grid_store = *msg;

	grid_update();

	/*for test*/
	// bool zero = true;
	// for(size_t i=0;i<grid_store.data.size();i++)	if(grid_store.data[i]!=-1)	zero = false;
	// if(zero){
	// 	std::cout << "reflesh" << std::endl;
	// 	grid_store = *msg;
	// }
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "occupancygrid_store");
	ros::NodeHandle nh;

	/*sub*/
	ros::Subscriber sub_grid = nh.subscribe("/occupancygrid/lidar", 1, callback_grid);
	ros::Subscriber sub_odom = nh.subscribe("/tinypower/odom", 1, callback_odom);
	
	/*pub*/
	ros::Publisher pub_grid = nh.advertise<nav_msgs::OccupancyGrid>("/occupancygrid/store",1);
	
	/*variables*/
	
	/*initialization*/

	/*loop*/
	ros::Rate loop_rate(40);
	while(ros::ok()){
		ros::spinOnce();
		
		if(!grid_store.data.empty())	pub_grid.publish(grid_store);
		
		loop_rate.sleep();
	}
}
