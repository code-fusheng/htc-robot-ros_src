/*
 * @Author: LitoNeo 
 * @Date: 2019-08-01 15:59:04 
 * @Last Modified by: LitoNeo
 * @Last Modified time: 2019-08-01 16:02:27
 */
#include <condition_variable>
#include <queue>
#include <thread>
#include <tf/transform_listener.h>
#include <automotive_msgs/ConfigMapManager.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <pcl_conversions/pcl_conversions.h>
#include <std_msgs/Bool.h>
#include <tf/transform_listener.h>
#include <automotive_msgs/FunctionSwitch.h>
#include <ros/service.h>
#include <automotive_msgs/SmartcarSolution.h>
#include <automotive_msgs/SmartcarSolutionRequest.h>
#include <automotive_msgs/SmartcarRunTypeResponse.h>

#include <htcbot_msgs/MapPathConf.h>


namespace pcd_map_manager
{
struct Area
{
	std::string filename;
	double x_min;
	double y_min;
	double z_min;
	double x_max;
	double y_max;
	double z_max;
	sensor_msgs::PointCloud2 points;
};
typedef std::vector<Area> AreaList;

double UPDATE_INTERVAL = 1000; // ms
double MARGIN = 100;		   // meter
// std::string MAP_DIR;
const std::string AREALIST_FILENAME = "pcd_info.csv";
// std::string data_base;
std::string _dir_static_map, _dir_dynamic_map;

AreaList default_AreaList, cached_AreaList;

uint8_t _FUNCTION_STATUS = automotive_msgs::FunctionSwitchResponse::FUNCTION_ENABLE;

AreaList read_arealist(const std::string &path)
{
	std::ifstream ifs(path.c_str());
	std::string line;
	AreaList ret;
	while (std::getline(ifs, line))
	{
		std::istringstream iss(line);
		std::string col;
		std::vector<std::string> cols;
		while (std::getline(iss, col, ','))
			cols.push_back(col);
		Area tmp;
		tmp.filename = cols[0];
		tmp.x_min = std::stod(cols[1]);
		tmp.y_min = std::stod(cols[2]);
		tmp.z_min = std::stod(cols[3]);
		tmp.x_max = std::stod(cols[4]);
		tmp.y_max = std::stod(cols[5]);
		tmp.z_max = std::stod(cols[6]);
		ret.push_back(tmp);
	}
	return ret;
}

bool is_in_area(double x, double y, const Area &area, double m)
{
	return ((area.x_min - m) <= x && x <= (area.x_max + m) && (area.y_min - m) <= y && y <= (area.y_max + m));
}

AreaList create_wantedAreaList(const geometry_msgs::Point &p)
{
	AreaList ret;
	for (const Area &area : default_AreaList)
	{
		if (is_in_area(p.x, p.y, area, MARGIN))
		{
			Area in_area(area);
			ret.push_back(in_area);
		}
	}
	return ret;
}

int is_in_cachedAreaList(Area wanted_area)
{
	int length = cached_AreaList.size();
	for (int i = 0; i < length; i++)
	{
		if (wanted_area.filename == cached_AreaList[i].filename)
		{
			return i;
		}
	}
	return -1;
}


class pcd_dynamicmap_manager {
public:
	pcd_dynamicmap_manager():pnh_("~"){

	};

	~pcd_dynamicmap_manager(){};

	void run() {
		pnh_.param<double>("margin", MARGIN, 100.0);
		pnh_.param<double>("update_interval", UPDATE_INTERVAL, 1000.0); // ms

		// default_AreaList = read_arealist(MAP_DIR + AREALIST_FILENAME);
		pcd_pub = nh_.advertise<sensor_msgs::PointCloud2>("/points_map", 1, true);
		pcd_pub_sparse = nh_.advertise<sensor_msgs::PointCloud2>("static_map", 1, true);
		// publish_sparse_map();

		current_sub = nh_.subscribe("/current_pose", 10, &pcd_dynamicmap_manager::publish_current_pcd, this);
		initial_sub = nh_.subscribe("/initialpose", 1, &pcd_dynamicmap_manager::publish_dragged_pcd, this);
		config_sub = nh_.subscribe("/set_pcdmap_manager", 1, &pcd_dynamicmap_manager::cb_config, this);

		map_path_conf_sub = nh_.subscribe("/htcbot/map_path_conf", 1, &pcd_dynamicmap_manager::handle_map_path_conf, this);

		ros::ServiceServer function_service = nh_.advertiseService("/function_switch/pcd_map_manager", &pcd_dynamicmap_manager::on_callback_function_switch, this);	

		ros::spin();
	}

private:
	ros::NodeHandle nh_, pnh_;
	ros::Time current_time;

	uint8_t auto_pilot_status = automotive_msgs::SmartcarSolution::Response::SMARTCAR_LIDAR;
	ros::Publisher pcd_pub, pcd_pub_sparse;
	ros::Subscriber current_sub, initial_sub, config_sub;
	ros::Subscriber map_path_conf_sub;

	bool on_callback_function_switch(automotive_msgs::FunctionSwitch::Request &req, automotive_msgs::FunctionSwitch::Response &res) {
		if (_FUNCTION_STATUS == req.switch_to) {
			res.current_function_status = _FUNCTION_STATUS;
			return true;
		}

		switch (req.switch_to)
		{
		case automotive_msgs::FunctionSwitch::Response::FUNCTION_ENABLE:
			_FUNCTION_STATUS = automotive_msgs::FunctionSwitch::Response::FUNCTION_ENABLE;
			current_sub = nh_.subscribe("/current_pose", 10, &pcd_dynamicmap_manager::publish_current_pcd, this);
			initial_sub = nh_.subscribe("/initialpose", 1, &pcd_dynamicmap_manager::publish_dragged_pcd, this);
			config_sub = nh_.subscribe("/set_pcdmap_manager", 1, &pcd_dynamicmap_manager::cb_config, this);
			ROS_INFO("[pcd map manager] Function ON");
			break;

		case automotive_msgs::FunctionSwitch::Response::FUNCTION_DISABLE:
			_FUNCTION_STATUS = automotive_msgs::FunctionSwitch::Response::FUNCTION_DISABLE;
			current_sub.shutdown();
			initial_sub.shutdown();
			config_sub.shutdown();
			ROS_INFO("[pcd map manager] Function OFF");
			break;

		default:
			break;
		}
		res.current_function_status = _FUNCTION_STATUS;

		return true;
	}

	sensor_msgs::PointCloud2 create_pcd(const geometry_msgs::Point &p) // 载入p±MARGIN所在区域的点云
	{
		AreaList wanted_AreaList = create_wantedAreaList(p);
		AreaList tmp_AreaList;

		bool cache_update = false;
		for (Area &area : wanted_AreaList)
		{
			int index = is_in_cachedAreaList(area);
			if (index == -1)
			{ // io
				Area in_area(area);
				pcl::io::loadPCDFile(_dir_dynamic_map + "/" + area.filename, in_area.points);
				tmp_AreaList.push_back(in_area);
				cache_update = true;
			}
			else
			{
				Area in_area(cached_AreaList[index]);
				tmp_AreaList.push_back(in_area);
			}
		}
		if (cache_update)
		{
			cached_AreaList.clear();
			cached_AreaList = tmp_AreaList;
		}
		else
		{
			sensor_msgs::PointCloud2 msg_pcd;
			msg_pcd.width = 0;
			return msg_pcd;
		}

		sensor_msgs::PointCloud2 msg_pcd;
		for (const Area &area : cached_AreaList)
		{
			if (msg_pcd.width == 0)
				msg_pcd = area.points;
			else
			{
				msg_pcd.width += area.points.width;
				msg_pcd.row_step += area.points.row_step;
				msg_pcd.data.insert(msg_pcd.data.end(), area.points.data.begin(), area.points.data.end());
			}
		}
		ROS_INFO("[PCD Map Manager] Created pcd map size = %d", msg_pcd.width);
		return msg_pcd;
	}

	void publish_pcd(sensor_msgs::PointCloud2 pcd, const int *errp = NULL)
	{
		if (pcd.width != 0)
		{
			pcd.header.frame_id = "map";
			pcd_pub.publish(pcd);
			ROS_INFO("[pcd_map_manager] Publish map with &d points", pcd.width);
			return;
		}

		ROS_INFO("[pcd_map_manager] No points to publish");
	}

	void publish_current_pcd(const geometry_msgs::PoseStamped &msg)
	{
		if (_dir_dynamic_map == "") return;
		static ros::Time last_update_time = ros::Time::now();
		if ((msg.header.stamp - last_update_time).toSec() * 1000 < UPDATE_INTERVAL)
			return;
		publish_pcd(create_pcd(msg.pose.position));
		last_update_time = msg.header.stamp;
	}

	void publish_dragged_pcd(const geometry_msgs::PoseWithCovarianceStamped &msg)
	{
		if (_dir_dynamic_map == "") {
			ROS_WARN("[PCD Map Manager] Received /initial_pose, but dynamic map dir is empty.");
			return;
		}
		geometry_msgs::PoseStamped cur_pose;
		cur_pose.header = msg.header;
		cur_pose.pose = msg.pose.pose;
		publish_pcd(create_pcd(msg.pose.pose.position));
	}

	bool publish_sparse_map() {
		ros::Duration(3.0).sleep();
		sensor_msgs::PointCloud2::Ptr msg_globalmap(new sensor_msgs::PointCloud2);
		int res = pcl::io::loadPCDFile(_dir_static_map+"/static.pcd", *msg_globalmap);
		if (res != 0)
		{
			ROS_WARN("[pcd map manager] Cann't load map");
			return false;
		}
		msg_globalmap->header.frame_id = "map";
		pcd_pub_sparse.publish(*msg_globalmap);
		return true;
	}

	void cb_config(const automotive_msgs::ConfigMapManagerConstPtr &msg) {
		ROS_INFO("[PCD Map Manager] Received config setting");
		_dir_static_map = msg->path_static_map;
		_dir_dynamic_map = msg->path_dynamic_map;
		default_AreaList = read_arealist(_dir_dynamic_map + "/" + AREALIST_FILENAME);
		publish_sparse_map();
	}

	void handle_map_path_conf(const htcbot_msgs::MapPathConfConstPtr &input) {
		_dir_static_map = input->map_static_path;
		_dir_dynamic_map = input->map_dynamic_path;
		default_AreaList = read_arealist(_dir_dynamic_map + "/" + AREALIST_FILENAME);
		publish_sparse_map();
	}

};  // end pcd_dynamicmap_manager

}  // end namespace pcd_map_manager


int main(int argc, char **argv)
{
	ros::init(argc, argv, "points_map_loader");

	pcd_map_manager::pcd_dynamicmap_manager app;
	app.run();

	return 0;
}
