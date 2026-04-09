#include "Planning.hpp"

PlanningNode::PlanningNode():rclcpp::Node("planning_node") {

    // Client for map
    map_client_ = this->create_client<nav_msgs::srv::GetMap>("/map_server/map");

    // Service for path
    plan_service_ = this->create_service<nav_msgs::srv::GetPlan>("/plan_path",
        std::bind(&PlanningNode::planPath, this, std::placeholders::_1, std::placeholders::_2)
    );

    // Publisher for path
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/planned_path", 10);

    RCLCPP_INFO(get_logger(), "Planning node started.");

    // Connect to map server
    while (!map_client_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_INFO(get_logger(), "Waiting for map server...");
    }

    // Request map
    auto request = std::make_shared<nav_msgs::srv::GetMap::Request>();
    auto future = map_client_->async_send_request(request,
        std::bind(&PlanningNode::mapCallback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(get_logger(), "Trying to fetch map...");
}

void PlanningNode::mapCallback(rclcpp::Client<nav_msgs::srv::GetMap>::SharedFuture future) {
    auto response = future.get();

    if (response) {
        map_ = response->map;
        RCLCPP_INFO(get_logger(), "Map received: %d x %d",
                    map_.info.width, map_.info.height);

        dilateMap();
    } else {
        RCLCPP_ERROR(get_logger(), "Failed to receive map.");
    }
}

void PlanningNode::planPath(
    const std::shared_ptr<nav_msgs::srv::GetPlan::Request> request,
    std::shared_ptr<nav_msgs::srv::GetPlan::Response> response) {

    aStar(request->start, request->goal);
    smoothPath();

    response->plan = path_;
    path_pub_->publish(path_);
    RCLCPP_INFO(get_logger(), "Prijata sprava, odosielam...");
}

void PlanningNode::dilateMap() {
    nav_msgs::msg::OccupancyGrid dilatedMap = map_;

    int radius = 10;

    for (int y = 0; y < (int)map_.info.height; y++) {
        for (int x = 0; x < (int)map_.info.width; x++) {

            int idx = y * map_.info.width + x;

            if (map_.data[idx] > 50) { // obstacle
                for (int dy = -radius; dy <= radius; dy++) {
                    for (int dx = -radius; dx <= radius; dx++) {

                        int nx = x + dx;
                        int ny = y + dy;

                        if (nx >= 0 && ny >= 0 && nx < (int)map_.info.width && ny < (int)map_.info.height) {

                            int nidx = ny * map_.info.width + nx;
                            dilatedMap.data[nidx] = 100;
                        }
                    }
                }
            }
        }
    }

    map_ = dilatedMap;
}

void PlanningNode::aStar(const geometry_msgs::msg::PoseStamped &start,
                         const geometry_msgs::msg::PoseStamped &goal) {

    path_.poses.clear();
    path_.header.frame_id = "map";
    path_.header.stamp = now();

    int width = map_.info.width;

    auto toMap = [&](double x, double y) {
        int mx = (x - map_.info.origin.position.x) / map_.info.resolution;
        int my = (y - map_.info.origin.position.y) / map_.info.resolution;
        return std::make_pair(mx, my);
    };

    auto [sx, sy] = toMap(start.pose.position.x, start.pose.position.y);
    auto [gx, gy] = toMap(goal.pose.position.x, goal.pose.position.y);
    
    if (map_.data[sy * width + sx] > 50 ||
    map_.data[gy * width + gx] > 50) {
    RCLCPP_ERROR(get_logger(), "Start or goal in obstacle!");
    return;
}

    std::vector<std::shared_ptr<Cell>> openList;
    std::vector<bool> closedList(map_.info.height * map_.info.width, false);

    auto startCell = std::make_shared<Cell>(sx, sy);
    startCell->g = 0;
    startCell->h = hypot(sx - gx, sy - gy);
    startCell->f = startCell->g + startCell->h;
    openList.push_back(startCell);

    while (!openList.empty()) {

        // find best (min f)
        auto current = *std::min_element(openList.begin(), openList.end(),
            [](auto a, auto b) { return a->f < b->f; });

        int cx = current->x;
        int cy = current->y;

        if (cx == gx && cy == gy) {
            // reconstruct path
            while (current != nullptr) {
                geometry_msgs::msg::PoseStamped pose;
                
                pose.header.frame_id = "map";
		pose.header.stamp = now();
		pose.pose.orientation.w = 1.0;

                pose.pose.position.x = cx * map_.info.resolution + map_.info.origin.position.x;
                pose.pose.position.y = cy * map_.info.resolution + map_.info.origin.position.y;

                path_.poses.push_back(pose);

                current = current->parent;
                if (current) {
                    cx = current->x;
                    cy = current->y;
                }
            }

            std::reverse(path_.poses.begin(), path_.poses.end());
            return;
        }

        openList.erase(std::remove(openList.begin(), openList.end(), current), openList.end());
        closedList[cy * width + cx] = true;

        // neighbors (8-dir)
        std::vector<std::pair<int,int>> neighbors = {
	    {1,0},{-1,0},{0,1},{0,-1},
	    {1,1},{1,-1},{-1,1},{-1,-1}
	};

        for (auto [dx, dy] : neighbors) {
            int nx = cx + dx;
            int ny = cy + dy;

            if (nx < 0 || ny < 0 || nx >= (int)map_.info.width || ny >= (int)map_.info.height)
                continue;

            int idx = ny * width + nx;

            if (closedList[idx] || map_.data[idx] > 50)
                continue;

            float cost = (dx != 0 && dy != 0) ? 1.41f : 1.0;
	    float new_g = current->g + cost;
            float new_h = hypot(nx - gx, ny - gy);
            float new_f = new_g + new_h;

	    bool inOpen = false;

	    for (auto &c : openList) {
	        if (c->x == nx && c->y == ny) {
	            inOpen = true;

	        if (new_g < c->g) {
        	    c->g = new_g;
        	    c->h = new_h;
        	    c->f = new_f;
        	    c->parent = current;
        	}
        	break;
    		}
	}

	if (!inOpen) {
    	    auto neighbor = std::make_shared<Cell>(nx, ny);
    	    neighbor->g = new_g;
	    neighbor->h = new_h;
	    neighbor->f = new_f;
	    neighbor->parent = current;

	    openList.push_back(neighbor);
	}
        }
    }

    RCLCPP_ERROR(get_logger(), "Unable to plan path.");
}

void PlanningNode::smoothPath() {
    if (path_.poses.size() < 3) return;

    std::vector<geometry_msgs::msg::PoseStamped> newPath = path_.poses;

    double alpha = 0.1;
    double beta = 0.3;

    for (int iter = 0; iter < 100; iter++) {
        for (size_t i = 1; i < newPath.size() - 1; i++) {

            newPath[i].pose.position.x +=
                alpha * (path_.poses[i].pose.position.x - newPath[i].pose.position.x) +
                beta * (newPath[i-1].pose.position.x + newPath[i+1].pose.position.x - 2 * newPath[i].pose.position.x);

            newPath[i].pose.position.y +=
                alpha * (path_.poses[i].pose.position.y - newPath[i].pose.position.y) +
                beta * (newPath[i-1].pose.position.y + newPath[i+1].pose.position.y - 2 * newPath[i].pose.position.y);
        }
    }

    path_.poses = newPath;
}

Cell::Cell(int c, int r) {
    x = c;
    y = r;
    g = h = f = 0.0;
    parent = nullptr;
}
