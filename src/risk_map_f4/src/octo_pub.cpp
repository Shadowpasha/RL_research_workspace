#include <rclcpp/rclcpp.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h> // For octomap_msgs::buildMsg
#include <octomap/octomap.h>          // For octomap::OcTree
#include <octomap/ColorOcTree.h>      // Specific ColorOcTree header
#include <fstream>                    // For file existence check
#include <sstream>                    // For parsing string streams
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <visualization_msgs/msg/marker_array.hpp> // For publishing object labels and shapes
#include <cmath>                      // For std::sqrt, std::max
#include <algorithm>                  // For std::max with initializer list
#include <chrono>                     // For std::chrono::system_clock
#include <ctime>                      // For std::time_t, std::localtime, std::strftime
#include <iomanip>                    // For std::put_time
#include <filesystem>                 // Added for std::filesystem

// OpenCV includes for image processing and saving
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

// Structure to hold object data read from the file
struct ObjectData {
    std::string class_name;
    double confidence; // New: Confidence score
    double x, y, z;
    double l, w, h;
};

class OctoMapFilePublisher : public rclcpp::Node
{
public:
    OctoMapFilePublisher()
        : Node("octomap_file_publisher")
    {
        // Declare and get parameters
        this->declare_parameter<std::string>("file_path", "/home/anas/Office/brgbd_octomap.ot");
        this->declare_parameter<std::string>("object_file_path", "/home/anas/Office/output.txt"); // New parameter for object file
        this->declare_parameter<std::string>("frame_id", "map");
        this->declare_parameter<std::string>("topic_name", "octomap_full");
        this->declare_parameter<bool>("latch_topic", true);          // Whether to latch the topic
        this->declare_parameter<double>("rotate_roll_deg", -120.0);  // New parameter for roll rotation in degrees
        this->declare_parameter<double>("rotate_pitch_deg", 0.0);    // New parameter for pitch rotation in degrees
        this->declare_parameter<double>("rotate_yaw_deg", 0.0);      // New parameter for yaw rotation in degrees
        this->declare_parameter<double>("translate_x", 0.0);         // New parameter for X translation
        this->declare_parameter<double>("translate_y", -2.0);        // New parameter for Y translation
        this->declare_parameter<double>("translate_z", 1.8);         // New parameter for Z translation
        this->declare_parameter<std::string>("grid_topic_ugv", "occupancy_grid_slice_ugv"); // Output OccupancyGrid topic
        this->declare_parameter<std::string>("grid_topic_uav", "occupancy_grid_slice_uav"); // Output OccupancyGrid topic
        this->declare_parameter<double>("slice_height_z_ugv", 0.2);  // Z-height for the 2D slice UGV
        this->declare_parameter<double>("slice_height_z_uav", 1.4);  // Z-height for the 2D slice UAV
        this->declare_parameter<double>("grid_resolution", 0.1);     // Resolution of the output 2D grid
        this->declare_parameter<double>("occupied_threshold", 0.2);  // Probability threshold for occupied
        this->declare_parameter<double>("free_threshold", 0.8);      // Probability threshold for free
        this->declare_parameter<double>("object_decay_radius_multiplier", 2.5); // Multiplier for object decay radius
        this->declare_parameter<std::string>("heatmap_output_directory", "/home/anas/Office/"); // New: Directory for heatmaps
        this->declare_parameter<bool>("save_heatmaps", true); // New: Enable/disable heatmap saving
        this->declare_parameter<double>("heatmap_resolution_multiplier", 7.0); // New: Multiplier for heatmap resolution

        std::string file_path = this->get_parameter("file_path").as_string();
        object_file_path_ = this->get_parameter("object_file_path").as_string(); // Get new parameter
        std::string frame_id = this->get_parameter("frame_id").as_string();
        std::string topic_name = this->get_parameter("topic_name").as_string();
        bool latch_topic = this->get_parameter("latch_topic").as_bool();
        double rotate_roll_deg = this->get_parameter("rotate_roll_deg").as_double();
        double rotate_pitch_deg = this->get_parameter("rotate_pitch_deg").as_double();
        double rotate_yaw_deg = this->get_parameter("rotate_yaw_deg").as_double();
        double translate_x = this->get_parameter("translate_x").as_double();
        double translate_y = this->get_parameter("translate_y").as_double();
        double translate_z = this->get_parameter("translate_z").as_double();
        std::string grid_topic_ugv = this->get_parameter("grid_topic_ugv").as_string();
        std::string grid_topic_uav = this->get_parameter("grid_topic_uav").as_string();
        double slice_height_z_ugv = this->get_parameter("slice_height_z_ugv").as_double();
        double slice_height_z_uav = this->get_parameter("slice_height_z_uav").as_double();
        double grid_resolution_ = this->get_parameter("grid_resolution").as_double();
        double occupied_threshold_ = this->get_parameter("occupied_threshold").as_double();
        double free_threshold_ = this->get_parameter("free_threshold").as_double();
        object_decay_radius_multiplier_ = this->get_parameter("object_decay_radius_multiplier").as_double();
        heatmap_output_directory_ = this->get_parameter("heatmap_output_directory").as_string();
        save_heatmaps_ = this->get_parameter("save_heatmaps").as_bool();
        heatmap_resolution_multiplier_ = this->get_parameter("heatmap_resolution_multiplier").as_double();


        // Create output directory if it doesn't exist
        if (save_heatmaps_) {
            if (!std::filesystem::exists(heatmap_output_directory_)) {
                std::filesystem::create_directories(heatmap_output_directory_);
                RCLCPP_INFO(this->get_logger(), "Created heatmap output directory: %s", heatmap_output_directory_.c_str());
            }
        }


        // Validate file path
        if (file_path.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "No OctoMap file_path parameter provided. Exiting.");
            rclcpp::shutdown();
            return;
        }

        // Create publisher with QoS settings
        rclcpp::QoS qos_profile(1); // Keep last 1 message
        if (latch_topic)
        {
            qos_profile.transient_local(); // Latch the topic
        }
        else
        {
            qos_profile.durability_volatile(); // Default (no latching)
        }

        octomap_publisher_ = this->create_publisher<octomap_msgs::msg::Octomap>(topic_name, qos_profile);
        grid_pub_ugv = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
            grid_topic_ugv,
            rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

        grid_pub_uav = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
            grid_topic_uav,
            rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
        
        // New publisher for RViz markers
        marker_array_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "object_labels",
            rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());


        RCLCPP_INFO(this->get_logger(), "OctoMap publisher node initialized.");
        RCLCPP_INFO(this->get_logger(), "  File: %s", file_path.c_str());
        RCLCPP_INFO(this->get_logger(), "  Object File: %s", object_file_path_.c_str());
        RCLCPP_INFO(this->get_logger(), "  Topic: %s", topic_name.c_str());
        RCLCPP_INFO(this->get_logger(), "  Frame ID: %s", frame_id.c_str());
        RCLCPP_INFO(this->get_logger(), "  Latching: %s", latch_topic ? "true" : "false");
        if (save_heatmaps_) {
            RCLCPP_INFO(this->get_logger(), "  Heatmaps will be saved to: %s", heatmap_output_directory_.c_str());
        } else {
            RCLCPP_INFO(this->get_logger(), "  Heatmap saving is disabled.");
        }


        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / 10.0),
            std::bind(&OctoMapFilePublisher::publish_map_callback, this));

        // Load and publish the map
        load_and_publish_map(file_path, frame_id, rotate_roll_deg, rotate_pitch_deg, rotate_yaw_deg, translate_x, translate_y, translate_z, slice_height_z_ugv, slice_height_z_uav, grid_resolution_, occupied_threshold_, free_threshold_);
    }

private:
    // Function to load object data from a text file
    std::vector<ObjectData> load_objects_from_file(const std::string &file_path) {
        std::vector<ObjectData> objects;
        std::ifstream file(file_path);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Object file not found at '%s'. No objects will be added.", file_path.c_str());
            return objects;
        }

        std::string line;
        while (std::getline(file, line)) {
            std::istringstream iss(line);
            ObjectData obj;
            std::string obj_prefix; // To read and discard "Obj:"
            // Updated parsing to include confidence score
            if (!(iss >> obj_prefix >> obj.class_name >> obj.confidence >> obj.x >> obj.y >> obj.z >> obj.l >> obj.w >> obj.h)) {
                RCLCPP_WARN(this->get_logger(), "Failed to parse line in object file: %s", line.c_str());
                continue;
            }
            objects.push_back(obj);
        }
        RCLCPP_INFO(this->get_logger(), "Loaded %zu objects from %s", objects.size(), file_path.c_str());
        return objects;
    }

    // Function to publish RViz markers for object labels and spheres
    void publish_object_markers(const std::vector<ObjectData>& objects, const std::string& frame_id) {
        visualization_msgs::msg::MarkerArray marker_array;
        int id_counter = 0;

        for (const auto& obj : objects) {
            // Marker for text label
            visualization_msgs::msg::Marker text_marker;
            text_marker.header.frame_id = frame_id;
            text_marker.header.stamp = this->now();
            text_marker.ns = "object_labels";
            text_marker.id = id_counter++; // Unique ID for each marker
            text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            text_marker.action = visualization_msgs::msg::Marker::ADD;

            text_marker.pose.position.x = obj.x;
            text_marker.pose.position.y = obj.y;
            // Place text slightly above the object's top surface for better visibility
            text_marker.pose.position.z = obj.z + obj.h / 2.0 + 0.1; 
            text_marker.pose.orientation.w = 1.0; // No rotation for text

            text_marker.scale.z = 0.3; // Height of the text

            text_marker.color.a = 1.0; // Alpha (opacity)
            text_marker.color.r = 1.0; // White color
            text_marker.color.g = 1.0;
            text_marker.color.b = 1.0;

            text_marker.text = obj.class_name;
            marker_array.markers.push_back(text_marker);

            // Marker for sphere visualization
            visualization_msgs::msg::Marker sphere_marker;
            sphere_marker.header.frame_id = frame_id;
            sphere_marker.header.stamp = this->now();
            sphere_marker.ns = "object_spheres"; // Different namespace for spheres
            sphere_marker.id = id_counter++; // Unique ID for each marker
            sphere_marker.type = visualization_msgs::msg::Marker::SPHERE;
            sphere_marker.action = visualization_msgs::msg::Marker::ADD;

            sphere_marker.pose.position.x = obj.x;
            sphere_marker.pose.position.y = obj.y;
            sphere_marker.pose.position.z = obj.z; // Center of the sphere at object's center
            sphere_marker.pose.orientation.w = 1.0; // No rotation for sphere

            // Calculate the diameter of the sphere to include the decay region
            double object_radius = std::max(obj.l, obj.w) / 2.0;
            double decay_radius = object_radius * object_decay_radius_multiplier_;
            double sphere_diameter = 2.0 * decay_radius; // Diameter for the sphere marker

            sphere_marker.scale.x = sphere_diameter;
            sphere_marker.scale.y = sphere_diameter;
            sphere_marker.scale.z = sphere_diameter;

            sphere_marker.color.a = 0.3; // Semi-transparent
            sphere_marker.color.r = 1.0; // Bright Red
            sphere_marker.color.g = 0.0;
            sphere_marker.color.b = 0.0;

            marker_array.markers.push_back(sphere_marker);
        }
        marker_array_pub_->publish(marker_array);
    }

    void generate_and_save_heatmap(const nav_msgs::msg::OccupancyGrid& grid_msg, const std::string& filename_prefix) {
        if (!save_heatmaps_) {
            return; // Do not save if disabled by parameter
        }

        if (grid_msg.info.width == 0 || grid_msg.info.height == 0) {
            RCLCPP_WARN(this->get_logger(), "Received empty grid from %s topic. Skipping heatmap generation.", filename_prefix.c_str());
            return;
        }

        // Create an OpenCV Mat from the OccupancyGrid data
        // OccupancyGrid.data is row-major, so use height, then width
        cv::Mat original_grid_image(grid_msg.info.height, grid_msg.info.width, CV_8UC1);

        for (unsigned int y = 0; y < grid_msg.info.height; ++y) {
            for (unsigned int x = 0; x < grid_msg.info.width; ++x) {
                int8_t occupancy_value = grid_msg.data[y * grid_msg.info.width + x];
                
                // Map ROS occupancy values to 0-255 for OpenCV
                // -1 (unknown) -> 0 (black/free)
                // 0 (free) -> 0 (black)
                // 100 (occupied) -> 255 (white)
                unsigned char pixel_value;
                if (occupancy_value == -1) {
                    pixel_value = 0; // Treat unknown as free/cold for heatmap
                } else {
                    // Scale 0-100 to 0-255
                    pixel_value = static_cast<unsigned char>(occupancy_value * 2.55);
                }
                original_grid_image.at<unsigned char>(y, x) = pixel_value;
            }
        }

        // Calculate new dimensions for higher resolution heatmap
        int new_width = static_cast<int>(grid_msg.info.width * heatmap_resolution_multiplier_);
        int new_height = static_cast<int>(grid_msg.info.height * heatmap_resolution_multiplier_);

        cv::Mat resized_grid_image;
        cv::resize(original_grid_image, resized_grid_image, cv::Size(new_width, new_height), 0, 0, cv::INTER_LINEAR);

        // Apply a colormap (e.g., COLORMAP_JET, COLORMAP_HOT, COLORMAP_RAINBOW)
        cv::Mat heatmap_image;
        cv::applyColorMap(resized_grid_image, heatmap_image, cv::COLORMAP_JET); // COLORMAP_JET is a good general heatmap

        // Construct filename with timestamp
        auto now = std::chrono::system_clock::now();
        std::time_t current_time = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        ss << std::put_time(std::localtime(&current_time), "%Y%m%d_%H%M%S");
        std::string timestamp_str = ss.str();

        std::string filename = heatmap_output_directory_ + "/" + filename_prefix + ".png";

        // Save the image
        try {
            cv::imwrite(filename, heatmap_image);
            RCLCPP_INFO(this->get_logger(), "Saved heatmap to %s", filename.c_str());
        } catch (const cv::Exception& ex) {
            RCLCPP_ERROR(this->get_logger(), "Error saving heatmap to %s: %s", filename.c_str(), ex.what());
        }
    }

    void load_and_publish_map(const std::string &file_path, const std::string &frame_id,
        double roll_deg, double pitch_deg, double yaw_deg, 
        double translate_x, double translate_y, double translate_z, double slice_height_z_ugv, double slice_height_z_uav, double grid_resolution_, double occupied_threshold_, double free_threshold_
        )
    {
        // Check if file exists
        std::ifstream file_check(file_path);
        if (!file_check.good())
        {
            RCLCPP_ERROR(this->get_logger(), "OctoMap file not found at '%s'. Exiting.", file_path.c_str());
            rclcpp::shutdown();
            return;
        }

        // Load the OctoMap tree from the binary file
        octomap::AbstractOcTree *tree = octomap::AbstractOcTree::read(file_path);

        if (!tree)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to read OctoMap from '%s'. Is it a valid .ot file?", file_path.c_str());
            rclcpp::shutdown();
            return;
        }

        // Cast to ColorOcTree
        octomap::ColorOcTree *color_octree = dynamic_cast<octomap::ColorOcTree *>(tree);

        if (!color_octree)
        {
            RCLCPP_ERROR(this->get_logger(), "Loaded OctoMap is not a Color OcTree. Cannot publish.");
            delete tree; // Clean up memory
            rclcpp::shutdown();
            return;
        }

        // Create a new ColorOcTree for the transformed map
        octomap::ColorOcTree *transformed_color_octree = new octomap::ColorOcTree(color_octree->getResolution());
        RCLCPP_INFO(this->get_logger(), "Applying transformation to OctoMap by re-inserting nodes...");

        // Convert degrees to radians
        double roll_rad = roll_deg * M_PI / 180.0;
        double pitch_rad = pitch_deg * M_PI / 180.0;
        double yaw_rad = yaw_deg * M_PI / 180.0;

        // Create an Eigen quaternion from Euler angles (ZYX order for roll, pitch, yaw)
        Eigen::Quaterniond q = Eigen::AngleAxisd(yaw_rad, Eigen::Vector3d::UnitZ()) *
                               Eigen::AngleAxisd(pitch_rad, Eigen::Vector3d::UnitY()) *
                               Eigen::AngleAxisd(roll_rad, Eigen::Vector3d::UnitX());

        // Create an Eigen Isometry3d transformation
        Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
        transform.rotate(q);
        transform.translation() << translate_x, translate_y, translate_z;

        // Iterate over all occupied nodes in the original tree and transform them
        for (octomap::ColorOcTree::iterator it = color_octree->begin(),
                                            end = color_octree->end();
             it != end; ++it)
        {
            if (color_octree->isNodeOccupied(*it))
            {
                // Get the center of the current node
                octomap::point3d p = it.getCoordinate();

                // Convert octomap::point3d to Eigen::Vector3d
                Eigen::Vector3d eigen_p(p.x(), p.y(), p.z());

                // Apply the transformation
                Eigen::Vector3d transformed_p = transform * eigen_p;

                // Get color from the original node
                octomap::ColorOcTreeNode::Color node_color = it->getColor();

                // Insert the transformed point with its color into the new tree
                transformed_color_octree->updateNode(transformed_p.x(), transformed_p.y(), transformed_p.z(), true); // Occupied
                transformed_color_octree->averageNodeColor(transformed_p.x(), transformed_p.y(), transformed_p.z(),
                                                            node_color.r, node_color.g, node_color.b);
            }
        }
        transformed_color_octree->updateInnerOccupancy(); // Update inner nodes for correct representation
        RCLCPP_INFO(this->get_logger(), "Transformation applied successfully by re-inserting nodes.");

        // Get OctoMap bounds
        double minX, minY, minZ, maxX, maxY, maxZ;
        transformed_color_octree->getMetricMin(minX, minY, minZ);
        transformed_color_octree->getMetricMax(maxX, maxY, maxZ);

        RCLCPP_INFO(this->get_logger(), "OctoMap bounds: X[%.2f, %.2f], Y[%.2f, %.2f], Z[%.2f, %.2f]",
                    minX, maxX, minY, maxY, minZ, maxZ);

        // Check if slice_height_z is within the OctoMap's Z bounds
        if ((slice_height_z_ugv < minZ || slice_height_z_ugv > maxZ) && (slice_height_z_uav < minZ || slice_height_z_uav > maxZ))
        {
            RCLCPP_WARN(this->get_logger(), "Both slice heights (UGV: %.2f m, UAV: %.2f m) are outside OctoMap Z bounds [%.2f, %.2f]. The resulting grid might be empty.",
                        slice_height_z_ugv, slice_height_z_uav, minZ, maxZ);
        }

        // Calculate grid dimensions
        unsigned int sizeX = static_cast<unsigned int>(std::ceil((maxX - minX) / grid_resolution_));
        unsigned int sizeY = static_cast<unsigned int>(std::ceil((maxY - minY) / grid_resolution_));

        if (sizeX == 0 || sizeY == 0)
        {
            RCLCPP_WARN(this->get_logger(), "Calculated grid dimensions are zero. OctoMap might be too small or resolution too high.");
            return;
        }

        // Create OccupancyGrid messages
        grid_msg_ugv.header.frame_id = "map"; // Use the same frame as the OctoMap
        grid_msg_ugv.header.stamp = this->now();
        grid_msg_ugv.info.resolution = grid_resolution_;
        grid_msg_ugv.info.width = sizeX;
        grid_msg_ugv.info.height = sizeY;
        grid_msg_ugv.info.origin.position.x = minX;
        grid_msg_ugv.info.origin.position.y = minY;
        grid_msg_ugv.info.origin.position.z = slice_height_z_ugv;
        grid_msg_ugv.info.origin.orientation.w = 1.0;

        grid_msg_uav.header.frame_id = "map";
        grid_msg_uav.header.stamp = this->now();
        grid_msg_uav.info.resolution = grid_resolution_;
        grid_msg_uav.info.width = sizeX;
        grid_msg_uav.info.height = sizeY;
        grid_msg_uav.info.origin.position.x = minX;
        grid_msg_uav.info.origin.position.y = minY;
        grid_msg_uav.info.origin.position.z = slice_height_z_uav;
        grid_msg_uav.info.origin.orientation.w = 1.0;

        // Initialize grid data to -1 (unknown)
        grid_msg_ugv.data.assign(sizeX * sizeY, -1);
        grid_msg_uav.data.assign(sizeX * sizeY, -1);

        // Iterate through the 2D grid cells and query the OctoMap
        for (unsigned int y_idx = 0; y_idx < sizeY; ++y_idx)
        {
            for (unsigned int x_idx = 0; x_idx < sizeX; ++x_idx)
            {
                double x_coord = minX + (x_idx + 0.5) * grid_resolution_; // Center of the cell
                double y_coord = minY + (y_idx + 0.5) * grid_resolution_; // Center of the cell

                // Process for UGV grid
                octomap::OcTreeNode *node_ugv = transformed_color_octree->search(x_coord, y_coord, slice_height_z_ugv);
                int8_t occupancy_value_ugv = -1; // Default to unknown
                if (node_ugv)
                {
                    double prob = node_ugv->getOccupancy();
                    if (prob >= occupied_threshold_)
                    {
                        occupancy_value_ugv = 100; // Occupied
                    }
                    else if (prob <= free_threshold_)
                    {
                        occupancy_value_ugv = 0; // Free
                    }
                } else {
                    occupancy_value_ugv = 0; // Assume free if no node found
                }
                grid_msg_ugv.data[y_idx * sizeX + x_idx] = occupancy_value_ugv;

                // Process for UAV grid
                octomap::OcTreeNode *node_uav = transformed_color_octree->search(x_coord, y_coord, slice_height_z_uav);
                int8_t occupancy_value_uav = -1; // Default to unknown
                if (node_uav)
                {
                    double prob = node_uav->getOccupancy();
                    if (prob >= occupied_threshold_)
                    {
                        occupancy_value_uav = 100; // Occupied
                    }
                    else if (prob <= free_threshold_)
                    {
                        occupancy_value_uav = 0; // Free
                    }
                } else {
                    occupancy_value_uav = 0; // Assume free if no node found
                }
                grid_msg_uav.data[y_idx * sizeX + x_idx] = occupancy_value_uav;
            }
        }

        // --- Add objects to OccupancyGrid ---
        objects_ = load_objects_from_file(object_file_path_); // Load objects

        // Apply the same transformation to object positions
        for (auto& obj : objects_) {
            Eigen::Vector3d eigen_obj_pos(obj.x, obj.y, obj.z);
            Eigen::Vector3d transformed_obj_pos = transform * eigen_obj_pos;
            obj.x = transformed_obj_pos.x();
            obj.y = transformed_obj_pos.y();
            obj.z = transformed_obj_pos.z();
        }

        for (const auto& obj : objects_) {
            // Calculate effective radius for the obstacle (using max of L or W)
            double object_radius = std::max(obj.l, obj.w) / 2.0;
            // Define a decay radius, beyond which occupancy becomes 0
            double decay_radius = object_radius * object_decay_radius_multiplier_;

            // Check if object's Z range (including decay) overlaps with UGV slice height
            bool affects_ugv_slice = (obj.z - decay_radius <= slice_height_z_ugv && obj.z + decay_radius >= slice_height_z_ugv);
            // Check if object's Z range (including decay) overlaps with UAV slice height
            bool affects_uav_slice = (obj.z - decay_radius <= slice_height_z_uav && obj.z + decay_radius >= slice_height_z_uav);

            // Iterate over grid cells around the object
            // Determine the bounding box for the object's influence to optimize iteration
            int min_x_idx = static_cast<int>(std::floor((obj.x - decay_radius - minX) / grid_resolution_));
            int max_x_idx = static_cast<int>(std::ceil((obj.x + decay_radius - minX) / grid_resolution_));
            int min_y_idx = static_cast<int>(std::floor((obj.y - decay_radius - minY) / grid_resolution_));
            int max_y_idx = static_cast<int>(std::ceil((obj.y + decay_radius - minY) / grid_resolution_));

            // Clamp indices to grid boundaries
            min_x_idx = std::max(0, min_x_idx);
            max_x_idx = std::min(static_cast<int>(sizeX - 1), max_x_idx);
            min_y_idx = std::max(0, min_y_idx);
            max_y_idx = std::min(static_cast<int>(sizeY - 1), max_y_idx);

            for (int y_idx = min_y_idx; y_idx <= max_y_idx; ++y_idx)
            {
                for (int x_idx = min_x_idx; x_idx <= max_x_idx; ++x_idx)
                {
                    double cell_center_x = minX + (x_idx + 0.5) * grid_resolution_;
                    double cell_center_y = minY + (y_idx + 0.5) * grid_resolution_;

                    double dist_to_object_center = std::sqrt(
                        std::pow(cell_center_x - obj.x, 2) +
                        std::pow(cell_center_y - obj.y, 2)
                    );

                    if (dist_to_object_center < decay_radius) {
                        int8_t new_occupancy = 0;
                        if (dist_to_object_center <= object_radius) {
                            // Fully occupied within the object's main radius
                            new_occupancy = 100;
                        } else {
                            // Gradual decay outside the main radius
                            // Linear decay: 100 at object_radius, 0 at decay_radius
                            double decay_factor = (dist_to_object_center - object_radius) / (decay_radius - object_radius);
                            new_occupancy = static_cast<int8_t>(100.0 * (1.0 - decay_factor));
                            // Ensure it doesn't go below 0
                            new_occupancy = std::max((int8_t)0, new_occupancy);
                        }

                        // Update UGV grid
                        if (affects_ugv_slice) {
                            // Take the maximum of current occupancy and new object-based occupancy
                            grid_msg_ugv.data[y_idx * sizeX + x_idx] = std::max(grid_msg_ugv.data[y_idx * sizeX + x_idx], new_occupancy);
                        }

                        // Update UAV grid
                        if (affects_uav_slice) {
                             // Take the maximum of current occupancy and new object-based occupancy
                            grid_msg_uav.data[y_idx * sizeX + x_idx] = std::max(grid_msg_uav.data[y_idx * sizeX + x_idx], new_occupancy);
                        }
                    }
                }
            }
        }
        // --- End of adding objects ---

        // Publish the OccupancyGrid messages
        grid_pub_ugv->publish(grid_msg_ugv);
        grid_pub_uav->publish(grid_msg_uav);

        // Create ROS2 OctoMap message
        if (!octomap_msgs::binaryMapToMsg(*transformed_color_octree, published_octomap_msg_))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to convert OctoMap to ROS message.");
            delete tree; // Clean up memory
            rclcpp::shutdown();
            return;
        }

        // Set header information
        published_octomap_msg_.header.frame_id = frame_id;
        published_octomap_msg_.header.stamp = this->now();

        RCLCPP_INFO(this->get_logger(), "Successfully loaded OctoMap (resolution: %.3fm, tree type: %s) from '%s'.",
                    transformed_color_octree->getResolution(), transformed_color_octree->getTreeType().c_str(), file_path.c_str());

        // Publish the object markers
        publish_object_markers(objects_, frame_id);

        // Generate and save heatmaps
        generate_and_save_heatmap(grid_msg_ugv, "ugv_heatmap");
        generate_and_save_heatmap(grid_msg_uav, "uav_heatmap");


        delete tree; // Clean up memory after building the message
        delete transformed_color_octree; // Clean up transformed tree
    }

    void publish_map_callback()
    {
        // Update timestamp before publishing
        grid_msg_ugv.header.stamp = this->now();
        grid_msg_uav.header.stamp = this->now();
        published_octomap_msg_.header.stamp = this->now();

        grid_pub_ugv->publish(grid_msg_ugv);
        grid_pub_uav->publish(grid_msg_uav);
        octomap_publisher_->publish(published_octomap_msg_);
        
        // Republish markers to ensure they stay visible in RViz
        if (!objects_.empty()) {
            publish_object_markers(objects_, published_octomap_msg_.header.frame_id);
        }

      
        
    }

    rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr octomap_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    octomap_msgs::msg::Octomap published_octomap_msg_; // Stored message for repeated publishing
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_ugv;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_uav;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_pub_; // New marker publisher
    nav_msgs::msg::OccupancyGrid grid_msg_ugv;
    nav_msgs::msg::OccupancyGrid grid_msg_uav;
    std::string object_file_path_;
    std::vector<ObjectData> objects_; // Store loaded objects
    double object_decay_radius_multiplier_; // Parameter for decay
    std::string heatmap_output_directory_; // New parameter for heatmap output directory
    bool save_heatmaps_; // New parameter to enable/disable heatmap saving
    double heatmap_resolution_multiplier_; // New parameter for heatmap resolution multiplier

};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OctoMapFilePublisher>();
    rclcpp::spin(node); // Keep the node alive
    rclcpp::shutdown();
    return 0;
}
