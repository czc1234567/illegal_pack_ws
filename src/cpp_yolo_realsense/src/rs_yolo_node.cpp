#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <opencv2/highgui.hpp>

// 替换为标准可视化消息
#include <visualization_msgs/msg/marker_array.hpp>

#include "yolo_trt.hpp"
#include "tracker.hpp"

// COCO 80类 (保持不变)
const std::vector<std::string> COCO_CLASSES = {
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat", "traffic light",
    "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow",
    "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
    "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove", "skateboard", "surfboard",
    "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple",
    "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch",
    "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote", "keyboard", "cell phone",
    "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors", "teddy bear",
    "hair drier", "toothbrush"
};

using SyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>;

class RealSenseYoloNode : public rclcpp::Node {
public:
    RealSenseYoloNode() : Node("realsense_yolo_cpp") {
        this->declare_parameter<std::string>("engine_path", "");
        std::string engine_path = this->get_parameter("engine_path").as_string();
        
        if (engine_path.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Please set 'engine_path' parameter!");
            return; 
        }

        detector_ = std::make_unique<YoloDetector>(engine_path);
        tracker_ = std::make_unique<ByteTracker>(30);
        RCLCPP_INFO(this->get_logger(), "Engine Loaded: %s", engine_path.c_str());

        info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/camera/camera/color/camera_info", 10, 
            std::bind(&RealSenseYoloNode::infoCallback, this, std::placeholders::_1));

        rgb_sub_.subscribe(this, "/camera/camera/color/image_raw");
        depth_sub_.subscribe(this, "/camera/camera/depth/image_rect_raw");

        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(20), rgb_sub_, depth_sub_);
        sync_->registerCallback(std::bind(&RealSenseYoloNode::syncCallback, this, std::placeholders::_1, std::placeholders::_2));

        // 修改：发布 MarkerArray (标准消息)
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/yolo/markers_3d", 10);
        
        cv::namedWindow("YOLOv11 3D", cv::WINDOW_AUTOSIZE);
    }

private:
    void infoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        if (has_intrinsics_) return;
        fx_ = msg->k[0]; fy_ = msg->k[4];
        cx_ = msg->k[2]; cy_ = msg->k[5];
        has_intrinsics_ = true;
    }

    void syncCallback(const sensor_msgs::msg::Image::ConstSharedPtr& rgb_msg, 
                      const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg) {
        if (!has_intrinsics_) return; 

        cv::Mat color_frame, depth_frame;
        try {
            color_frame = cv_bridge::toCvShare(rgb_msg, "bgr8")->image;
            depth_frame = cv_bridge::toCvShare(depth_msg, "16UC1")->image;
        } catch (cv_bridge::Exception& e) { return; }

        auto start = std::chrono::high_resolution_clock::now();

        // 1. 推理 & 跟踪
        auto raw_detections = detector_->detect(color_frame);
        std::vector<TrackObject> track_inputs;
        for (const auto& det : raw_detections) {
            track_inputs.push_back({det.box, det.class_id, det.confidence, -1});
        }
        auto tracked_objects = tracker_->update(track_inputs);

        // 2. 准备 MarkerArray
        visualization_msgs::msg::MarkerArray marker_array;
        
        // 先添加一个 DELETEALL 的标记，清除上一帧的旧残留
        visualization_msgs::msg::Marker delete_marker;
        delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        marker_array.markers.push_back(delete_marker);

        // 3. 处理每个目标
        for (const auto& obj : tracked_objects) {
            float dist_mm = getDepth(obj.rect, depth_frame);
            
            if (dist_mm > 0 && dist_mm <= 5000) {
                float z = dist_mm / 1000.0f;
                int u = obj.rect.x + obj.rect.width / 2;
                int v = obj.rect.y + obj.rect.height / 2;
                float x = (u - cx_) * z / fx_;
                float y = (v - cy_) * z / fy_;

                std::string class_name = "unknown";
                if (obj.label >= 0 && obj.label < (int)COCO_CLASSES.size()) {
                    class_name = COCO_CLASSES[obj.label];
                }

                // === 构建 3D 文字标记 (包含 ID 和类别) ===
                visualization_msgs::msg::Marker marker;
                marker.header = rgb_msg->header; // 重要：继承相机坐标系
                marker.ns = class_name;          // 命名空间用作类别名
                marker.id = obj.track_id;        // ID 用作 Track ID
                marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
                marker.action = visualization_msgs::msg::Marker::ADD;
                
                marker.pose.position.x = x;
                marker.pose.position.y = y;
                marker.pose.position.z = z;
                marker.pose.orientation.w = 1.0;
                
                // 设置显示文本
                marker.text = class_name + "#" + std::to_string(obj.track_id);
                
                marker.scale.z = 0.2; // 文字高度 20cm
                marker.color.r = 0.0f; marker.color.g = 1.0f; marker.color.b = 0.0f; marker.color.a = 1.0f;
                marker.lifetime = rclcpp::Duration::from_seconds(0.2); // 0.2秒后自动消失

                marker_array.markers.push_back(marker);

                // === 构建 3D 球体标记 (显示具体位置) ===
                visualization_msgs::msg::Marker sphere = marker;
                sphere.id = obj.track_id + 10000; // 避免ID冲突
                sphere.type = visualization_msgs::msg::Marker::SPHERE;
                sphere.text = "";
                sphere.scale.x = 0.1; sphere.scale.y = 0.1; sphere.scale.z = 0.1; // 10cm 的球
                sphere.color.r = 1.0f; sphere.color.g = 0.0f; sphere.color.b = 0.0f; sphere.color.a = 0.8f;
                marker_array.markers.push_back(sphere);

                // 2D 绘图
                cv::rectangle(color_frame, obj.rect, cv::Scalar(0, 255, 0), 2);
                std::string label = class_name + " ID:" + std::to_string(obj.track_id) + 
                                    " " + cv::format("%.2fm", z);
                cv::putText(color_frame, label, cv::Point(obj.rect.x, obj.rect.y - 5), 
                            cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
            }
        }

        // 4. 发布
        marker_pub_->publish(marker_array);

        // FPS
        auto end = std::chrono::high_resolution_clock::now();
        float ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
        cv::putText(color_frame, "FPS: " + std::to_string(int(1000.0 / (ms + 1e-5))), 
                    cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 255), 2);

        cv::imshow("YOLOv11 3D", color_frame);
        cv::waitKey(1);
    }

    float getDepth(const cv::Rect& box, const cv::Mat& depth_img) {
        int cx = std::clamp(box.x + box.width / 2, 0, depth_img.cols - 1);
        int cy = std::clamp(box.y + box.height / 2, 0, depth_img.rows - 1);
        std::vector<uint16_t> depths;
        for (int y = std::max(0, cy-1); y <= std::min(depth_img.rows-1, cy+1); y++) {
            for (int x = std::max(0, cx-1); x <= std::min(depth_img.cols-1, cx+1); x++) {
                uint16_t d = depth_img.at<uint16_t>(y, x);
                if (d > 0) depths.push_back(d);
            }
        }
        if (depths.empty()) return 0.0f;
        std::nth_element(depths.begin(), depths.begin() + depths.size()/2, depths.end());
        return (float)depths[depths.size()/2];
    }

    std::unique_ptr<YoloDetector> detector_;
    std::unique_ptr<ByteTracker> tracker_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> rgb_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
    
    // 修改为 MarkerArray 发布器
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

    bool has_intrinsics_ = false;
    float fx_, fy_, cx_, cy_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RealSenseYoloNode>());
    rclcpp::shutdown();
    return 0;
}