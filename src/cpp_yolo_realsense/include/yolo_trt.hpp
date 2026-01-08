#ifndef YOLO_TRT_HPP
#define YOLO_TRT_HPP

#include <fstream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <NvInfer.h>
#include <cuda_runtime_api.h>

// 检测结果结构体
struct Detection {
    int class_id;
    float confidence;
    cv::Rect box;
};

class Logger : public nvinfer1::ILogger {
    void log(Severity severity, const char* msg) noexcept override {
        // 只打印错误信息，保持终端干净
        if (severity <= Severity::kERROR) std::cout << "[TRT] " << msg << std::endl;
    }
};

class YoloDetector {
public:
    YoloDetector(const std::string& engine_path);
    ~YoloDetector();
    std::vector<Detection> detect(cv::Mat& frame);

private:
    void loadEngine(const std::string& path);
    void postprocess(const cv::Mat& img, std::vector<Detection>& outputs);

    nvinfer1::IRuntime* runtime = nullptr;
    nvinfer1::ICudaEngine* engine = nullptr;
    nvinfer1::IExecutionContext* context = nullptr;
    cudaStream_t stream;
    Logger gLogger;

    void* device_buffers[2];
    float* host_input = nullptr;
    float* host_output = nullptr;

    const std::string INPUT_NAME = "images";
    const std::string OUTPUT_NAME = "output0";
    const int INPUT_W = 640;
    const int INPUT_H = 640;
    size_t input_size;
    size_t output_size;
};

// implementation

inline YoloDetector::YoloDetector(const std::string& path) {
    loadEngine(path);
    input_size = 1 * 3 * INPUT_H * INPUT_W * sizeof(float);
    
    // === 修改点：适配 COCO 80类 ===
    // 84 = 4 (cx,cy,w,h) + 80 (classes)
    // 8400 = 锚框数量 (对于 640x640 输入)
    output_size = 1 * 84 * 8400 * sizeof(float); 
    
    cudaMalloc(&device_buffers[0], input_size);
    cudaMalloc(&device_buffers[1], output_size);
    
    host_input = new float[1 * 3 * INPUT_H * INPUT_W];
    host_output = new float[1 * 84 * 8400]; // 确保申请足够大的内存

    context->setTensorAddress(INPUT_NAME.c_str(), device_buffers[0]);
    context->setTensorAddress(OUTPUT_NAME.c_str(), device_buffers[1]);
    cudaStreamCreate(&stream);
}

inline YoloDetector::~YoloDetector() {
    cudaStreamDestroy(stream);
    cudaFree(device_buffers[0]);
    cudaFree(device_buffers[1]);
    delete[] host_input;
    delete[] host_output;
    delete context; delete engine; delete runtime;
}

inline void YoloDetector::loadEngine(const std::string& path) {
    std::ifstream file(path, std::ios::binary | std::ios::ate);
    std::streamsize size = file.tellg();
    file.seekg(0, std::ios::beg);
    std::vector<char> buffer(size);
    file.read(buffer.data(), size);
    runtime = nvinfer1::createInferRuntime(gLogger);
    engine = runtime->deserializeCudaEngine(buffer.data(), size);
    context = engine->createExecutionContext();
}

inline std::vector<Detection> YoloDetector::detect(cv::Mat& frame) {
    // 1. Preprocess
    cv::Mat resized;
    cv::resize(frame, resized, cv::Size(INPUT_W, INPUT_H));
    int area = INPUT_W * INPUT_H;
    float* p_input = host_input;
    unsigned char* p_img = resized.data;
    // HWC -> CHW, BGR -> RGB, Normalize
    for (int i = 0; i < area; ++i, p_img += 3) {
        p_input[i] = p_img[0] / 255.0f;        // B
        p_input[i + area] = p_img[1] / 255.0f; // G
        p_input[i + 2 * area] = p_img[2] / 255.0f; // R
    }

    // 2. Infer
    cudaMemcpyAsync(device_buffers[0], host_input, input_size, cudaMemcpyHostToDevice, stream);
    context->enqueueV3(stream);
    cudaMemcpyAsync(host_output, device_buffers[1], output_size, cudaMemcpyDeviceToHost, stream);
    cudaStreamSynchronize(stream);

    // 3. Postprocess
    std::vector<Detection> detections;
    postprocess(frame, detections);
    return detections;
}

inline void YoloDetector::postprocess(const cv::Mat& img, std::vector<Detection>& outputs) {
    // === 修改点：类别数设为 80 ===
    int nc = 80; 
    int anchors = 8400;
    
    float x_factor = (float)img.cols / INPUT_W;
    float y_factor = (float)img.rows / INPUT_H;

    std::vector<int> class_ids;
    std::vector<float> confidences;
    std::vector<cv::Rect> boxes;

    for (int i = 0; i < anchors; i++) {
        float max_score = 0.0f;
        int class_id = -1;
        // 寻找最大类别分数
        for (int c = 0; c < nc; c++) {
            float score = host_output[(4 + c) * anchors + i];
            if (score > max_score) { max_score = score; class_id = c; }
        }

        if (max_score > 0.25f) {
            float cx = host_output[0 * anchors + i];
            float cy = host_output[1 * anchors + i];
            float w = host_output[2 * anchors + i];
            float h = host_output[3 * anchors + i];
            
            int left = int((cx - 0.5 * w) * x_factor);
            int top = int((cy - 0.5 * h) * y_factor);
            int width = int(w * x_factor);
            int height = int(h * y_factor);
            
            boxes.push_back(cv::Rect(left, top, width, height));
            confidences.push_back(max_score);
            class_ids.push_back(class_id);
        }
    }

    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes, confidences, 0.25f, 0.45f, indices);
    for (int idx : indices) {
        outputs.push_back({class_ids[idx], confidences[idx], boxes[idx]});
    }
}

#endif