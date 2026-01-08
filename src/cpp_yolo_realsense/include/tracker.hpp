#ifndef TRACKER_HPP
#define TRACKER_HPP

#include <opencv2/opencv.hpp>
#include <opencv2/video/tracking.hpp> // 修复：使用正确的头文件
#include <vector>
#include <deque>
#include <algorithm>

struct TrackObject {
    cv::Rect rect;
    int label;
    float prob;
    int track_id = -1;
};

class KalmanTrack {
public:
    KalmanTrack(cv::Rect rect, int id) : track_id(id), time_since_update(0) {
        kf = cv::KalmanFilter(4, 2, 0);
        kf.transitionMatrix = (cv::Mat_<float>(4, 4) << 1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1);
        
        kf.statePre.at<float>(0) = rect.x + rect.width / 2;
        kf.statePre.at<float>(1) = rect.y + rect.height / 2;
        kf.statePre.at<float>(2) = rect.width;
        kf.statePre.at<float>(3) = rect.height;
        
        kf.statePost = kf.statePre.clone();
        cv::setIdentity(kf.measurementMatrix);
        cv::setIdentity(kf.processNoiseCov, cv::Scalar::all(1e-4));
        cv::setIdentity(kf.measurementNoiseCov, cv::Scalar::all(1e-1));
        cv::setIdentity(kf.errorCovPost, cv::Scalar::all(.1));
        
        last_rect = rect;
    }

    cv::Rect predict() {
        cv::Mat p = kf.predict();
        last_rect.x = p.at<float>(0) - p.at<float>(2) / 2;
        last_rect.y = p.at<float>(1) - p.at<float>(3) / 2;
        last_rect.width = p.at<float>(2);
        last_rect.height = p.at<float>(3);
        time_since_update++;
        return last_rect;
    }

    void update(cv::Rect rect) {
        cv::Mat measurement(2, 1, CV_32F);
        measurement.at<float>(0) = rect.x + rect.width / 2;
        measurement.at<float>(1) = rect.y + rect.height / 2;
        kf.statePre.at<float>(2) = rect.width;
        kf.statePre.at<float>(3) = rect.height;
        kf.correct(measurement);
        last_rect = rect;
        time_since_update = 0;
    }

    int track_id;
    int time_since_update;
    cv::Rect last_rect;
    cv::KalmanFilter kf;
};

class ByteTracker {
public:
    ByteTracker(int max_lost = 30) : max_time_lost(max_lost), next_id(1) {}

    std::vector<TrackObject> update(std::vector<TrackObject>& detections) {
        std::vector<TrackObject> results;

        for (auto& track : tracks) track.predict();

        std::vector<bool> det_matched(detections.size(), false);
        std::vector<bool> track_matched(tracks.size(), false);

        for (size_t t = 0; t < tracks.size(); t++) {
            if (track_matched[t]) continue;
            int best_det = -1;
            float best_iou = 0.1f;
            for (size_t d = 0; d < detections.size(); d++) {
                if (det_matched[d]) continue;
                float iou = getIoU(tracks[t].last_rect, detections[d].rect);
                if (iou > best_iou) { best_iou = iou; best_det = d; }
            }
            if (best_det != -1) {
                tracks[t].update(detections[best_det].rect);
                track_matched[t] = true; det_matched[best_det] = true;
                TrackObject obj = detections[best_det];
                obj.track_id = tracks[t].track_id;
                results.push_back(obj);
            }
        }

        for (size_t d = 0; d < detections.size(); d++) {
            if (!det_matched[d]) {
                tracks.emplace_back(detections[d].rect, next_id++);
                TrackObject obj = detections[d];
                obj.track_id = next_id - 1;
                results.push_back(obj);
            }
        }

        for (auto it = tracks.begin(); it != tracks.end();) {
            if (it->time_since_update > max_time_lost) it = tracks.erase(it);
            else ++it;
        }
        return results;
    }

private:
    std::vector<KalmanTrack> tracks;
    int max_time_lost;
    int next_id;

    float getIoU(cv::Rect box1, cv::Rect box2) {
        int x1 = std::max(box1.x, box2.x);
        int y1 = std::max(box1.y, box2.y);
        int x2 = std::min(box1.x + box1.width, box2.x + box2.width);
        int y2 = std::min(box1.y + box1.height, box2.y + box2.height);
        if (x1 >= x2 || y1 >= y2) return 0.0f;
        float intersection = (float)(x2 - x1) * (y2 - y1);
        return intersection / (box1.width * box1.height + box2.width * box2.height - intersection);
    }
};

#endif