#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp> 
#include <iostream>   
#include <fstream>             
#include <vector>
#include <unordered_map>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

struct ArucoConfig {
    int dictionaryId;
    std::vector<int> markerIds;
    std::vector<float> markerLengths;
};

class arucoDet
{
private:
    ros::NodeHandle nh;
    ros::Subscriber image_sub;
    std::string image_topic;

    std::string json_path;

    cv::Ptr<cv::aruco::DetectorParameters> detectorParams;
    cv::Ptr<cv::aruco::Dictionary> dictionary;
    std::unordered_map<int, cv::Mat> id_length_map;
    std::unordered_map<int, float> length_map;

    cv::Mat camMatrix, distCoeffs;

    ArucoConfig config;
    int cnt = 0;

public:
    arucoDet()
    {
        std::vector<float> camMatrix_vec;
        nh.param<std::vector<float>>("arucoDet/camMatrix", camMatrix_vec, std::vector<float>());
        // deep copy
        camMatrix = cv::Mat(3, 3, CV_32F, camMatrix_vec.data()).clone();

        std::vector<float> distCoeffs_vec;
        nh.param<std::vector<float>>("arucoDet/distCoeffs", distCoeffs_vec, std::vector<float>());
        distCoeffs = cv::Mat(1, 5, CV_32F, distCoeffs_vec.data()).clone();

        nh.param<std::string>("arucoDet/image_topic", image_topic, "/camera/image_raw");
        image_sub = nh.subscribe<sensor_msgs::Image>(image_topic, 1, &arucoDet::imageCallback, this);

        nh.param<std::string>("arucoDet/json_path", json_path, "./params.json");
        if(loadParams(json_path))
        {
            detectorParams = cv::aruco::DetectorParameters::create();
            dictionary = cv::aruco::getPredefinedDictionary(config.dictionaryId);
        }
    }

    bool loadParams(const std::string& file_path)
    {
        std::ifstream json_file(file_path);
        if(!json_file.is_open())
        {
            std::cerr << "file not found" << std::endl;
            return false;
        }

        json json_reader;
        try {
            json_file >> json_reader;
        }
        catch (const json::parse_error& e) {
            std::cerr << "failed to read file" << std::endl;
            return false;
        }

        try {
            config.dictionaryId = json_reader["ArucoDetector"]["dictionaryId"];
            config.markerIds = json_reader["ArucoDetector"]["markerIds"].get<std::vector<int>>();
            config.markerLengths = json_reader["ArucoDetector"]["markerLengths"].get<std::vector<float>>();
        }
        catch (const json::exception& e) {
            std::cerr << "json field error" << std::endl;
            return false;
        }

        for(size_t i = 0; i < config.markerIds.size(); i++)
        {
            int id = config.markerIds[i];
            float markerLength = config.markerLengths[i];
            cv::Mat marker_coords(4, 1, CV_32FC3);
            marker_coords.ptr<cv::Vec3f>(0)[0] = cv::Vec3f(-markerLength/2.f, markerLength/2.f, 0);
            marker_coords.ptr<cv::Vec3f>(0)[1] = cv::Vec3f(markerLength/2.f, markerLength/2.f, 0);
            marker_coords.ptr<cv::Vec3f>(0)[2] = cv::Vec3f(markerLength/2.f, -markerLength/2.f, 0);
            marker_coords.ptr<cv::Vec3f>(0)[3] = cv::Vec3f(-markerLength/2.f, -markerLength/2.f, 0);
            id_length_map[id] = marker_coords;
            length_map[id] = markerLength;
        }

        return true;
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& msg)
    {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv::Mat image = cv_ptr->image;
        arucoMarkersDetection(image);
    }

    void arucoMarkersDetection(cv::Mat& img)
    {
        std::vector<int> marker_ids;
        std::vector<std::vector<cv::Point2f>> marker_corners;
        // 左上，右上，右下和左下依次记录marker_corners
        cv::aruco::detectMarkers(img, dictionary, marker_corners, marker_ids, detectorParams);

        size_t nMarkers = marker_corners.size();
        std::vector<cv::Vec3d> rvecs(nMarkers), tvecs(nMarkers);
        if(!marker_ids.empty())
        {
            for(size_t i = 0; i < nMarkers; i++)
            {
                // safety check
                if(marker_corners.at(i).size() != 4) continue;

                solvePnP(id_length_map[marker_ids[i]], marker_corners.at(i), camMatrix, distCoeffs, rvecs.at(i), tvecs.at(i), false, cv::SOLVEPNP_EPNP);
                cv::aruco::drawAxis(img, camMatrix, distCoeffs, rvecs.at(i), tvecs.at(i), 0.1);
                std::cout << "id: " << marker_ids[i] << ", trans:" << tvecs.at(i)[0] << ", " << tvecs.at(i)[1] << ", " << tvecs.at(i)[2] << std::endl;
                cv::Mat R;
                cv::Rodrigues(rvecs.at(i), R);
                std::cout << "rot: " << R.at<float>(0, 0) << ", " << R.at<float>(0, 1) << ", " << R.at<float>(0, 2) << std::endl;
            }
            if(cnt == 0)
            {
                cv::imwrite("/home/vulcan/ground_aerial_simulation/aruco_detection.png", img);
            }
            cnt++;

        }

    }

};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "aruco_detection");

    arucoDet aruco_detector;

    // // set coordinate system
    // cv::Mat objPoints(4, 1, CV_32FC3);
    // objPoints.ptr<Vec3f>(0)[0] = Vec3f(-markerLength/2.f, markerLength/2.f, 0);
    // objPoints.ptr<Vec3f>(0)[1] = Vec3f(markerLength/2.f, markerLength/2.f, 0);
    // objPoints.ptr<Vec3f>(0)[2] = Vec3f(markerLength/2.f, -markerLength/2.f, 0);
    // objPoints.ptr<Vec3f>(0)[3] = Vec3f(-markerLength/2.f, -markerLength/2.f, 0);

    // while(inputVideo.grab()) {
    //     cv::Mat image, imageCopy;
    //     inputVideo.retrieve(image);

    //     double tick = (double)getTickCount();

    //     vector<int> ids;
    //     vector<vector<Point2f> > corners, rejected;

    //     // detect markers and estimate pose
    //     detector.detectMarkers(image, corners, ids, rejected);

    //     size_t nMarkers = corners.size();
    //     vector<Vec3d> rvecs(nMarkers), tvecs(nMarkers);

    //     if(estimatePose && !ids.empty()) {
    //         // Calculate pose for each marker
    //         for (size_t i = 0; i < nMarkers; i++) {
    //             solvePnP(objPoints, corners.at(i), camMatrix, distCoeffs, rvecs.at(i), tvecs.at(i));
    //         }
    //     }
    //     double currentTime = ((double)getTickCount() - tick) / getTickFrequency();
    //     totalTime += currentTime;
    //     totalIterations++;
    //     if(totalIterations % 30 == 0) {
    //         cout << "Detection Time = " << currentTime * 1000 << " ms "
    //              << "(Mean = " << 1000 * totalTime / double(totalIterations) << " ms)" << endl;
    //     }
    //     // draw results
    //     image.copyTo(imageCopy);
    //     if(!ids.empty()) {
    //         cv::aruco::drawDetectedMarkers(imageCopy, corners, ids);

    //         if(estimatePose) {
    //             for(unsigned int i = 0; i < ids.size(); i++)
    //                 cv::drawFrameAxes(imageCopy, camMatrix, distCoeffs, rvecs[i], tvecs[i], markerLength * 1.5f, 2);
    //         }
    //     }

    //     if(showRejected && !rejected.empty())
    //         cv::aruco::drawDetectedMarkers(imageCopy, rejected, noArray(), Scalar(100, 0, 255));

    //     imshow("out", imageCopy);
    //     char key = (char)waitKey(waitTime);
    //     if(key == 27) break;
    // }

    ros::spin();
    
    return 0;
}
