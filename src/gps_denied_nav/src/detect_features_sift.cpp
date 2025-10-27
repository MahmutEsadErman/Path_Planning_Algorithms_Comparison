#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <iostream>
#include <string>

int main(int argc, char* argv[])
{
  // Video file path
  std::string video_path = "output_video.mp4";
  
  if (argc > 1) {
    video_path = argv[1];
  }
  
  // Open the video file
  cv::VideoCapture video(video_path);
  
  if (!video.isOpened()) {
    std::cerr << "Error: Could not open video file: " << video_path << std::endl;
    return -1;
  }
  
  // Get video properties
  int fps = static_cast<int>(video.get(cv::CAP_PROP_FPS));
  int width = static_cast<int>(video.get(cv::CAP_PROP_FRAME_WIDTH));
  int height = static_cast<int>(video.get(cv::CAP_PROP_FRAME_HEIGHT));
  int frame_count = static_cast<int>(video.get(cv::CAP_PROP_FRAME_COUNT));
  
  std::cout << "Video properties:" << std::endl;
  std::cout << "  Resolution: " << width << "x" << height << std::endl;
  std::cout << "  FPS: " << fps << std::endl;
  std::cout << "  Total frames: " << frame_count << std::endl;
  
  // Create SIFT detector
  int max_features = 1000;
  cv::Ptr<cv::SIFT> sift = cv::SIFT::create(max_features);
  
  // Create video writer for output
  int fourcc = cv::VideoWriter::fourcc('m', 'p', '4', 'v');
  cv::VideoWriter output_video("output_sift_features.mp4", fourcc, fps, cv::Size(width, height));
  
  if (!output_video.isOpened()) {
    std::cerr << "Error: Could not create output video writer" << std::endl;
    return -1;
  }
  
  cv::Mat frame, gray, frame_with_features;
  std::vector<cv::KeyPoint> keypoints;
  cv::Mat descriptors;
  int frame_num = 0;
  
  std::cout << "\nProcessing video..." << std::endl;
  
  while (true) {
    video >> frame;
    
    if (frame.empty()) {
      break;
    }
    
    // Convert to grayscale
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    
    // Detect SIFT features
    sift->detectAndCompute(gray, cv::noArray(), keypoints, descriptors);
    
    // Draw keypoints on the frame
    frame_with_features = frame.clone();
    cv::drawKeypoints(frame, keypoints, frame_with_features, 
                      cv::Scalar(0, 255, 0), 
                      cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    
    // Add text information
    std::string info_text = "Features detected: " + std::to_string(keypoints.size());
    cv::putText(frame_with_features, info_text, cv::Point(10, 30),
                cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);
    
    std::string frame_text = "Frame: " + std::to_string(frame_num) + "/" + std::to_string(frame_count);
    cv::putText(frame_with_features, frame_text, cv::Point(10, 70),
                cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);
    
    // Write frame to output video
    output_video.write(frame_with_features);
    
    // Display frame
    cv::imshow("SIFT Features", frame_with_features);
    
    if (frame_num % 30 == 0) {
      std::cout << "Processed frame: " << frame_num << "/" << frame_count 
                << " (Features: " << keypoints.size() << ")" << std::endl;
    }
    
    frame_num++;
    
    // Press 'q' to quit
    if (cv::waitKey(1) == 'q') {
      break;
    }
    
    keypoints.clear();
  }
  
  std::cout << "\nProcessing complete!" << std::endl;
  std::cout << "Output video saved as: output_sift_features.mp4" << std::endl;
  
  // Release resources
  video.release();
  output_video.release();
  cv::destroyAllWindows();
  
  return 0;
}