#include <drive_ros_image_recognition/speed_marking_detection.h>
#include <drive_ros_image_recognition/geometry_common.h>
#include <pluginlib/class_list_macros.h>

#if defined(DRAW_DEBUG)
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#endif

#include "opencv2/text.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

namespace drive_ros_image_recognition {

bool isRepetitive(const std::string& s)
{
    int count = 0;
    for (int i=0; i<(int)s.size(); i++)
    {
        if ((s[i] == 'i') ||
                (s[i] == 'l') ||
                (s[i] == 'I'))
            count++;
    }
    if (count > ((int)s.size()+1)/2)
    {
        return true;
    }
    return false;
}

void er_draw(std::vector<cv::Mat> &channels, std::vector<std::vector<cv::text::ERStat> > &regions, std::vector<cv::Vec2i> group, cv::Mat& segmentation)
{
    for (int r=0; r<(int)group.size(); r++)
    {
        cv::text::ERStat er = regions[group[r][0]][group[r][1]];
        if (er.parent != NULL) // deprecate the root region
        {
            int newMaskVal = 255;
            int flags = 4 + (newMaskVal << 8) + cv::FLOODFILL_FIXED_RANGE + cv::FLOODFILL_MASK_ONLY;
            cv::floodFill(channels[group[r][0]],segmentation,cv::Point(er.pixel%channels[group[r][0]].cols,er.pixel/channels[group[r][0]].cols),
                      cv::Scalar(255),0,cv::Scalar(er.level),cv::Scalar(0),flags);
        }
    }
}

SpeedMarkingDetection::SpeedMarkingDetection()
  : Detection(ros::NodeHandle(), ros::NodeHandle("~"), nullptr)
{
}

SpeedMarkingDetection::SpeedMarkingDetection(const ros::NodeHandle nh, const ros::NodeHandle pnh, image_transport::ImageTransport* it)
  : NM1_("trained_classifierNM1.xml"), NM2_("trained_classifierNM2.xml"), Detection(nh, pnh, it)
{
}

SpeedMarkingDetection::~SpeedMarkingDetection() {
}

bool SpeedMarkingDetection::find() {

  cv::Mat processed_image;
  cv::cvtColor(*current_image_,processed_image,cv::COLOR_GRAY2RGB);
  std::vector<cv::Mat> channels;
  channels.push_back(processed_image);
  ros::WallTime t0 = ros::WallTime::now();
  // Create ERFilter objects with the 1st and 2nd stage default classifiers
  cv::Ptr<cv::text::ERFilter> er_filter1 = cv::text::createERFilterNM1(cv::text::loadClassifierNM1(NM1_),8,0.00015f,0.13f,0.2f,true,0.1f);
  cv::Ptr<cv::text::ERFilter> er_filter2 = cv::text::createERFilterNM2(cv::text::loadClassifierNM2(NM2_),0.5);

  std::vector<std::vector<cv::text::ERStat> > regions(1);
  // Apply the default cascade classifier to each independent channel (could be done in parallel)
  er_filter1->run(*current_image_, regions[0]);
  er_filter2->run(*current_image_, regions[0]);
  ros::WallDuration calc_dur = ros::WallTime::now()-t0;
  ROS_INFO_STREAM("[speed marking detection] Region detection took "<<calc_dur.toSec());

  cv::Mat out_img_decomposition = cv::Mat::zeros(processed_image.rows+2, processed_image.cols+2, CV_8UC1);
  std::vector<cv::Vec2i> tmp_group;
  for (int i=0; i<(int)regions.size(); i++)
  {
      for (int j=0; j<(int)regions[i].size();j++)
      {
          tmp_group.push_back(cv::Vec2i(i,j));
      }
      cv::Mat tmp= cv::Mat::zeros(processed_image.rows+2, processed_image.cols+2, CV_8UC1);
      er_draw(channels, regions, tmp_group, tmp);
      if (i > 0)
          tmp = tmp / 2;
      out_img_decomposition = out_img_decomposition | tmp;
      tmp_group.clear();
  }

  // Detect character groups
  std::vector<std::vector<cv::Vec2i> > nm_region_groups;
  std::vector<cv::Rect> nm_boxes;
  cv::text::erGrouping(processed_image, channels, regions, nm_region_groups, nm_boxes, cv::text::ERGROUPING_ORIENTATION_HORIZ);

  /*Text Recognition (OCR)*/

  cv::Ptr<cv::text::OCRTesseract> ocr = cv::text::OCRTesseract::create();
  std::string output;

  cv::Mat out_img;
  cv::Mat out_img_detection;
  cv::Mat out_img_segmentation = cv::Mat::zeros(processed_image.rows+2, processed_image.cols+2, CV_8UC1);
  processed_image.copyTo(out_img);
  processed_image.copyTo(out_img_detection);
  float scale_img  = 600.f/processed_image.rows;
  float scale_font = (float)(2-scale_img)/1.4f;
  std::vector<std::string> words_detection;

  for (int i=0; i<(int)nm_boxes.size(); i++)
  {
    cv::rectangle(out_img_detection, nm_boxes[i].tl(), nm_boxes[i].br(), cv::Scalar(0,255,255), 3);

    cv::Mat group_img = cv::Mat::zeros(processed_image.rows+2, processed_image.cols+2, CV_8UC1);
    er_draw(channels, regions, nm_region_groups[i], group_img);
    cv::Mat group_segmentation;
    group_img.copyTo(group_segmentation);
    //image(nm_boxes[i]).copyTo(group_img);
    group_img(nm_boxes[i]).copyTo(group_img);
    copyMakeBorder(group_img,group_img,15,15,15,15,cv::BORDER_CONSTANT,cv::Scalar(0));

    std::vector<cv::Rect> boxes;
    std::vector<std::string> words;
    std::vector<float> confidences;
    ocr->run(group_img, output, &boxes, &words, &confidences, cv::text::OCR_LEVEL_WORD);

    output.erase(remove(output.begin(), output.end(), '\n'), output.end());
    ROS_INFO_STREAM("[speed marking detection] OCR output: "<<output);
    if (output.size() < 3) {
      ROS_WARN_STREAM("[speed marking detection] Less than 3 detections, skipping. Number of detections: "<<output.size());
      continue;
    }

    for (int j=0; j<(int)boxes.size(); j++)
    {
      boxes[j].x += nm_boxes[i].x-15;
      boxes[j].y += nm_boxes[i].y-15;

      ROS_INFO_STREAM("[speed marking detection] word = "<< words[j]<<" confidence = "<< confidences[j]);
      if ((words[j].size() < 2) || (confidences[j] < 51) ||
          ((words[j].size()==2) && (words[j][0] == words[j][1])) ||
          ((words[j].size()< 4) && (confidences[j] < 60)) ||
          isRepetitive(words[j]))
        continue;
      words_detection.push_back(words[j]);
      cv::rectangle(out_img, boxes[j].tl(), boxes[j].br(), cv::Scalar(255,0,255),3);
      cv::Size word_size = getTextSize(words[j], cv::FONT_HERSHEY_SIMPLEX, (double)scale_font, (int)(3*scale_font), NULL);
      cv::rectangle(out_img, boxes[j].tl()-cv::Point(3,word_size.height+3), boxes[j].tl()+cv::Point(word_size.width,0), cv::Scalar(255,0,255),-1);
      cv::putText(out_img, words[j], boxes[j].tl()-cv::Point(1,1), cv::FONT_HERSHEY_SIMPLEX, scale_font, cv::Scalar(255,255,255),(int)(3*scale_font));
      out_img_segmentation = out_img_segmentation | group_segmentation;
    }
  }


  //resize(out_img_detection,out_img_detection,Size(image.cols*scale_img,image.rows*scale_img),0,0,INTER_LINEAR_EXACT);
  //imshow("detection", out_img_detection);
  //imwrite("detection.jpg", out_img_detection);
  //resize(out_img,out_img,Size(image.cols*scale_img,image.rows*scale_img),0,0,INTER_LINEAR_EXACT);
#if defined(DRAW_DEBUG)
  cv::namedWindow("recognition", cv::WINDOW_NORMAL);
  cv::imshow("recognition", out_img);
  cv::waitKey(0);
#endif
  //imwrite("recognition.jpg", out_img);
  //imwrite("segmentation.jpg", out_img_segmentation);
  //imwrite("decomposition.jpg", out_img_decomposition);

  return 0;
}

bool SpeedMarkingDetection::init() {
  if (!pnh_.getParam("NM1", NM1_)) {
    ROS_WARN_STREAM("[speed marking detection] Unable to load 'NM1' parameter, using default: "<<NM1_);
  }

  if (!pnh_.getParam("NM2", NM2_)) {
    ROS_WARN_STREAM("[speed marking detection] Unable to load 'NM2' parameter, using default: "<<NM2_);
  }
  return Detection::init();
}

} // namespace drive_ros_image_recognition

PLUGINLIB_EXPORT_CLASS(drive_ros_image_recognition::SpeedMarkingDetection, nodelet::Nodelet)
