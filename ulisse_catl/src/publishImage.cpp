#include <publishImage.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class ImagePublisher : public rclcpp::Node
{
  public:
    ImagePublisher()
    : Node("mqtt_publisher")
    {
      //publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10); // TODO REMOVE
      timer_ = this->create_wall_timer(100ms, std::bind(&ImagePublisher::TimedSomething, this));
      subscriber_ = this->create_subscription<sensor_msgs::msg::CompressedImage>("/center_camera/image_color/compressed", 10, std::bind(&ImagePublisher::CameraCallback, this, _1));
      subscriberUncompressed_ = this->create_subscription<sensor_msgs::msg::Image>("/ladybug/camera1/image_raw", 10, std::bind(&ImagePublisher::CameraCallbackUncompressed, this, _1));
      imagePublisherMQTT_ = nullptr;//std::make_shared<pahho::MQTTPublisher>("ulisseStatusPub", "catl/unige/ulisse/status",  "127.0.0.1", 1883); // TODO CHECK ARGUMENTS
      count_ = 0;
    }

  private:

    void EncodeImage(cv::Mat &img, stng::Stanag4609Conversion &conv, const std::string testFolder, const std::string testName) const {
        auto unixTime = std::time(nullptr);
        
        stng::ImageParams imgParams;
        imgParams.outputCodecId = AV_CODEC_ID_H264;
        imgParams.bitRate = 500000; // 1000
        imgParams.gopSize = 12; // 10
        cv::resize(img, img, cv::Size(1280, 720), cv::INTER_LINEAR);
        //cv::transpose(img, img);
        //cv::flip(img, img, 1);
        std::cerr << tc::bluL << "[EncodeImage] Starting to encode!" << tc::none << std::endl;
        cv::imshow("Loaded Image", img);
        cv::waitKey(1);
        if (conv.ReadFramesFromSource(img, imgParams)) {
            if (conv.EncodeFrames("h264_nvenc", testFolder + "_" + testName + "_encoded.ts", -1, -1)) {
                conv.Mux("mpegts", testFolder + "_" + testName + "_muxed_" + std::to_string(unixTime) + "_" + std::to_string(count_) + ".ts", 4500); // TODO DISABLE
                //conv.Mux("mpegts", "udp://127.0.0.1:9000", 4500); // TODO RESTORE
                std::cerr << tc::greenL << "[EncodeImage] Img encoded!" << tc::greenL << std::endl;
            }
        }
    }

    void CameraCallbackUncompressed(const sensor_msgs::msg::Image::SharedPtr msg) {
      std::cerr << tc::bluL << "[FrameCallback] START" << tc::none << std::endl;
      count_++;
      cv_bridge::CvImagePtr cv_ptr;
      try {
          cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
          auto img = cv_ptr->image;
          if (imagePublisherMQTT_ != nullptr) {
              auto stamp = msg->header.stamp;
              auto tCurr = stamp.nanosec * pow(10,-9) + stamp.sec;
              std::cerr << tc::bluL << "[FrameCallback] Publishing - START" << tc::none << std::endl;
              imagePublisherMQTT_->PublishImage(img, tCurr);
              std::cerr << tc::bluL << "[FrameCallback] Publishing - END" << tc::none << std::endl;
          }
          else {
              stng::Stanag4609Conversion conv;
              std::cerr << tc::bluL << "[FrameCallback] Encoding - START" << tc::none << std::endl;
              EncodeImage(img, conv, OUTPUT_ENCODING_PATH, "test");
              std::cerr << tc::bluL << "[FrameCallback] Encoding - END" << tc::none << std::endl;
          }
      } catch (cv_bridge::Exception& e) {
          std::cerr << "[UncompressedFrameCallback] cv_bridge exception: " << e.what() << std::endl;
          return;
      }
      std::cerr << tc::bluL << "[FrameCallback] END" << tc::none << std::endl;
    }

    void CameraCallback(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
      std::cerr << tc::bluL << "[FrameCallback] START" << tc::none << std::endl;
      count_++;
      cv_bridge::CvImagePtr cv_ptr;
      try {
          cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
          auto img = cv_ptr->image;
          if (imagePublisherMQTT_ != nullptr) {
              auto stamp = msg->header.stamp;
              auto tCurr = stamp.nanosec * pow(10,-9) + stamp.sec;
              std::cerr << tc::bluL << "[FrameCallback] Publishing - START" << tc::none << std::endl;
              imagePublisherMQTT_->PublishImage(img, tCurr);
              std::cerr << tc::bluL << "[FrameCallback] Publishing - END" << tc::none << std::endl;
          }
          else {
              stng::Stanag4609Conversion conv;
              std::cerr << tc::bluL << "[FrameCallback] Encoding - START" << tc::none << std::endl;
              EncodeImage(img, conv, OUTPUT_ENCODING_PATH, "test");
              std::cerr << tc::bluL << "[FrameCallback] Encoding - END" << tc::none << std::endl;
          }
      } catch (cv_bridge::Exception& e) {
          std::cerr << "[UncompressedFrameCallback] cv_bridge exception: " << e.what() << std::endl;
          return;
      }
      std::cerr << tc::bluL << "[FrameCallback] END" << tc::none << std::endl;
    }

    void TimedSomething() const {}
    
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriberUncompressed_;
    std::shared_ptr<pahho::MQTTPublisher> imagePublisherMQTT_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImagePublisher>());
  rclcpp::shutdown();
  return 0;
}
