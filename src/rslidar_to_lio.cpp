#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <cmath>
#include <cstring>
#include <vector>

static constexpr float ELEV_MIN_DEG = -7.0f;
static constexpr float ELEV_MAX_DEG =  52.0f;
static constexpr float ELEV_RANGE   = ELEV_MAX_DEG - ELEV_MIN_DEG;
static constexpr float DEG_PER_RAD  = 180.0f / M_PI;
static constexpr float SCAN_PERIOD  = 0.1f;

class RslidarToLio : public rclcpp::Node
{
public:
    RslidarToLio() : Node("rslidar_to_lio")
    {
        this->declare_parameter<int>("n_scan", 96);
        n_scan_ = this->get_parameter("n_scan").as_int();

        // PointXYZIRCAEDT output fields (32 bytes)
        auto mf = [](const std::string & name, uint32_t offset, uint8_t dtype) {
            sensor_msgs::msg::PointField f;
            f.name = name; f.offset = offset; f.datatype = dtype; f.count = 1;
            return f;
        };
        fields_xyzirc_ = {
            mf("x",           0,  sensor_msgs::msg::PointField::FLOAT32),
            mf("y",           4,  sensor_msgs::msg::PointField::FLOAT32),
            mf("z",           8,  sensor_msgs::msg::PointField::FLOAT32),
            mf("intensity",   12, sensor_msgs::msg::PointField::UINT8),
            mf("return_type", 13, sensor_msgs::msg::PointField::UINT8),
            mf("channel",     14, sensor_msgs::msg::PointField::UINT16),
            mf("azimuth",     16, sensor_msgs::msg::PointField::FLOAT32),
            mf("elevation",   20, sensor_msgs::msg::PointField::FLOAT32),
            mf("distance",    24, sensor_msgs::msg::PointField::FLOAT32),
            mf("time_stamp",  28, sensor_msgs::msg::PointField::UINT32),
        };

        // PointXYZIRT output fields (24 bytes) — for /points_raw used by lidarslam
        fields_xyzi_ = {
            mf("x",         0,  sensor_msgs::msg::PointField::FLOAT32),
            mf("y",         4,  sensor_msgs::msg::PointField::FLOAT32),
            mf("z",         8,  sensor_msgs::msg::PointField::FLOAT32),
            mf("intensity", 12, sensor_msgs::msg::PointField::FLOAT32),
            mf("ring",      16, sensor_msgs::msg::PointField::UINT16),
            mf("time",      20, sensor_msgs::msg::PointField::FLOAT32),
        };

        buf_xyzirc_.reserve(200000 * 32);
        buf_xyzi_.reserve(200000 * 24);

        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/points", rclcpp::QoS(rclcpp::KeepLast(5)).reliable(),
            std::bind(&RslidarToLio::callback, this, std::placeholders::_1));

        // For lidarslam mapping
        pub_raw_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/points_raw", rclcpp::SensorDataQoS());

        // For Autoware NDT — RELIABLE QoS
        auto reliable_qos = rclcpp::QoS(rclcpp::KeepLast(5)).reliable();
        pub_concat_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/sensing/lidar/concatenated/pointcloud", reliable_qos);

        RCLCPP_INFO(this->get_logger(),
            "rslidar_to_lio started | n_scan=%d | publishes /points_raw + /sensing/lidar/concatenated/pointcloud",
            n_scan_);
    }

private:
    void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        const uint32_t N    = msg->width * msg->height;
        const uint32_t ps   = msg->point_step;
        const uint8_t * src = msg->data.data();

        uint32_t off_x = 0, off_y = 4, off_z = 8, off_i = 12;
        for (const auto & f : msg->fields) {
            if      (f.name == "x")         off_x = f.offset;
            else if (f.name == "y")         off_y = f.offset;
            else if (f.name == "z")         off_z = f.offset;
            else if (f.name == "intensity") off_i = f.offset;
        }

        buf_xyzirc_.resize(N * 32);
        buf_xyzi_.resize(N * 24);
        uint8_t * dst_rc = buf_xyzirc_.data();
        uint8_t * dst_i  = buf_xyzi_.data();

        uint32_t valid = 0;
        const float inv_total = SCAN_PERIOD / static_cast<float>(N);

        for (uint32_t i = 0; i < N; ++i, src += ps)
        {
            float x, y, z, intensity;
            std::memcpy(&x,         src + off_x, 4);
            std::memcpy(&y,         src + off_y, 4);
            std::memcpy(&z,         src + off_z, 4);
            std::memcpy(&intensity, src + off_i, 4);

            if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
                continue;

            const float dist_xy  = std::sqrt(x*x + y*y);
            const float elev_deg = (dist_xy > 1e-4f)
                ? std::atan2(z, dist_xy) * DEG_PER_RAD : 0.0f;
            const float clamped  = std::max(ELEV_MIN_DEG, std::min(ELEV_MAX_DEG, elev_deg));
            const uint16_t ring  = static_cast<uint16_t>(
                std::min((int)((clamped - ELEV_MIN_DEG) / ELEV_RANGE * n_scan_), n_scan_ - 1));
            const float t_off    = static_cast<float>(i) * inv_total;

            // ── /points_raw (PointXYZIRT, 24 bytes) ──
            uint8_t * pr = dst_i + valid * 24;
            std::memcpy(pr + 0,  &x,         4);
            std::memcpy(pr + 4,  &y,         4);
            std::memcpy(pr + 8,  &z,         4);
            std::memcpy(pr + 12, &intensity,  4);
            std::memcpy(pr + 16, &ring,       2);
            pr[18] = 0; pr[19] = 0;
            std::memcpy(pr + 20, &t_off,      4);

            // ── /sensing/lidar/concatenated/pointcloud (PointXYZIRCAEDT, 32 bytes) ──
            const float dist      = std::sqrt(x*x + y*y + z*z);
            const float azimuth   = std::atan2(y, x);
            const float elevation = std::atan2(z, std::max(dist_xy, 1e-6f));
            const uint8_t  intensity_u8  = static_cast<uint8_t>(std::min(intensity, 255.0f));
            const uint8_t  return_type   = 0;
            const uint32_t time_stamp    = static_cast<uint32_t>(
                static_cast<uint64_t>(std::abs(t_off) * 1e9f) & 0xFFFFFFFF);

            uint8_t * pc = dst_rc + valid * 32;
            std::memcpy(pc + 0,  &x,            4);
            std::memcpy(pc + 4,  &y,            4);
            std::memcpy(pc + 8,  &z,            4);
            pc[12] = intensity_u8;
            pc[13] = return_type;
            std::memcpy(pc + 14, &ring,          2);
            std::memcpy(pc + 16, &azimuth,       4);
            std::memcpy(pc + 20, &elevation,     4);
            std::memcpy(pc + 24, &dist,          4);
            std::memcpy(pc + 28, &time_stamp,    4);

            ++valid;
        }

        if (valid == 0) return;

        auto now = this->get_clock()->now();

        // Publish /points_raw (lidar_link frame, original timestamp)
        sensor_msgs::msg::PointCloud2 out_raw;
        //out_raw.header      = msg->header;
        out_raw.header.stamp= now; 
        out_raw.header.frame_id = "lidar_link";
        out_raw.height      = 1;
        out_raw.width       = valid;
        out_raw.fields      = fields_xyzi_;
        out_raw.is_bigendian = false;
        out_raw.is_dense    = true;
        out_raw.point_step  = 24;
        out_raw.row_step    = 24 * valid;
        out_raw.data.assign(buf_xyzi_.begin(), buf_xyzi_.begin() + valid * 24);
        pub_raw_->publish(out_raw);

        // Publish /sensing/lidar/concatenated/pointcloud (lidar_link, ROS time)
        sensor_msgs::msg::PointCloud2 out_concat;
        out_concat.header.frame_id = "lidar_link";
        out_concat.header.stamp    = now;
        out_concat.height      = 1;
        out_concat.width       = valid;
        out_concat.fields      = fields_xyzirc_;
        out_concat.is_bigendian = false;
        out_concat.is_dense    = true;
        out_concat.point_step  = 32;
        out_concat.row_step    = 32 * valid;
        out_concat.data.assign(buf_xyzirc_.begin(), buf_xyzirc_.begin() + valid * 32);
        pub_concat_->publish(out_concat);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr    pub_raw_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr    pub_concat_;
    int n_scan_;
    std::vector<sensor_msgs::msg::PointField> fields_xyzirc_;
    std::vector<sensor_msgs::msg::PointField> fields_xyzi_;
    std::vector<uint8_t> buf_xyzirc_;
    std::vector<uint8_t> buf_xyzi_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RslidarToLio>());
    rclcpp::shutdown();
    return 0;
}