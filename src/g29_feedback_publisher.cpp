#include <rclcpp/rclcpp.hpp>

#include <linux/input.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <fcntl.h>
#include <math.h>

#include "ros_g29_force_feedback/msg/force_feedback.hpp"
#include "ros_g29_force_feedback/msg/wheel_data.hpp"

class G29ForceFeedback : public rclcpp::Node {

private:
    rclcpp::Subscription<ros_g29_force_feedback::msg::ForceFeedback>::SharedPtr sub_target;
    
    rclcpp::TimerBase::SharedPtr timer;

    rclcpp::Publisher<ros_g29_force_feedback::msg::WheelData>::SharedPtr pub_data;
    rclcpp::TimerBase::SharedPtr timer_pub;

    // device info
    int m_device_handle;
    static constexpr int m_axis_code = ABS_X;
    static constexpr int m_accel_axis = ABS_Z;
    static constexpr int m_brake_axis = ABS_RZ;
    int m_axis_min, m_axis_max;
    int m_accel_min, m_accel_max;
    int m_brake_min, m_brake_max;

    // rosparam
    std::string m_device_name;
    double m_loop_period;
    double m_pub_period;
    double m_max_torque;
    double m_min_torque;
    double m_brake_position;
    double m_brake_torque;
    double m_auto_centering_max_torque;
    double m_auto_centering_max_position;
    double m_eps;
    bool m_auto_centering;

    // variables
    ros_g29_force_feedback::msg::ForceFeedback m_target;
    bool m_is_target_updated = false;
    bool m_is_brake_range = false;
    struct ff_effect m_effect;
    double m_position;
    double m_torque;
    double m_attack_length;
    double m_accel_pos, m_brake_pos;

public:
    G29ForceFeedback();
    ~G29ForceFeedback();

private:
    void targetCallback(const ros_g29_force_feedback::msg::ForceFeedback::SharedPtr in_target);
    void loop();
    int testBit(int bit, unsigned char *array);
    void initDevice();
    void calcRotateForce(double &torque, double &attack_length, const ros_g29_force_feedback::msg::ForceFeedback &target, const double &current_position);
    void calcCenteringForce(double &torque, const ros_g29_force_feedback::msg::ForceFeedback &target, const double &current_position);
    void uploadForce(const double &position, const double &force, const double &attack_length);
    void publish();
};


G29ForceFeedback::G29ForceFeedback() 
    : Node("g29_force_feedback"){
        
    sub_target = this->create_subscription<ros_g29_force_feedback::msg::ForceFeedback>(
        "/ff_target", 
        rclcpp::SystemDefaultsQoS(), 
        std::bind(&G29ForceFeedback::targetCallback, this, std::placeholders::_1));

    pub_data = this->create_publisher<ros_g29_force_feedback::msg::WheelData>("/wheel_data", 10);
    
    declare_parameter("device_name", m_device_name);
    declare_parameter("loop_period", m_loop_period);
    declare_parameter("pub_period", m_pub_period);
    declare_parameter("max_torque", m_max_torque);
    declare_parameter("min_torque", m_min_torque);
    declare_parameter("brake_position", m_brake_position);
    declare_parameter("brake_torque", m_brake_torque);
    declare_parameter("auto_centering_max_torque", m_auto_centering_max_torque);
    declare_parameter("auto_centering_max_position", m_auto_centering_max_position);
    declare_parameter("eps", m_eps);
    declare_parameter("auto_centering", m_auto_centering);

    get_parameter("device_name", m_device_name);
    get_parameter("loop_period", m_loop_period);
    get_parameter("pub_period", m_pub_period);
    get_parameter("max_torque", m_max_torque);
    get_parameter("min_torque", m_min_torque);
    get_parameter("brake_position", m_brake_position);
    get_parameter("brake_torque", m_brake_torque);
    get_parameter("auto_centering_max_torque", m_auto_centering_max_torque);
    get_parameter("auto_centering_max_position", m_auto_centering_max_position);
    get_parameter("eps", m_eps);
    get_parameter("auto_centering", m_auto_centering);

    initDevice();

    rclcpp::sleep_for(std::chrono::seconds(1));

    timer = this->create_wall_timer(std::chrono::milliseconds((int)(m_loop_period*1000)), 
            std::bind(&G29ForceFeedback::loop,this));

    timer_pub = this->create_wall_timer(std::chrono::milliseconds((int)(m_pub_period*1000)), 
            std::bind(&G29ForceFeedback::publish,this));
}

G29ForceFeedback::~G29ForceFeedback() {

    m_effect.type = FF_CONSTANT;
    m_effect.id = -1;
    m_effect.u.constant.level = 0;
    m_effect.direction = 0;
    // upload m_effect
    if (ioctl(m_device_handle, EVIOCSFF, &m_effect) < 0) {
        std::cout << "failed to upload m_effect" << std::endl;
    }
}


// update input event with timer callback
void G29ForceFeedback::loop() {

    struct input_event event;
    double last_position = m_position;
    // get current state
    while (read(m_device_handle, &event, sizeof(event)) == sizeof(event)) {
        if (event.type == EV_ABS) {
            switch (event.code) {
                case m_axis_code:
                    m_position = (event.value - (m_axis_max + m_axis_min) * 0.5) * 2 / (m_axis_max - m_axis_min);
                    break;
                case m_accel_axis:
                    m_accel_pos = (event.value - (double) m_accel_min) / (m_accel_max - m_accel_min);
                    break;
                case m_brake_axis:
                    m_brake_pos = (event.value - (double) m_brake_min) / (m_brake_max - m_brake_min);
                    break;
                default:
                    break;
            }
        }
    }

    if (m_is_brake_range || m_auto_centering) {
        calcCenteringForce(m_torque, m_target, m_position);
        m_attack_length = 0.0;

    } else {
        calcRotateForce(m_torque, m_attack_length, m_target, m_position);
        m_is_target_updated = false;
    }

    uploadForce(m_target.position, m_torque, m_attack_length);
}

// publish data
void G29ForceFeedback::publish() {
    auto msg = ros_g29_force_feedback::msg::WheelData();

    msg.header.stamp = this->get_clock()->now();
    msg.wheel_position = m_position;
    msg.accel_position = m_accel_pos;
    msg.brake_position = m_brake_pos;

    pub_data->publish(msg);
}


void G29ForceFeedback::calcRotateForce(double &torque,
                                       double &attack_length,
                                       const ros_g29_force_feedback::msg::ForceFeedback &target,
                                       const double &current_position) {

    double diff = target.position - current_position;
    double direction = (diff > 0.0) ? 1.0 : -1.0;

    if (fabs(diff) < m_eps) {
        torque = 0.0;
        attack_length = 0.0;

    } else if (fabs(diff) < m_brake_position) {
        m_is_brake_range = true;
        torque = target.torque * m_brake_torque * -direction;
        attack_length = m_loop_period;

    } else {
        torque = target.torque * direction;
        attack_length = m_loop_period;
    }
}


void G29ForceFeedback::calcCenteringForce(double &torque,
                                          const ros_g29_force_feedback::msg::ForceFeedback &target,
                                          const double &current_position) {

    double diff = target.position - current_position;
    double direction = (diff > 0.0) ? 1.0 : -1.0;

    if (fabs(diff) < m_eps)
        torque = 0.0;

    else {
        double torque_range = m_auto_centering_max_torque - m_min_torque;
        double power = (fabs(diff) - m_eps) / (m_auto_centering_max_position - m_eps);
        double buf_torque = power * torque_range + m_min_torque;
        torque = std::min(buf_torque, m_auto_centering_max_torque) * direction;
    }
}


// update input event with writing information to the event file
void G29ForceFeedback::uploadForce(const double &position,
                                   const double &torque,
                                   const double &attack_length) {

    // std::cout << torque << std::endl;
    // set effect
    m_effect.u.constant.level = 0x7fff * std::min(torque, m_max_torque);
    m_effect.direction = 0xC000;
    m_effect.u.constant.envelope.attack_level = 0; /* 0x7fff * force / 2 */
    m_effect.u.constant.envelope.attack_length = attack_length;
    m_effect.u.constant.envelope.fade_level = 0;
    m_effect.u.constant.envelope.fade_length = attack_length;

    // upload effect
    if (ioctl(m_device_handle, EVIOCSFF, &m_effect) < 0) {
        std::cout << "failed to upload effect" << std::endl;
    }
}


// get target information of wheel control from ros message
void G29ForceFeedback::targetCallback(const ros_g29_force_feedback::msg::ForceFeedback::SharedPtr in_msg) {

    if (m_target.position == in_msg->position && m_target.torque == fabs(in_msg->torque)) {
        m_is_target_updated = false;

    } else {
        m_target = *in_msg;
        m_target.torque = fabs(m_target.torque);
        m_is_target_updated = true;
        m_is_brake_range = false;
    }
}


// initialize force feedback device
void G29ForceFeedback::initDevice() {
    // setup device
    unsigned char key_bits[1+KEY_MAX/8/sizeof(unsigned char)];
    unsigned char abs_bits[1+ABS_MAX/8/sizeof(unsigned char)];
    unsigned char ff_bits[1+FF_MAX/8/sizeof(unsigned char)];
    struct input_event event;
    struct input_absinfo abs_info;

    m_device_handle = open(m_device_name.c_str(), O_RDWR|O_NONBLOCK);
    if (m_device_handle < 0) {
        std::cout << "ERROR: cannot open device : "<< m_device_name << std::endl;
        exit(1);

    } else {std::cout << "device opened" << std::endl;}

    // which axes has the device?
    memset(abs_bits, 0, sizeof(abs_bits));
    if (ioctl(m_device_handle, EVIOCGBIT(EV_ABS, sizeof(abs_bits)), abs_bits) < 0) {
        std::cout << "ERROR: cannot get abs bits" << std::endl;
        exit(1);
    }

    // get some information about force feedback
    memset(ff_bits, 0, sizeof(ff_bits));
    if (ioctl(m_device_handle, EVIOCGBIT(EV_FF, sizeof(ff_bits)), ff_bits) < 0) {
        std::cout << "ERROR: cannot get ff bits" << std::endl;
        exit(1);
    }

    // get axis value range
    if (ioctl(m_device_handle, EVIOCGABS(m_axis_code), &abs_info) < 0) {
        std::cout << "ERROR: cannot get wheel axis range" << std::endl;
        exit(1);
    }
    m_axis_max = abs_info.maximum;
    m_axis_min = abs_info.minimum;
    if (m_axis_min >= m_axis_max) {
        std::cout << "ERROR: axis range has bad value" << std::endl;
        exit(1);
    }


    // Get accelerator axis info
    if(ioctl(m_device_handle, EVIOCGABS(m_accel_axis), &abs_info) < 0) {
        std::cout << "ERROR: cannot get accel xis range" << std::endl;
        exit(1);
    }
    m_accel_min = abs_info.minimum;
    m_accel_max = abs_info.maximum;

    // Get brake axis info
    if(ioctl(m_device_handle, EVIOCGABS(m_brake_axis), &abs_info) < 0) {
        std::cout << "ERROR: cannot get brake axis range" << std::endl;
        exit(1);
    }
    m_brake_min = abs_info.minimum;
    m_brake_max = abs_info.maximum;

    // check force feedback is supported?
    if(!testBit(FF_CONSTANT, ff_bits)) {
        std::cout << "ERROR: force feedback is not supported" << std::endl;
        exit(1);

    } else { std::cout << "force feedback supported" << std::endl; }

    // auto centering off
    memset(&event, 0, sizeof(event));
    event.type = EV_FF;
    event.code = FF_AUTOCENTER;
    event.value = 0;
    if (write(m_device_handle, &event, sizeof(event)) != sizeof(event)) {
        std::cout << "failed to disable auto centering" << std::endl;
        exit(1);
    }

    // init effect and get effect id
    memset(&m_effect, 0, sizeof(m_effect));
    m_effect.type = FF_CONSTANT;
    m_effect.id = -1; // initial value
    m_effect.trigger.button = 0;
    m_effect.trigger.interval = 0;
    m_effect.replay.length = 0xffff;  // longest value
    m_effect.replay.delay = 0; // delay from write(...)
    m_effect.u.constant.level = 0;
    m_effect.direction = 0xC000;
    m_effect.u.constant.envelope.attack_length = 0;
    m_effect.u.constant.envelope.attack_level = 0;
    m_effect.u.constant.envelope.fade_length = 0;
    m_effect.u.constant.envelope.fade_level = 0;

    if (ioctl(m_device_handle, EVIOCSFF, &m_effect) < 0) {
        std::cout << "failed to upload m_effect" << std::endl;
        exit(1);
    }

    // start m_effect
    memset(&event, 0, sizeof(event));
    event.type = EV_FF;
    event.code = m_effect.id;
    event.value = 1;
    if (write(m_device_handle, &event, sizeof(event)) != sizeof(event)) {
        std::cout << "failed to start event" << std::endl;
        exit(1);
    }
}


// util for initDevice()
int G29ForceFeedback::testBit(int bit, unsigned char *array) {

    return ((array[bit / (sizeof(unsigned char) * 8)] >> (bit % (sizeof(unsigned char) * 8))) & 1);
}


int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);

    auto g29_ff = std::make_shared<G29ForceFeedback>();
    rclcpp::spin(g29_ff);

    rclcpp::shutdown();
    return 0;
}
