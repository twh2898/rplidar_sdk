#include <fmt/core.h>

#include <array>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <vector>

#include "sl_lidar.h"
#include "sl_lidar_driver.h"

using fmt::print;
using fmt::println;

using namespace sl;

static bool checkSLAMTECLIDARHealth(const std::unique_ptr<ILidarDriver> & drv) {
    sl_lidar_response_device_health_t healthinfo;
    sl_result op_result = drv->getHealth(healthinfo);

    if (SL_IS_OK(op_result)) {
        println("SLAMTEC Lidar health status : {}", healthinfo.status);
        if (healthinfo.status == SL_LIDAR_STATUS_ERROR) {
            println(stderr, "Error, slamtec lidar internal error detected. Please reboot the device to retry.");
            // enable the following code if you want slamtec lidar to be reboot by software
            // drv->reset();
            return false;
        }
        else {
            return true;
        }
    }
    else {
        println(stderr, "Error, cannot retrieve the lidar health code: {:x}", op_result);
        return false;
    }
}

static const char * resultErrorString(sl_result res) {
    switch (res) {
        case SL_RESULT_OK:
            return "No Error";
        case SL_RESULT_ALREADY_DONE:
            return "ALREADY_DONE";
        case SL_RESULT_INVALID_DATA:
            return "INVALID_DATA";
        case SL_RESULT_OPERATION_FAIL:
            return "OPERATION_FAIL";
        case SL_RESULT_OPERATION_TIMEOUT:
            return "OPERATION_TIMEOUT";
        case SL_RESULT_OPERATION_STOP:
            return "OPERATION_STOP";
        case SL_RESULT_OPERATION_NOT_SUPPORT:
            return "OPERATION_NOT_SUPPORT";
        case SL_RESULT_FORMAT_NOT_SUPPORT:
            return "FORMAT_NOT_SUPPORT";
        case SL_RESULT_INSUFFICIENT_MEMORY:
            return "INSUFFICIENT_MEMORY";
        default:
            return "Other Error";
    }
}

static void printMotorStats(const LidarMotorInfo & motor) {
    println("Motor specs:");
    print("Contorl Support: ");
    switch (motor.motorCtrlSupport) {
        case MotorCtrlSupportNone:
            println("None");
            break;
        case MotorCtrlSupportPwm:
            println("PWM");
            break;
        case MotorCtrlSupportRpm:
            println("RPM");
            break;
        default:
            println("Other {}", (uint32_t)motor.motorCtrlSupport);
            break;
    }
    println("Desired speed {} of [{}, {}]", motor.desired_speed, motor.min_speed, motor.max_speed);
}

static void printScanModes(const std::vector<LidarScanMode> & modes) {
    size_t max_name_len = 0;
    for (auto & mode : modes) {
        size_t name_len = strlen(mode.scan_mode);
        if (name_len > max_name_len)
            max_name_len = name_len;
    }

    println("Scan modes:");
    for (auto & mode : modes) {
        println("{} {:{}s} answer type: {:02x} us/sample: {:f} max dist: {:f} m",
                mode.id,
                mode.scan_mode,
                max_name_len,
                mode.ans_type,
                mode.us_per_sample,
                mode.max_distance);
    }
}

static std::unique_ptr<IChannel> makeChannel(const std::string & device, int baudrate) {
    auto _channel = createSerialPortChannel(device, baudrate);
    if (!_channel) {
        println(stderr, "Failed to create channel: {}", resultErrorString(_channel.err));
        exit(-1);
    }

    std::unique_ptr<IChannel> channel(*_channel);
    if (!channel) {
        println(stderr, "insufficent memory, exit");
        exit(-2);
    }

    return channel;
}

static std::unique_ptr<ILidarDriver> makeDriver(const std::unique_ptr<IChannel> & channel) {
    auto _drv = createLidarDriver();
    if (!_drv) {
        println(stderr, "Failed to create lidar driver: {}", resultErrorString(_drv.err));
        exit(-1);
    }

    std::unique_ptr<ILidarDriver> drv(*_drv);
    if (!drv) {
        println(stderr, "insufficent memory, exit");
        exit(-2);
    }

    auto res = drv->connect(channel.get());
    if (SL_IS_FAIL(res)) {
        println(stderr, "Failed to connect channel to driver: {}", resultErrorString(res));
        exit(-1);
    }

    return drv;
}

int main(int argc, const char * argv[]) {
    auto channel = makeChannel("/dev/ttyUSB0", 115200);
    auto drv = makeDriver(channel);

    sl_lidar_response_device_info_t devinfo;
    auto res = drv->getDeviceInfo(devinfo);
    if (SL_IS_FAIL(res)) {
        println(stderr, "Failed to get device info: {}", resultErrorString(res));
        return -1;
    }

    // print out the device serial number, firmware and hardware version number..
    print("SLAMTEC LIDAR S/N: ");
    for (int pos = 0; pos < 16; ++pos) {
        print("{:02x}", devinfo.serialnum[pos]);
    }

    print(
        "\n"
        "Firmware Ver: {}.{:02}\n"
        "Hardware Rev: {}\n",
        devinfo.firmware_version >> 8,
        devinfo.firmware_version & 0xFF,
        (int)devinfo.hardware_version);

    // check health...
    if (!checkSLAMTECLIDARHealth(drv)) {
        return -1;
    }

    LidarMotorInfo motor;
    if (SL_IS_FAIL(drv->getMotorInfo(motor))) {
        println(stderr, "Failed to get motor info");
        return -1;
    }

    printMotorStats(motor);

    std::vector<LidarScanMode> modes;
    if (SL_IS_FAIL(drv->getAllSupportedScanModes(modes))) {
        println(stderr, "Failed to get scan modes");
        return -1;
    }

    printScanModes(modes);

    drv->stop();

    return 0;
}
