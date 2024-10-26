#include <fmt/core.h>
#include <fmt/os.h>

#include <array>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <memory>
#include <vector>

#include "sl_lidar.h"
#include "sl_lidar_driver.h"

using fmt::print;
using fmt::println;

using namespace sl;

static bool checkSLAMTECLIDARHealth(const std::unique_ptr<ILidarDriver> & drv) {
    sl_result op_result;
    sl_lidar_response_device_health_t healthinfo;

    op_result = drv->getHealth(healthinfo);
    if (SL_IS_OK(op_result)) { // the macro IS_OK is the preperred way to judge whether the operation is succeed.
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

/*
Scan modes:
0 Standard    answer type: 81 us/sample: 508.000000 max dist: 12.000000 m
1 Express     answer type: 82 us/sample: 254.000000 max dist: 12.000000 m
2 Boost       answer type: 84 us/sample: 127.000000 max dist: 12.000000 m
3 Sensitivity answer type: 84 us/sample: 127.000000 max dist: 12.000000 m
4 Stability   answer type: 84 us/sample: 201.000000 max dist: 12.000000 m
*/

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

    drv->setMotorSpeed();

    std::ofstream of("data.bin", std::ios::out | std::ios::binary);

    println("Reading from lidar...");
    drv->startScan(false, true);

    for (;;) {
        std::array<sl_lidar_response_measurement_node_hq_t, 8192> nodes;
        size_t count = nodes.size();

        auto op_result = drv->grabScanDataHq(nodes.data(), count);

        if (SL_IS_FAIL(op_result)) {
            println(stderr, "Failed to read from lidar: {}", resultErrorString(op_result));
            drv->stop();
            return -1;
        }

        drv->ascendScanData(nodes.data(), count);
        of.write(reinterpret_cast<char *>(&count), sizeof(size_t));
        of.write(reinterpret_cast<char *>(nodes.data()), count * sizeof(sl_lidar_response_measurement_node_hq_t));
    }

    drv->stop();

    return 0;
}
