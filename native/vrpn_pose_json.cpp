#include <chrono>
#include <csignal>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <unistd.h>
#include <vrpn_Tracker.h>

namespace {

volatile std::sig_atomic_t g_should_exit = 0;

struct Pose {
    double timestamp_sec = 0.0;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double qx = 0.0;
    double qy = 0.0;
    double qz = 0.0;
    double qw = 1.0;
    bool valid = false;
};

struct Options {
    std::string endpoint;
    std::string tracker;
    std::string host;
    std::string port;
    double z_offset = 0.0;
    bool invert_yaw = false;
};

void handle_signal(int) {
    g_should_exit = 1;
}

double yaw_from_quaternion(const Pose& pose, bool invert_yaw) {
    const double siny_cosp = 2.0 * (pose.qw * pose.qz + pose.qx * pose.qy);
    const double cosy_cosp = 1.0 - 2.0 * (pose.qy * pose.qy + pose.qz * pose.qz);
    const double yaw = std::atan2(siny_cosp, cosy_cosp);
    return invert_yaw ? -yaw : yaw;
}

void print_pose(const Options& options, const Pose& pose) {
    std::cout << std::fixed << std::setprecision(9)
              << "{\"source\":\"vrpn\","
              << "\"endpoint\":\"" << options.endpoint << "\","
              << "\"timestamp\":" << pose.timestamp_sec << ","
              << "\"x\":" << pose.x << ","
              << "\"y\":" << pose.y << ","
              << "\"z\":" << pose.z + options.z_offset << ","
              << "\"qx\":" << pose.qx << ","
              << "\"qy\":" << pose.qy << ","
              << "\"qz\":" << pose.qz << ","
              << "\"qw\":" << pose.qw << ","
              << "\"yaw\":" << yaw_from_quaternion(pose, options.invert_yaw)
              << ",\"z_offset\":" << options.z_offset
              << ",\"invert_yaw\":" << (options.invert_yaw ? "true" : "false")
              << "}" << std::endl;
}

void VRPN_CALLBACK handle_tracker(void* userdata, const vrpn_TRACKERCB info) {
    auto* pose = static_cast<Pose*>(userdata);
    pose->timestamp_sec = static_cast<double>(info.msg_time.tv_sec) +
                          static_cast<double>(info.msg_time.tv_usec) / 1000000.0;
    pose->x = info.pos[0];
    pose->y = info.pos[1];
    pose->z = info.pos[2];
    pose->qx = info.quat[0];
    pose->qy = info.quat[1];
    pose->qz = info.quat[2];
    pose->qw = info.quat[3];
    pose->valid = true;
}

Options options_from_args(int argc, char** argv) {
    Options options;
    if (const char* value = std::getenv("RMTT_VRPN_TRACKER")) {
        options.tracker = value;
    }
    if (const char* value = std::getenv("RMTT_VRPN_HOST")) {
        options.host = value;
    }
    if (const char* value = std::getenv("RMTT_VRPN_PORT")) {
        options.port = value;
    }
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if ((arg == "--endpoint" || arg == "-e") && i + 1 < argc) {
            options.endpoint = argv[++i];
        }
        else if (arg == "--tracker" && i + 1 < argc) {
            options.tracker = argv[++i];
        } else if (arg == "--host" && i + 1 < argc) {
            options.host = argv[++i];
        } else if (arg == "--port" && i + 1 < argc) {
            options.port = argv[++i];
        } else if (arg == "--z-offset" && i + 1 < argc) {
            options.z_offset = std::atof(argv[++i]);
        } else if (arg == "--invert-yaw") {
            options.invert_yaw = true;
        } else if (arg == "--help" || arg == "-h") {
            std::cout << "Usage: vrpn_pose_json [--endpoint tracker@host:port] [--z-offset meters] [--invert-yaw]\n"
                      << "       vrpn_pose_json [--tracker name --host host --port port] [--z-offset meters] [--invert-yaw]\n";
            std::exit(0);
        }
    }
    if (options.endpoint.empty()) {
        if (options.tracker.empty() || options.host.empty() || options.port.empty()) {
            std::cerr << "VRPN endpoint is required. Pass --endpoint or set RMTT_VRPN_TRACKER, RMTT_VRPN_HOST, and RMTT_VRPN_PORT.\n";
            std::exit(2);
        }
        options.endpoint = options.tracker + "@" + options.host + ":" + options.port;
    }
    return options;
}

}  // namespace

int main(int argc, char** argv) {
    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);

    const Options options = options_from_args(argc, argv);
    Pose latest;
    std::uint64_t seen = 0;
    std::uint64_t printed = 0;

    auto tracker = std::make_unique<vrpn_Tracker_Remote>(options.endpoint.c_str());
    tracker->shutup = true;
    tracker->register_change_handler(&latest, &handle_tracker);

    while (!g_should_exit) {
        tracker->mainloop();
        if (latest.valid) {
            ++seen;
            if (seen != printed) {
                print_pose(options, latest);
                printed = seen;
            }
            latest.valid = false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    tracker->unregister_change_handler(&latest, &handle_tracker);
    return 0;
}
