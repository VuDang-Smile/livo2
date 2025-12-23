#include "LIVMapper.h"
#include <signal.h>
#include <memory>

// Global pointer to mapper for signal handler
std::shared_ptr<LIVMapper> g_mapper_ptr = nullptr;

void signalHandler(int signum) {
  // Write to both stdout and a debug file
  std::ofstream debug_file("/tmp/fastlivo_shutdown.log", std::ios::app);
  debug_file << "========================================\n";
  debug_file << "Signal " << signum << " received at " << time(nullptr) << "\n";
  debug_file << "g_mapper_ptr is " << (g_mapper_ptr ? "valid" : "null") << "\n";
  debug_file.flush();
  
  std::cout << "\n\033[1;33m========================================\033[0m" << std::endl;
  std::cout << "\033[1;33mInterrupt signal (" << signum << ") received!\033[0m" << std::endl;
  std::cout << "\033[1;33mSaving aggregated PCD files before exit...\033[0m" << std::endl;
  std::cout << "\033[1;33m========================================\033[0m" << std::endl;
  std::cout.flush();
  
  if (g_mapper_ptr) {
    debug_file << "Calling savePCD(true)...\n";
    debug_file.flush();
    g_mapper_ptr->savePCD(true);
    debug_file << "savePCD(true) completed\n";
    debug_file.flush();
  }
  
  debug_file << "Calling rclcpp::shutdown()\n";
  debug_file.close();
  
  rclcpp::shutdown();
  exit(signum);
}

int main(int argc, char **argv)
{
  // Register signal handler for graceful shutdown
  signal(SIGINT, signalHandler);
  signal(SIGTERM, signalHandler);
  
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.allow_undeclared_parameters(true);
  options.automatically_declare_parameters_from_overrides(true);

  rclcpp::Node::SharedPtr nh;
  image_transport::ImageTransport it_(nh);
  auto mapper = std::make_shared<LIVMapper>(nh, "laserMapping", options);
  g_mapper_ptr = mapper;  // Store for signal handler
  
  mapper->initializeSubscribersAndPublishers(nh, it_);
  mapper->run(nh);
  
  // Normal exit - also save
  std::cout << "\033[1;32mNormal exit - saving aggregated PCD files...\033[0m" << std::endl;
  mapper->savePCD(true);
  
  rclcpp::shutdown();
  return 0;
}
