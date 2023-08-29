#include "rclcpp/rclcpp.hpp"
class ConfigParameterServer : public rclcpp::Node
{
public:
    ConfigParameterServer() : Node("ConfigParameterServer",
                                   rclcpp::NodeOptions()
                                       .allow_undeclared_parameters(true)
                                       .automatically_declare_parameters_from_overrides(true)) {}
private:
};
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ConfigParameterServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}