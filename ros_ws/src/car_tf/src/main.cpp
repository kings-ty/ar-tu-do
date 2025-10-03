#include "laserscan_transformer.h"
/**
 * @brief Starts the laserscan transformer
 *
 * @param argc number of input arguments
 * @param argv input arguments
 * @return int exit code
 */
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto laserscan_transformer = std::make_shared<LaserscanTransformer>();
    rclcpp::spin(laserscan_transformer);
    rclcpp::shutdown();

    return EXIT_SUCCESS;
}