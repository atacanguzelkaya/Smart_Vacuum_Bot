/* --Includes-- */
#include "../include/Goals.h"
#include "../include/Cleaner.h"

/**
 * @brief      main  Execution starts here
 *
 * @param      argc  The argc
 * @param      argv  The argv
 *
 * @return     Returns 0 upon successful execution
 */
int main(int argc, char **argv) {
const std::vector<std::vector<double>> Goal_Points{{2, 0.1, 90},{0.5, 0.1, 180}};
  ros::init(argc, argv, "Clean");
  ros::NodeHandle n;
  Cleaner Bot(Goal_Points);
  Bot.Clean_Room();
  return 0;
}
