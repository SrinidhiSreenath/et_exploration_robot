// CPP Headers
#include <cstdint>
#include <utility>

class Grid {
 private:
  uint32_t gridHeight_;
  uint32_t gridWidth_;
  int8_t OccupancyProbability_;
  bool isFrontierCell_;
  int clusterNumber_;

 public:
  Grid() {}
  ~Grid() {}

  void setGridState(uint32_t height, uint32_t width);

  std::pair<uint32_t, uint32_t> getGridState();

  void updateProbability(int8_t updatedProbability);

  int8_t getProbability();

  void setFrontierStatus(bool status);

  bool getFrontierStatus();

  void setClusterNumber(int num);

  int getClusterNumber();
};
