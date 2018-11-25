#ifndef ALGORITHM_BASE_H
#define ALGORITHM_BASE_H

namespace shop {
namespace common {
class AlgorithmBase {
public:
  virtual bool initAlgorithm() = 0;
  virtual ~AlgorithmBase();

protected:
  AlgorithmBase();
};
} // namespace common
}

#endif