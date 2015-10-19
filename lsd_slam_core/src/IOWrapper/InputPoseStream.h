#ifndef INPUTPOSESTREAM_H
#define INPUTPOSESTREAM_H

#include "IOWrapper/NotifyBuffer.h"
#include "IOWrapper/TimestampedObject.h"

namespace lsd_slam {

class InputPoseStream {
 public:
  virtual ~InputPoseStream() {};

  virtual void run() {};

  inline NotifyBuffer<TimestampedPose>* getBuffer() {
    return poseBuffer;
  };

 protected:
  NotifyBuffer<TimestampedPose>* poseBuffer;
};


}

#endif
