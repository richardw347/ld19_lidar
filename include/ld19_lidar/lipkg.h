/**
* @file         lipkg.h
* @author       LD Robot
* @version      V01

* @brief
* @note
* @attention    COPYRIGHT LDROBOT
**/

#ifndef LD19_LIDAR_LIPKG
#define LD19_LIDAR_LIPKG

#include <stdint.h>
#include <vector>
#include <array>
#include <iostream>
#include <sensor_msgs/msg/laser_scan.hpp>

#define ANGLE_TO_RADIAN(angle) ((angle) * 3141.59 / 180000)
#define RADIAN_TO_ANGLE(angle) ((angle) * 180000 / 3141.59)

enum
{
  PKG_HEADER = 0x54,
  PKG_VER_LEN = 0x2C,
  POINT_PER_PACK = 12,
};

typedef struct __attribute__((packed))
{
  uint16_t distance;
  uint8_t confidence;
} LidarPointStructDef;

typedef struct __attribute__((packed))
{
  uint8_t header;
  uint8_t ver_len;
  uint16_t speed;
  uint16_t start_angle;
  LidarPointStructDef point[POINT_PER_PACK];
  uint16_t end_angle;
  uint16_t timestamp;
  uint8_t crc8;
} LiDARFrameTypeDef;

struct PointData
{
  float angle;
  uint16_t distance;
  uint8_t confidence;
  double x;
  double y;
  PointData(float angle, uint16_t distance, uint8_t confidence, double x = 0, double y = 0)
  {
    this->angle = angle;
    this->distance = distance;
    this->confidence = confidence;
    this->x = x;
    this->y = y;
  }
  PointData()
  {
  }
  friend std::ostream & operator << (std::ostream & os, const PointData & data)
      {
      os << data.angle << " " << data.distance << " " << (int)data.confidence << " " << data.x <<
      " " << data.y;
      return os;
    }
};

class LiPkg
{
public:
  LiPkg();
  /*Lidar spin speed (Hz)*/
  double GetSpeed(void)
  {
    return mSpeed;
  }
  uint16_t GetTimestamp(void)
  {
    return mTimestamp;
  } /*time stamp of the packet */
  bool IsFrameReady(void)
  {
    return mFrameReady;
  } /*Lidar data frame is ready*/
  void ResetFrameReady(void)
  {
    mFrameReady = false;
  }
  long GetErrorTimes(void)
  {
    return mErrorTimes;
  }                                          /*the number of errors in parser process of lidar data frame*/
  bool Parse(const uint8_t * data, long len); /*parse single packet*/
  bool AssemblePacket();                     /*combine stantard data into data frames and calibrate*/
  void SetPopulateCallback(
    std::function < void(const std::vector < PointData > &laser_data) > callback)
  {
    mPopulateCallback = callback;
  }

private:
  uint16_t mTimestamp;
  double mSpeed;
  std::vector < uint8_t > mDataTmp;
  long mErrorTimes;
  std::array < PointData, POINT_PER_PACK > mOnePkg;
  std::vector < PointData > mFrameTmp;
  bool mFrameReady;
  std::function < void(const std::vector < PointData > &laser_data) > mPopulateCallback;
};
/********************* (C) COPYRIGHT LD Robot *******END OF FILE ********/
#endif /* LD19_LIDAR_LIPKG */
