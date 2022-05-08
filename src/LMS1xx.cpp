#include "ros2_sick/LMS1xx/LMS1xx.h"

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <memory>

LMS1xx::LMS1xx() : connected_(false) {}

LMS1xx::~LMS1xx() {}

void LMS1xx::connect(std::string host, int port)
{
  if (!connected_)
  {
    socket_fd_ = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (socket_fd_)
    {
      struct sockaddr_in stSockAddr;
      stSockAddr.sin_family = PF_INET;
      stSockAddr.sin_port = htons(port);
      inet_pton(AF_INET, host.c_str(), &stSockAddr.sin_addr);

      int ret = ::connect(socket_fd_, (struct sockaddr*)&stSockAddr,
                          sizeof(stSockAddr));

      if (ret == 0)
      {
        connected_ = true;
      }
    }
  }
}

void LMS1xx::disconnect()
{
  if (connected_)
  {
    close(socket_fd_);
    connected_ = false;
  }
}

bool LMS1xx::isConnected() { return connected_; }

void LMS1xx::startMeas()
{
  sprintf(buf, "%c%s%c", 0x02, "sMN LMCstartmeas", 0x03);
  write(socket_fd_, buf, strlen(buf));

  int len = read(socket_fd_, buf, 100);
  if (buf[0] != 0x02) std::cout << "invalid packet recieved" << std::endl;
  buf[len] = 0;
}

void LMS1xx::stopMeas()
{
  sprintf(buf, "%c%s%c", 0x02, "sMN LMCstopmeas", 0x03);
  write(socket_fd_, buf, strlen(buf));

  int len = read(socket_fd_, buf, 100);
  if (buf[0] != 0x02) std::cout << "invalid packet recieved" << std::endl;
  buf[len] = 0;
}

status_t LMS1xx::queryStatus()
{
  sprintf(buf, "%c%s%c", 0x02, "sRN STlms", 0x03);

  write(socket_fd_, buf, strlen(buf));

  int len = read(socket_fd_, buf, 100);
  if (buf[0] != 0x02) std::cout << "invalid packet recieved" << std::endl;
  buf[len] = 0;
  std::cout << "RX " << buf << std::endl;

  int ret;
  sscanf((buf + 10), "%d", &ret);

  return (status_t)ret;
}

void LMS1xx::login()
{
  int result;
  sprintf(buf, "%c%s%c", 0x02, "sMN SetAccessMode 03 F4724744", 0x03);
  std::cout << buf << 0x02 << " sMN SetAccessMode 03 F4724744 " << 0x03
            << std::endl;
  fd_set readset;
  struct timeval timeout;

  do  //loop until data is available to read
  {
    timeout.tv_sec = 1;
    timeout.tv_usec = 0;

    write(socket_fd_, buf, strlen(buf));

    FD_ZERO(&readset);
    FD_SET(socket_fd_, &readset);
    result = select(socket_fd_ + 1, &readset, NULL, NULL, &timeout);

  } while (result <= 0);

  int len = read(socket_fd_, buf, 100);
  if (buf[0] != 0x02) std::cout << "invalid packet recieved" << std::endl;
  buf[len] = 0;
  std::cout << "RX " << buf << std::endl;
}

scanCfg LMS1xx::getScanCfg() const
{
  scanCfg cfg;
  sprintf(buf, "%c%s%c", 0x02, "sRN LMPscancfg", 0x03);

  write(socket_fd_, buf, strlen(buf));

  int len = read(socket_fd_, buf, 100);
  if (buf[0] != 0x02) std::cout << "invalid packet recieved" << std::endl;
  buf[len] = 0;
  std::cout << "RX " << buf << std::endl;

  sscanf(buf + 1, "%*s %*s %X %*d %X %X %X", &cfg.scaningFrequency,
         &cfg.angleResolution, &cfg.startAngle, &cfg.stopAngle);
  return cfg;
}

void LMS1xx::setScanCfg(const scanCfg& cfg)
{
  sprintf(buf, "%c%s %X +1 %X %X %X%c", 0x02, "sMN mLMPsetscancfg",
          cfg.scaningFrequency, cfg.angleResolution, cfg.startAngle,
          cfg.stopAngle, 0x03);

  write(socket_fd_, buf, strlen(buf));

  int len = read(socket_fd_, buf, 100);

  buf[len - 1] = 0;
}

void LMS1xx::setScanDataCfg(const scanDataCfg& cfg)
{
  sprintf(buf, "%c%s %02X 00 %d %d 0 %02X 00 %d %d 0 %d +%d%c", 0x02,
          "sWN LMDscandatacfg", cfg.outputChannel, cfg.remission ? 1 : 0,
          cfg.resolution, cfg.encoder, cfg.position ? 1 : 0,
          cfg.deviceName ? 1 : 0, cfg.timestamp ? 1 : 0, cfg.outputInterval,
          0x03);
  std::cout << "TX " << buf << std::endl;

  write(socket_fd_, buf, strlen(buf));

  int len = read(socket_fd_, buf, 100);
  buf[len - 1] = 0;
}

scanOutputRange LMS1xx::getScanOutputRange() const
{
  scanOutputRange outputRange;
  sprintf(buf, "%c%s%c", 0x02, "sRN LMPoutputRange", 0x03);

  write(socket_fd_, buf, strlen(buf));

  read(socket_fd_, buf, 100);

  sscanf(buf + 1, "%*s %*s %*d %X %X %X", &outputRange.angleResolution,
         &outputRange.startAngle, &outputRange.stopAngle);
  return outputRange;
}

void LMS1xx::scanContinous(int start)
{
  sprintf(buf, "%c%s %d%c", 0x02, "sEN LMDscandata", start, 0x03);

  write(socket_fd_, buf, strlen(buf));

  int len = read(socket_fd_, buf, 100);

  if (buf[0] != 0x02) std::cout << "invalid packet recieved" << std::endl;

  buf[len] = 0;
  std::cout << "RX " << buf << std::endl;
}

bool LMS1xx::getScanData(scanData& scan_data)
{
  fd_set rfds;
  FD_ZERO(&rfds);
  FD_SET(socket_fd_, &rfds);

  // Block a total of up to 100ms waiting for more data from the laser.
  while (1)
  {
    // Would be great to depend on linux's behaviour of updating the timeval, but unfortunately
    // that's non-POSIX (doesn't work on OS X, for example).
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 400000;

    int retval = select(socket_fd_ + 1, &rfds, NULL, NULL, &tv);
    if (retval)
    {
      buffer_.readFrom(socket_fd_);

      // Will return pointer if a complete message exists in the buffer,
      // otherwise will return null.
      char* buffer_data = buffer_.getNextBuffer();

      if (buffer_data)
      {
        parseScanData(buffer_data, scan_data);
        buffer_.popLastBuffer();
        return true;
      }
    }
    else
    {
      // Select timed out or there was an fd error.
      return false;
    }
  }
}

void LMS1xx::parseScanData(char* buffer, scanData& data)
{
  // Scan data packet type
  enum class type
  {
    DIST1 = 0,
    DIST2,
    RSST1,
    RSST2,
    //Set only if packet is invalid
    UNINIT
  };

  char* tok = strtok(buffer, " ");  //Type of command
  tok = strtok(NULL, " ");          //Command
  tok = strtok(NULL, " ");          //VersionNumber
  tok = strtok(NULL, " ");          //DeviceNumber
  tok = strtok(NULL, " ");          //Serial number
  tok = strtok(NULL, " ");          //DeviceStatus
  tok = strtok(NULL, " ");          //MessageCounter
  tok = strtok(NULL, " ");          //ScanCounter
  tok = strtok(NULL, " ");          //PowerUpDuration
  tok = strtok(NULL, " ");          //TransmissionDuration
  tok = strtok(NULL, " ");          //InputStatus
  tok = strtok(NULL, " ");          //OutputStatus
  tok = strtok(NULL, " ");          //ReservedByteA
  tok = strtok(NULL, " ");          //ScanningFrequency
  tok = strtok(NULL, " ");          //MeasurementFrequency
  tok = strtok(NULL, " ");
  tok = strtok(NULL, " ");
  tok = strtok(NULL, " ");
  tok = strtok(NULL, " ");  //NumberEncoders
  int NumberEncoders;
  sscanf(tok, "%d", &NumberEncoders);
  for (int i = 0; i < NumberEncoders; i++)
  {
    tok = strtok(NULL, " ");  //EncoderPosition
    tok = strtok(NULL, " ");  //EncoderSpeed
  }

  tok = strtok(NULL, " ");  //NumberChannels16Bit
  int NumberChannels16Bit;
  sscanf(tok, "%d", &NumberChannels16Bit);
  //std::cout << "NumberChannels16Bit : " << NumberChannels16Bit << std::endl;

  for (int i = 0; i < NumberChannels16Bit; i++)
  {
    type type = type::UNINIT;  // 0 DIST1 1 DIST2 2 RSSI1 3 RSSI2
    char content[6];
    tok = strtok(NULL, " ");  //MeasuredDataContent
    sscanf(tok, "%s", content);
    if (!strcmp(content, "DIST1"))
    {
      type = type::DIST1;
    }
    else if (!strcmp(content, "DIST2"))
    {
      type = type::DIST2;
    }
    else if (!strcmp(content, "RSSI1"))
    {
      type = type::RSST1;
    }
    else if (!strcmp(content, "RSSI2"))
    {
      type = type::RSST2;
    }

    if (type == type::UNINIT)
    {  //TODO handle this properly
      std::cout << "Scan packet had no type header! \n";
      return;
    }

    tok = strtok(NULL, " ");  //ScalingFactor
    tok = strtok(NULL, " ");  //ScalingOffset
    tok = strtok(NULL, " ");  //Starting angle
    tok = strtok(NULL, " ");  //Angular step width
    tok = strtok(NULL, " ");  //NumberData
    int NumberData;
    sscanf(tok, "%X", &NumberData);

    switch (type)
    {
      case type::DIST1:
        data.dist_len1 = NumberData;
        break;
      case type::DIST2:
        data.dist_len2 = NumberData;
        break;
      case type::RSST1:
        data.rssi_len1 = NumberData;
        break;
      case type::RSST2:
        data.rssi_len2 = NumberData;
        break;
      case type::UNINIT:
        break;
    }

    for (int i = 0; i < NumberData; i++)
    {
      int dat;
      tok = strtok(NULL, " ");  //data
      sscanf(tok, "%X", &dat);

      switch (type)
      {
        case type::DIST1:
          data.dist1[i] = dat;
          break;
        case type::DIST2:
          data.dist2[i] = dat;
          break;
        case type::RSST1:
          data.rssi1[i] = dat;
          break;
        case type::RSST2:
          data.rssi2[i] = dat;
          break;
        case type::UNINIT:
          break;
      }
    }
  }

  tok = strtok(NULL, " ");  //NumberChannels8Bit
  int NumberChannels8Bit;
  sscanf(tok, "%d", &NumberChannels8Bit);

  for (int i = 0; i < NumberChannels8Bit; i++)
  {
    type type = type::UNINIT;  // 0 DIST1 1 DIST2 2 RSSI1 3 RSSI2
    char content[6];
    tok = strtok(NULL, " ");  //MeasuredDataContent
    sscanf(tok, "%s", content);
    if (!strcmp(content, "DIST1"))
    {
      type = type::DIST1;
    }
    else if (!strcmp(content, "DIST2"))
    {
      type = type::DIST2;
    }
    else if (!strcmp(content, "RSSI1"))
    {
      type = type::RSST1;
    }
    else if (!strcmp(content, "RSSI2"))
    {
      type = type::RSST2;
    }

    if (type == type::UNINIT)
    {  //TODO handle this properly
      std::cout << "Scan packet had no type header! \n";
      return;
    }

    tok = strtok(NULL, " ");  //ScalingFactor
    tok = strtok(NULL, " ");  //ScalingOffset
    tok = strtok(NULL, " ");  //Starting angle
    tok = strtok(NULL, " ");  //Angular step width
    tok = strtok(NULL, " ");  //NumberData
    int NumberData;
    sscanf(tok, "%X", &NumberData);

    switch (type)
    {
      case type::DIST1:
        data.dist_len1 = NumberData;
        break;
      case type::DIST2:
        data.dist_len2 = NumberData;
        break;
      case type::RSST1:
        data.rssi_len1 = NumberData;
        break;
      case type::RSST2:
        data.rssi_len2 = NumberData;
        break;
      case type::UNINIT:
        break;
    }

    for (int i = 0; i < NumberData; i++)
    {
      int dat;
      tok = strtok(NULL, " ");  //data
      sscanf(tok, "%X", &dat);

      switch (type)
      {
        case type::DIST1:
          data.dist1[i] = dat;
          break;
        case type::DIST2:
          data.dist2[i] = dat;
          break;
        case type::RSST1:
          data.rssi1[i] = dat;
          break;
        case type::RSST2:
          data.rssi2[i] = dat;
          break;
        case type::UNINIT:
          break;
      }
    }
  }
}

void LMS1xx::saveConfig()
{
  sprintf(buf, "%c%s%c", 0x02, "sMN mEEwriteall", 0x03);

  write(socket_fd_, buf, strlen(buf));

  int len = read(socket_fd_, buf, 100);

  if (buf[0] != 0x02) std::cout << "invalid packet recieved" << std::endl;
  buf[len] = 0;
  std::cout << "RX: " << buf << std::endl;
}

void LMS1xx::startDevice()
{
  sprintf(buf, "%c%s%c", 0x02, "sMN Run", 0x03);

  write(socket_fd_, buf, strlen(buf));

  int len = read(socket_fd_, buf, 100);

  if (buf[0] != 0x02) std::cout << "invalid packet recieved" << std::endl;
  buf[len] = 0;
  std::cout << "RX " << buf << std::endl;
}
