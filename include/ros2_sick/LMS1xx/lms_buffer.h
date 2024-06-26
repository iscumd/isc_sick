/*
	lms_buffer.h
	ISC SICK LMS1xx Node
	For use with LMS1xx series at 25Hz


*/

#pragma once

#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <cstdint>
#include <cstdio>

constexpr uint16_t LMS_BUFFER_SIZE = 50000;
constexpr uint8_t LMS_STX  = 0x02;
constexpr uint8_t LMS_ETX = 0x03;

class LMSBuffer
{
public:
  LMSBuffer() : total_length_(0), end_of_first_message_(0)
  {
  }

  void readFrom(int fd)
  {
    int ret = read(fd, buffer_ + total_length_, sizeof(buffer_) - total_length_);

    if (ret > 0)
    {
      total_length_ += ret;
      //printf("Read %d bytes from fd, total length is %d.", ret, total_length_);
    }
    else
    {

      //printf("Buffer read() returned error.");
    }
  }

  char* getNextBuffer()
  {
    if (total_length_ == 0)
    {
      // Buffer is empty, no scan data present.
      //printf("Empty buffer, nothing to return.");
      return NULL;
    }

    // The objective is to have a message starting at the start of the buffer, so if that's not
    // the case, then we look for a start-of-message character and shift the buffer back, discarding
    // any characters in the middle.
    char* start_of_message = (char*)memchr(buffer_, LMS_STX, total_length_);
    if (start_of_message == NULL)
    {
      // None found, buffer reset.
      //printf("No STX found, dropping %d bytes from buffer.", total_length_);
      total_length_ = 0;
    }
    else if (buffer_ != start_of_message)
    {
      // Start of message found, ahead of the start of buffer. Therefore shift the buffer back.
      //printf("Shifting buffer, dropping %ld bytes, %ld bytes remain.",
      //        (start_of_message - buffer_), total_length_ - (start_of_message - buffer_));
      shiftBuffer(start_of_message);
    }

    // Now look for the end of message character.
    end_of_first_message_ = (char*)memchr(buffer_, LMS_ETX, total_length_);
    if (end_of_first_message_ == NULL)
    {
      // No end of message found, therefore no message to parse and return.
      //printf("No ETX found, nothing to return.");
      return NULL;
    }

    // Null-terminate buffer.
    *end_of_first_message_ = 0;
    return buffer_;
  }

  void popLastBuffer()
  {
    if (end_of_first_message_)
    {
      shiftBuffer(end_of_first_message_ + 1);
      end_of_first_message_ = NULL;
    }
  }

private:
  void shiftBuffer(char* new_start)
  {
    // Shift back anything remaining in the buffer.
    uint16_t remaining_length = total_length_ - (new_start - buffer_);

    if (remaining_length > 0)
    {
      memmove(buffer_, new_start, remaining_length);
    }
    total_length_ = remaining_length;
  }

  char buffer_[LMS_BUFFER_SIZE];
  uint16_t total_length_;

  char* end_of_first_message_;
};
