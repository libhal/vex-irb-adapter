#pragma once

#include <cstdio>
#include <vector>

namespace irb_adapter {
class adapter
{
public:
  struct low_freq_data
  {
    uint8_t strongest_photo_diode = 0;
    uint8_t intensity = 0;
  };

  struct hi_freq_data
  {
    uint8_t strongest_photo_diode = 0;
    uint8_t intensity = 0;
  };

  struct camera_data
  {
    uint16_t x_center = 0;
    uint16_t y_center = 0;
    uint16_t block_width = 0;
    uint16_t block_height = 0;
  };

  adapter(uint8_t p_port);
  low_freq_data get_low_freq_data();
  hi_freq_data get_hi_freq_data();
  camera_data get_cam_data();
  // TODO(#7): Add destructor to close file

private:
  bool request_data(std::vector<char>& write_buffer,
                    std::vector<char>& read_buffer);
  uint8_t m_port;
  FILE* m_port_file;
};
}  // namespace irb_adapter
