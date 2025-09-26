#include <cstddef>
#include <cstdio>
#include <iostream>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <utility>
#include <vector>

#include "irb_adapter.h"
#include "vex.h"

namespace irb_adapter {
enum camera_index
{
  x_center_low,
  x_center_hi,
  y_center_low,
  y_center_hi,
  block_width_low,
  block_width_hi,
  block_height_low,
  block_height_hi
};

enum photo_diode_index
{
  diode_number,
  intensity_value
};

adapter::adapter(uint8_t p_port)
  : m_port(p_port)
{
  // make thread to run reads here
  char port_buffer[12];
  sprintf(port_buffer, "/dev/port%u", m_port);
  m_port_file = fopen(port_buffer, "wb+");
  if (m_port_file == NULL) {
    printf("Failed opening port\n");
  }
}

bool adapter::request_data(std::vector<char>& write_buffer,
                           std::vector<char>& read_buffer)
{
  bool success = true;
  if (m_port_file == NULL) {
    printf("Port not open...\n");
    return false;
  }

  auto bytes_written = fwrite(write_buffer.data(),
                              sizeof write_buffer[0],
                              write_buffer.size(),
                              m_port_file);
  if (bytes_written != write_buffer.size()) {
    printf("Failed write to port, %u bytes written.\n", bytes_written);
    success = false;
  }
  // wait for data
  vex::this_thread::sleep_for(85);

  // rudimentary "timeout"
  size_t bytes_read = 0;
  int attempts = 0;
  while (attempts < 15 && bytes_read == 0) {
    bytes_read = fread(&read_buffer[0], 1, read_buffer.size(), m_port_file);
    attempts++;
  }
  if (bytes_read != read_buffer.size()) {
    printf("Failed read from port, %u bytes read.\n", bytes_read);
    success = false;
  }
  return success;
}

adapter::low_freq_data adapter::get_low_freq_data()
{
  std::vector<char> read_buffer(3);
  std::vector<char> write_buffer = { 'l' };
  adapter::low_freq_data return_data;

  if (request_data(write_buffer, read_buffer)) {
    // check checksum
    auto calculated_checksum =
      read_buffer[diode_number] + read_buffer[intensity_value];
    if (calculated_checksum != read_buffer.back()) {
      printf("Checksum mismatch...\n");
      return return_data;
    }
    return_data.strongest_photo_diode = (int)read_buffer[diode_number];
    return_data.intensity = (int)read_buffer[intensity_value] & 0b01111111;
  }
  return return_data;
}

adapter::hi_freq_data adapter::get_hi_freq_data()
{
  std::vector<char> read_buffer(3);
  std::vector<char> write_buffer = { 'h' };
  adapter::hi_freq_data return_data;

  if (request_data(write_buffer, read_buffer)) {
    // check checksum
    auto calculated_checksum =
      read_buffer[diode_number] + read_buffer[intensity_value];
    if (calculated_checksum != read_buffer.back()) {
      printf("Checksum mismatch...\n");
      return return_data;
    }
    return_data.strongest_photo_diode = (int)read_buffer[diode_number];
    return_data.intensity = (int)read_buffer[intensity_value] & 0b01111111;
  }
  return return_data;
}

adapter::camera_data adapter::get_cam_data()
{
  std::vector<char> read_buffer(9);
  std::vector<char> write_buffer = { 'c' };
  adapter::camera_data return_data;

  if (request_data(write_buffer, read_buffer)) {
    // check checksum
    uint8_t calculated_checksum = 0x00;
    for (size_t i = 0; i < read_buffer.size() - 1; i++) {
      calculated_checksum += read_buffer[i];
    }
    if (calculated_checksum != read_buffer.back()) {
      printf("Checksum mismatch...\n");
      return return_data;
    }
    return_data.x_center = ((uint8_t)read_buffer[x_center_hi] << 8) |
                           (uint8_t)read_buffer[x_center_low];
    return_data.y_center = ((uint8_t)read_buffer[y_center_hi] << 8) |
                           (uint8_t)read_buffer[y_center_low];
    return_data.block_width = ((uint8_t)read_buffer[block_width_hi] << 8) |
                              (uint8_t)read_buffer[block_width_low];
    return_data.block_height = ((uint8_t)read_buffer[block_height_hi] << 8) |
                               (uint8_t)read_buffer[block_height_low];
  }
  return return_data;
}

}  // namespace irb_adapter
