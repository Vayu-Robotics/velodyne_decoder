/*
 *  Copyright (C) 2007, 2009-2012 Austin Robot Technology, Patrick Beeson, Jack O'Quin
 *  Copyright (C) 2021, Martin Valgur
 *
 *  License: Modified BSD Software License Agreement
 */

#include "velodyne_decoder/stream_decoder.h"

#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>

#include "velodyne_decoder/time_conversion.h"

namespace velodyne_decoder {

StreamDecoder::StreamDecoder(const Config &config) : config_(config), scan_decoder_(config) {
  packets_per_scan_ = calc_packets_per_scan(config_.model, config_.rpm);
  scan_packets_.reserve(packets_per_scan_);
}

int StreamDecoder::calc_packets_per_scan(const std::string &model, double rpm) {
  double packet_rate; // packet frequency (Hz)
  if (model == "Alpha Prime") {
    // 3 firing cycles in a data packet. 3 x 53.3 μs = 0.1599 ms is the
    // accumulation delay per packet.
    // 1 packet/0.1599 ms = 6253.9 packets/second
    // packet_rate = 6253.9;

    // Newer firmware calculations
    // 3 * 58.5688 us = 169.7064 us per packet = 5892.53 packets / second
    packet_rate = 5892.53;

  } else if (model == "HDL-64E_S2" || model == "HDL-64E_S2.1") {
    // generates 1333312 points per second
    // 1 packet holds 384 points
    packet_rate = 1333312. / 384.;
  } else if (model == "HDL-64E") {
    packet_rate = 2600.0;
  } else if (model == "HDL-64E_S3") {
    // generates 2222220 points per second (half for strongest and half for latest)
    // 1 packet holds 384 points
    packet_rate = 2222220. / 384.;
  } else if (model == "HDL-32E") {
    packet_rate = 1808.0;
  } else if (model == "VLP-32C") {
    packet_rate = 1507.0;
  } else if (model == "VLP-16") {
    // 754 Packets/Second for Last or Strongest mode 1508 for dual (VLP-16 User Manual)
    packet_rate = 754;
  } else {
    throw std::invalid_argument("Unknown sensor model: " + model);
  }
  if (rpm <= 0) {
    throw std::invalid_argument("Invalid RPM value in config: " + std::to_string(rpm));
  }
  double frequency = rpm / 60.0;
  return static_cast<int>(ceil(packet_rate / frequency));
}

// Re-written to use azimuth based clouds instead of packet
std::optional<std::pair<Time, PointCloud>> //
StreamDecoder::decode(Time stamp, const RawPacketData &packet) {
  if (config_.gps_time) {
    stamp = getPacketTimestamp(&(packet[1200]), stamp);
  }

  auto *raw = reinterpret_cast<const raw_packet_t *>(&packet[0]);

  int last_block_azimuth = raw->blocks[11].rotation - 18000;
  while (last_block_azimuth < 0) {
    last_block_azimuth += 36000;
  }
  while (last_block_azimuth >= 36000) {
    last_block_azimuth -= 36000;
  }

  scan_packets_.emplace_back(stamp, packet);

  if (last_block_azimuth < previous_packet_azimuth_) {
    Time scan_stamp;
    scan_stamp = scan_packets_.back().stamp;
    PointCloud scan = scan_decoder_.decode(scan_stamp, scan_packets_);

    scan_packets_.clear();
    // Add this packet back in as it contains the beginning of the spin
    scan_packets_.emplace_back(stamp, packet);
    previous_packet_azimuth_ = last_block_azimuth;

    return std::make_pair(scan_stamp, scan);
  }

  previous_packet_azimuth_ = last_block_azimuth;
  return std::nullopt;
}

std::optional<std::pair<Time, PointCloud>> //
StreamDecoder::decode(const VelodynePacket &packet) {
  return decode(packet.stamp, packet.data);
}

} // namespace velodyne_decoder
