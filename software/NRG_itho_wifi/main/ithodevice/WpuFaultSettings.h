#pragma once

// Heat pump (WPU) fault settings: read-only viewer.
//
// Every heat pump fault has a configuration record (priority, fault time, retry time, max
// retries, report type, block flags) that decides how the unit reacts to it. This module reads
// those records from the heat pump (0x4030 bank 0) on request and keeps the result in RAM for
// the web page and the REST API. Nothing is written to the heat pump.
//
// When it reads:
//   - opening the "Heat pump" page when there is no data yet (or the list version changed),
//   - the "Read from heat pump" button,
//   - POST /api/v2/wpu/fault_settings/refresh.
// Never at boot and never periodically: the settings only change when someone changes them.
//
// Other devices: every entry point checks isWpuDevice() first, so HRU, Autotemp, CVE etc.
// never get a bank 0 request.

#include <cstdint>
#include <ArduinoJson.h>

enum class WpuFaultSettingsStart : uint8_t
{
  started,    // a new read was queued
  busy,       // a read is already running
  not_wpu     // connected device is not a heat pump
};

// Starts reading all fault settings (in the background, via the I2C queue)
WpuFaultSettingsStart wpuFaultSettingsRefresh();

// True when there is no usable data yet for the current list version
bool wpuFaultSettingsNeedRead();

// Fills root with the last read result:
//   state          "none" | "reading" | "ok" | "partial" | "unsupported" | "error" | "not_wpu"
//   list_version, list_version_known, count, read (records received), age (s), stale
//   report_types[], block_flags[] (names, index = value / bit)
//   faults[]       {code, text, valid, priority, fault_time, retry_time, max_retries,
//                   report_type, report, block_flags, blocks[]}
// With withFaults = false only the header fields are added (progress messages).
// Returns false if the data lock could not be taken (caller may retry).
bool getWpuFaultSettingsJSON(JsonObject root, bool withFaults = true);
