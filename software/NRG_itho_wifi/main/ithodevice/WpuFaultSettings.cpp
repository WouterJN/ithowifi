// Heat pump (WPU) fault settings: read-only viewer. See WpuFaultSettings.h for what and when.
//
// How a read works:
//   The records are read 3 at a time (0x4030 bank 0, see i2csvc::decodeWpuFaultSettings).
//   Each chunk is one job on the I2C queue; when it is done it queues the next chunk. So the
//   bus is never held for the whole read, status polling keeps running in between, and the
//   small I2C queue (10 jobs) is never filled by this feature.
//   A chunk without a usable reply is tried once more. If the very first chunk fails twice the
//   heat pump does not support it ("unsupported"); a later chunk that fails is marked invalid
//   and the read continues ("partial").
//
// Number of records:
//   The heat pump does not report how many fault settings it has; it depends on the list version
//   (i2csvc::wpuFaultSettingsCount). For an unknown list version the newest known size is read
//   and records past the real end (no reply) are dropped at the end.

#include "WpuFaultSettings.h"

#include "IthoDevice.h"
#include "i2c_service.h"
#include "notifyClients.h"
#include "sys_log.h"

#include <vector>

namespace
{
  constexpr uint32_t STALL_TIMEOUT_MS = 10000; // no progress for this long while reading = failed

  enum class State : uint8_t
  {
    none,
    reading,
    ok,
    partial,
    unsupported,
    error
  };

  struct Cache
  {
    State state = State::none;
    uint8_t listVersion = 0;
    bool listKnown = false;
    uint8_t count = 0;
    std::vector<i2csvc::WpuFaultSetting> rec;
    std::vector<uint8_t> valid; // 1 = record received
    uint8_t retries = 0;        // retries of the current chunk
    uint32_t lastProgress = 0;  // millis() of the last chunk while reading
    uint32_t readAt = 0;        // millis() when the last read finished
    bool hasData = false;
  };

  Cache cache;
  SemaphoreHandle_t cacheMutex = xSemaphoreCreateMutex();

  bool lock() { return xSemaphoreTake(cacheMutex, pdMS_TO_TICKS(200)) == pdTRUE; }
  void unlock() { xSemaphoreGive(cacheMutex); }

  const char *stateName(State s)
  {
    switch (s)
    {
    case State::reading:
      return "reading";
    case State::ok:
      return "ok";
    case State::partial:
      return "partial";
    case State::unsupported:
      return "unsupported";
    case State::error:
      return "error";
    default:
      return "none";
    }
  }

  // A read that stopped making progress (e.g. a chunk job was dropped) counts as failed
  void checkStall()
  {
    if (cache.state == State::reading && millis() - cache.lastProgress > STALL_TIMEOUT_MS)
      cache.state = State::error;
  }

  // Sends the current state to the web page; the full table only at the end of a read
  void pushToWeb(bool withFaults)
  {
    JsonDocument doc;
    if (getWpuFaultSettingsJSON(doc["wpufaultsettings"].to<JsonObject>(), withFaults))
      notifyClients(doc.as<JsonObject>());
  }

  void readChunk(uint8_t index);

  void queueChunk(uint8_t index)
  {
    i2cQueueAddCmd([index]()
                   { readChunk(index); });
  }

  // Called with the lock held: decides the final state of a read
  void finishLocked()
  {
    size_t lastValid = 0;
    size_t nValid = 0;
    for (size_t i = 0; i < cache.valid.size(); i++)
    {
      if (cache.valid[i])
      {
        nValid++;
        lastValid = i + 1;
      }
    }
    // Unknown list version: records after the last answered one do not exist on this unit
    if (!cache.listKnown && nValid > 0 && lastValid < cache.count)
    {
      cache.count = static_cast<uint8_t>(lastValid);
      cache.rec.resize(lastValid);
      cache.valid.resize(lastValid);
    }
    if (nValid == 0)
      cache.state = State::unsupported;
    else
      cache.state = nValid == cache.count ? State::ok : State::partial;
    cache.hasData = nValid > 0;
    cache.readAt = millis();
  }

  // Reads one chunk of up to 3 records starting at index. Runs from the I2C queue.
  void readChunk(uint8_t index)
  {
    if (!isWpuDevice())
    {
      if (lock())
      {
        cache.state = State::error;
        unlock();
      }
      pushToWeb(false);
      return;
    }

    if (!lock())
      return; // lock busy for >200 ms: the stall check turns this into "error"
    const uint8_t count = cache.count;
    unlock();
    if (index >= count)
      return;
    const uint8_t n = (count - index) < i2csvc::WPU_FAULT_SETTINGS_PER_READ ? (count - index) : i2csvc::WPU_FAULT_SETTINGS_PER_READ;

    uint8_t rx[I2C_SLAVE_RX_BUF_LEN]{};
    const size_t len = queryBank4030(0x82, i2csvc::WPU_FAULT_SETTINGS_BANK, index, n, rx);
    const i2csvc::Reply r = i2csvc::parseReply(rx, len);
    uint16_t start = 0;
    std::vector<i2csvc::WpuFaultSetting> out;
    // Only accept a reply for exactly the records we asked for (a late reply to an earlier
    // request would have another start index)
    const bool good = len && r.payload && r.ok && i2csvc::decodeWpuFaultSettings(r.payload, r.len, start, out) &&
                      start == index && !out.empty() && out.size() <= n;

    if (!lock())
      return;
    uint8_t next;
    if (good)
    {
      for (size_t i = 0; i < out.size(); i++)
      {
        cache.rec[index + i] = out[i];
        cache.valid[index + i] = 1;
      }
      cache.retries = 0;
      next = static_cast<uint8_t>(index + out.size());
    }
    else if (cache.retries < 1)
    {
      cache.retries++;
      cache.lastProgress = millis();
      unlock();
      queueChunk(index); // try this chunk once more
      return;
    }
    else
    {
      cache.retries = 0;
      if (index == 0)
      {
        // Not even the first records: this heat pump (firmware) does not answer bank 0
        finishLocked();
        unlock();
        W_LOG("I2C: heat pump fault settings (4030 bank 0) not answered");
        pushToWeb(true);
        return;
      }
      next = static_cast<uint8_t>(index + n); // skip, these records stay invalid
    }
    cache.lastProgress = millis();

    if (next < cache.count)
    {
      unlock();
      pushToWeb(false); // progress
      queueChunk(next);
      return;
    }

    finishLocked();
    const State st = cache.state;
    const uint8_t cnt = cache.count;
    unlock();
    I_LOG("I2C: heat pump fault settings read: %s, %u records", stateName(st), cnt);
    pushToWeb(true);
  }

  void addFaultsArray(JsonArray arr)
  {
    for (size_t i = 0; i < cache.rec.size(); i++)
    {
      JsonObject o = arr.add<JsonObject>();
      o["code"] = i;
      const char *text = i2csvc::wpuFaultName(static_cast<uint8_t>(i), cache.listVersion);
      if (text)
        o["text"] = text;
      else
        o["text"] = "fault " + std::to_string(i);
      o["valid"] = cache.valid[i] != 0;
      if (!cache.valid[i])
        continue;
      const i2csvc::WpuFaultSetting &s = cache.rec[i];
      o["priority"] = s.priority;
      o["fault_time"] = s.faultTime;
      o["retry_time"] = s.retryTime;
      o["max_retries"] = s.maxRetries;
      o["report_type"] = s.reportType;
      const char *rep = i2csvc::wpuReportTypeName(s.reportType);
      o["report"] = rep ? rep : "unknown";
      o["block_flags"] = s.blockFlags;
      JsonArray blocks = o["blocks"].to<JsonArray>();
      for (uint8_t b = 0; b < 8; b++)
        if (s.blockFlags & (1 << b))
          blocks.add(i2csvc::wpuBlockFlagName(b));
    }
  }
} // namespace

WpuFaultSettingsStart wpuFaultSettingsRefresh()
{
  if (!isWpuDevice())
    return WpuFaultSettingsStart::not_wpu;
  if (!lock())
    return WpuFaultSettingsStart::busy;
  checkStall();
  if (cache.state == State::reading)
  {
    unlock();
    return WpuFaultSettingsStart::busy;
  }
  const uint8_t lv = itho_fwversion;
  const uint8_t known = i2csvc::wpuFaultSettingsCount(lv);
  cache.listVersion = lv;
  cache.listKnown = known != 0;
  cache.count = known ? known : i2csvc::WPU_FAULT_SETTINGS_MAX;
  cache.rec.assign(cache.count, i2csvc::WpuFaultSetting{});
  cache.valid.assign(cache.count, 0);
  cache.retries = 0;
  cache.hasData = false;
  cache.readAt = 0;
  cache.state = State::reading;
  cache.lastProgress = millis();
  const uint8_t count = cache.count;
  unlock();

  D_LOG("I2C: reading heat pump fault settings, list version %u, %u records%s", lv, count,
        known ? "" : " (unknown list version)");
  queueChunk(0);
  return WpuFaultSettingsStart::started;
}

bool wpuFaultSettingsNeedRead()
{
  if (!isWpuDevice() || !lock())
    return false;
  checkStall();
  const bool need = cache.state != State::reading && (!cache.hasData || cache.listVersion != itho_fwversion);
  unlock();
  return need;
}

bool getWpuFaultSettingsJSON(JsonObject root, bool withFaults)
{
  if (!isWpuDevice())
  {
    root["state"] = "not_wpu";
    root["device"] = getIthoType();
    return true;
  }
  if (!lock())
    return false;
  checkStall();
  root["state"] = stateName(cache.state);
  root["list_version"] = cache.listVersion;
  root["list_version_known"] = cache.listKnown;
  root["count"] = cache.count;
  uint8_t nValid = 0;
  for (uint8_t v : cache.valid)
    nValid += v;
  root["read"] = nValid;
  if (cache.readAt)
    root["age"] = (millis() - cache.readAt) / 1000UL;
  root["stale"] = cache.hasData && cache.listVersion != itho_fwversion;
  if (withFaults)
  {
    JsonArray rt = root["report_types"].to<JsonArray>();
    for (uint8_t i = 0; i < i2csvc::WPU_REPORT_TYPE_COUNT; i++)
      rt.add(i2csvc::wpuReportTypeName(i));
    JsonArray bf = root["block_flags"].to<JsonArray>();
    for (uint8_t b = 0; b < 8; b++)
      bf.add(i2csvc::wpuBlockFlagName(b));
    if (cache.state != State::reading)
      addFaultsArray(root["faults"].to<JsonArray>());
  }
  unlock();
  return true;
}
