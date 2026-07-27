#include <Arduino.h>

namespace Config {
constexpr uint32_t kSerialBaud = 1000000;
constexpr size_t kChannelCount = 8;
constexpr size_t kPacketLength = 1 + 2 * kChannelCount;
constexpr uint8_t kPacketHeader = 'P';
constexpr uint8_t kPpmPin = 3;
constexpr uint16_t kFrameUs = 20000;
constexpr uint16_t kMarkUs = 300;
constexpr uint16_t kMinimumPulseUs = 1000;
constexpr uint16_t kMaximumPulseUs = 2000;
constexpr uint32_t kCommandTimeoutUs = 250000;
constexpr uint16_t kTimerTicksPerUs = 3;
constexpr uint16_t kFallbackPulseUs[kChannelCount] = {
  1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500
};
}

uint8_t gPacket[Config::kPacketLength];
size_t gPacketIndex = 0;
volatile uint16_t gActivePulseUs[Config::kChannelCount];
volatile uint16_t gPendingPulseUs[Config::kChannelCount];
volatile bool gPendingReady = false;
volatile bool gMarkActive = false;
volatile uint8_t gChannelIndex = 0;
uint32_t gLastCommandUs = 0;
bool gCommandActive = false;

inline PortGroup& portGroupForPin(uint8_t pin) {
  return PORT->Group[g_APinDescription[pin].ulPort];
}

inline uint32_t portMaskForPin(uint8_t pin) {
  return 1ul << g_APinDescription[pin].ulPin;
}

inline void setPpmLow() {
  PortGroup& group = portGroupForPin(Config::kPpmPin);
  uint32_t mask = portMaskForPin(Config::kPpmPin);
  group.OUTCLR.reg = mask;
  group.DIRSET.reg = mask;
}

inline void setPpmHigh() {
  PortGroup& group = portGroupForPin(Config::kPpmPin);
  uint32_t mask = portMaskForPin(Config::kPpmPin);
  group.OUTSET.reg = mask;
  group.DIRSET.reg = mask;
}

void waitForTimer3Sync() {
  while (TC3->COUNT16.STATUS.bit.SYNCBUSY) {
  }
}

void scheduleTimerUs(uint16_t durationUs) {
  TC3->COUNT16.CC[0].reg =
    static_cast<uint16_t>(durationUs * Config::kTimerTicksPerUs - 1U);
  waitForTimer3Sync();
}

void configureTimer3() {
  PM->APBCMASK.reg |= PM_APBCMASK_TC3;
  GCLK->CLKCTRL.reg = static_cast<uint16_t>(
    GCLK_CLKCTRL_CLKEN |
    GCLK_CLKCTRL_GEN_GCLK0 |
    GCLK_CLKCTRL_ID(GCM_TCC2_TC3));
  while (GCLK->STATUS.bit.SYNCBUSY) {
  }

  TC3->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;
  waitForTimer3Sync();
  TC3->COUNT16.CTRLA.reg =
    TC_CTRLA_MODE_COUNT16 |
    TC_CTRLA_WAVEGEN_MFRQ |
    TC_CTRLA_PRESCALER_DIV16 |
    TC_CTRLA_PRESCSYNC_PRESC;
  waitForTimer3Sync();
  TC3->COUNT16.COUNT.reg = 0;
  waitForTimer3Sync();
  scheduleTimerUs(1000U);
  TC3->COUNT16.INTFLAG.reg = TC_INTFLAG_MC0;
  TC3->COUNT16.INTENSET.reg = TC_INTENSET_MC0;
  NVIC_ClearPendingIRQ(TC3_IRQn);
  NVIC_SetPriority(TC3_IRQn, 0);
  NVIC_EnableIRQ(TC3_IRQn);
  TC3->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;
  waitForTimer3Sync();
}

uint16_t decodeUint16LE(const uint8_t* data) {
  return static_cast<uint16_t>(data[0]) |
         static_cast<uint16_t>(static_cast<uint16_t>(data[1]) << 8);
}

bool decodePacket(uint16_t* pulseUs) {
  for (size_t channelIndex = 0;
       channelIndex < Config::kChannelCount;
       ++channelIndex) {
    uint16_t value = decodeUint16LE(gPacket + 1U + 2U * channelIndex);
    if (value < Config::kMinimumPulseUs ||
        value > Config::kMaximumPulseUs) {
      return false;
    }
    pulseUs[channelIndex] = value;
  }
  return true;
}

void queuePulses(const uint16_t* pulseUs) {
  noInterrupts();
  for (size_t channelIndex = 0;
       channelIndex < Config::kChannelCount;
       ++channelIndex) {
    gPendingPulseUs[channelIndex] = pulseUs[channelIndex];
  }
  gPendingReady = true;
  interrupts();
}

void queueFallback() {
  uint16_t pulseUs[Config::kChannelCount];
  for (size_t channelIndex = 0;
       channelIndex < Config::kChannelCount;
       ++channelIndex) {
    pulseUs[channelIndex] = Config::kFallbackPulseUs[channelIndex];
  }
  queuePulses(pulseUs);
}

void serviceSerialInput() {
  while (Serial.available() > 0) {
    uint8_t nextByte = static_cast<uint8_t>(Serial.read());
    if (gPacketIndex == 0U && nextByte != Config::kPacketHeader) {
      continue;
    }

    gPacket[gPacketIndex++] = nextByte;
    if (gPacketIndex != Config::kPacketLength) {
      continue;
    }

    uint16_t pulseUs[Config::kChannelCount];
    if (decodePacket(pulseUs)) {
      queuePulses(pulseUs);
      gLastCommandUs = micros();
      gCommandActive = true;
    }
    gPacketIndex = 0U;
  }
}

void serviceCommandTimeout() {
  if (!gCommandActive) {
    return;
  }
  uint32_t elapsedUs = micros() - gLastCommandUs;
  if (elapsedUs < Config::kCommandTimeoutUs) {
    return;
  }
  queueFallback();
  gCommandActive = false;
}

uint16_t computeSyncGapUs() {
  uint32_t channelSumUs = 0U;
  for (size_t channelIndex = 0;
       channelIndex < Config::kChannelCount;
       ++channelIndex) {
    channelSumUs += gActivePulseUs[channelIndex];
  }
  return static_cast<uint16_t>(
    Config::kFrameUs - channelSumUs - Config::kMarkUs);
}

void setup() {
  Serial.begin(Config::kSerialBaud);
  pinMode(Config::kPpmPin, OUTPUT);
  setPpmLow();
  for (size_t channelIndex = 0;
       channelIndex < Config::kChannelCount;
       ++channelIndex) {
    gActivePulseUs[channelIndex] =
      Config::kFallbackPulseUs[channelIndex];
    gPendingPulseUs[channelIndex] =
      Config::kFallbackPulseUs[channelIndex];
  }
  configureTimer3();
}

void loop() {
  serviceSerialInput();
  serviceCommandTimeout();
}

void TC3_Handler() {
  if ((TC3->COUNT16.INTFLAG.reg & TC_INTFLAG_MC0) == 0U) {
    return;
  }
  TC3->COUNT16.INTFLAG.reg = TC_INTFLAG_MC0;

  if (!gMarkActive) {
    if (gChannelIndex == 0U && gPendingReady) {
      for (size_t channelIndex = 0;
           channelIndex < Config::kChannelCount;
           ++channelIndex) {
        gActivePulseUs[channelIndex] = gPendingPulseUs[channelIndex];
      }
      gPendingReady = false;
    }
    setPpmHigh();
    scheduleTimerUs(Config::kMarkUs);
    gMarkActive = true;
    return;
  }

  setPpmLow();
  if (gChannelIndex < Config::kChannelCount) {
    uint16_t gapUs = static_cast<uint16_t>(
      gActivePulseUs[gChannelIndex] - Config::kMarkUs);
    ++gChannelIndex;
    scheduleTimerUs(gapUs);
  } else {
    gChannelIndex = 0U;
    scheduleTimerUs(computeSyncGapUs());
  }
  gMarkActive = false;
}
