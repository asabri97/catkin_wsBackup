/*
 * Copyright 2017, 2018 Simon Rasmussen (refactor)
 *
 * Copyright 2015, 2016 Thomas Timm Andersen (original version)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include <cstdlib>
#include "ur_modern_driver/ur/consumer.h"
#include "ur_modern_driver/ur/messages_parser.h"
#include "ur_modern_driver/ur/parser.h"
#include "ur_modern_driver/ur/producer.h"
#include "ur_modern_driver/ur/rt_parser.h"
#include "ur_modern_driver/ur/state_parser.h"
#include "ur_modern_driver/ur/stream.h"

static const int UR_PRIMARY_PORT = 30001;

class URFactory : private URMessagePacketConsumer
{
private:
  URStream stream_;
  URMessageParser parser_;

  uint8_t major_version_;
  uint8_t minor_version_;

  bool consume(VersionMessage& vm)
  {
    LOG_INFO("Got VersionMessage:");
    LOG_INFO("project name: %s", vm.project_name.c_str());
    LOG_INFO("version: %u.%u.%d", vm.major_version, vm.minor_version, vm.svn_version);
    LOG_INFO("build date: %s", vm.build_date.c_str());

    major_version_ = vm.major_version;
    minor_version_ = vm.minor_version;

    return true;
  }

  void setupConsumer()
  {
  }
  void teardownConsumer()
  {
  }
  void stopConsumer()
  {
  }

public:
  URFactory(std::string& host) : stream_(host, UR_PRIMARY_PORT)
  {
    URProducer<MessagePacket> prod(stream_, parser_);
    std::vector<unique_ptr<MessagePacket>> results;

    prod.setupProducer();

    if (!prod.tryGet(results) || results.size() == 0)
    {
      LOG_FATAL("No version message received, init failed!");
      std::exit(EXIT_FAILURE);
    }

    for (auto const& p : results)
    {
      p->consumeWith(*this);
    }

    if (major_version_ == 0 && minor_version_ == 0)
    {
      LOG_FATAL("No version message received, init failed!");
      std::exit(EXIT_FAILURE);
    }

    prod.teardownProducer();
  }

  bool isVersion3()
  {
    return major_version_ >= 3;
  }

  std::unique_ptr<URCommander> getCommander(URStream& stream)
  {
    if (major_version_ == 1)
      return std::unique_ptr<URCommander>(new URCommander_V1_X(stream));
    else if (major_version_ == 3)
    {
      if (minor_version_ < 3)
        return std::unique_ptr<URCommander>(new URCommander_V3_1__2(stream));
      else
        return std::unique_ptr<URCommander>(new URCommander_V3_3(stream));
    }
    else if (major_version_ == 5)
    {
      return std::unique_ptr<URCommander>(new URCommander_V3_3(stream));
    }
  }

  std::unique_ptr<URParser<StatePacket>> getStateParser()
  {
    if (major_version_ == 1)
    {
      return std::unique_ptr<URParser<StatePacket>>(new URStateParser_V1_X);
    }
    else if (major_version_ == 3)
    {
      if (minor_version_ < 3)
        return std::unique_ptr<URParser<StatePacket>>(new URStateParser_V3_0__1);
      else if (minor_version_ < 5)
        return std::unique_ptr<URParser<StatePacket>>(new URStateParser_V3_2);
      else if (minor_version_ < 10)
        return std::unique_ptr<URParser<StatePacket>>(new URStateParser_V3_5);
      else if (minor_version_ >= 10)
        return std::unique_ptr<URParser<StatePacket>>(new URStateParser_V3_10__5_4);
      else
      {
        LOG_FATAL("UR software version %u.%u not yet supported", major_version_, minor_version_);
        std::exit(EXIT_FAILURE);
      }
    }
    else if (major_version_ == 5)
    {
      if (minor_version_ < 4)
        return std::unique_ptr<URParser<StatePacket>>(new URStateParser_V3_5);
      else if (minor_version_ >= 4)
        return std::unique_ptr<URParser<StatePacket>>(new URStateParser_V3_10__5_4);
    }
    else
    {
      LOG_FATAL("UR software version %u.%u not yet supported", major_version_, minor_version_);
      std::exit(EXIT_FAILURE);
    }
  }

  std::unique_ptr<URParser<RTPacket>> getRTParser()
  {
    if (major_version_ == 1)
    {
      if (minor_version_ < 8)
        return std::unique_ptr<URParser<RTPacket>>(new URRTStateParser_V1_6__7);
      else
        return std::unique_ptr<URParser<RTPacket>>(new URRTStateParser_V1_8);
    }
    else if (major_version_ == 3)
    {
      if (minor_version_ < 2)
        return std::unique_ptr<URParser<RTPacket>>(new URRTStateParser_V3_0__1);
      else if (minor_version_ < 5)
        return std::unique_ptr<URParser<RTPacket>>(new URRTStateParser_V3_2__3);
      else if (minor_version_ < 10)
        return std::unique_ptr<URParser<RTPacket>>(new URRTStateParser_V3_5__5_1);
      else if (minor_version_ >= 10)
        return std::unique_ptr<URParser<RTPacket>>(new URRTStateParser_V3_10__5_4);
      else
      {
        LOG_FATAL("UR software version %u.%u not yet supported", major_version_, minor_version_);
        std::exit(EXIT_FAILURE);
      }
    }
    else if (major_version_ == 5)
    {
      if (minor_version_ < 4)
        return std::unique_ptr<URParser<RTPacket>>(new URRTStateParser_V3_5__5_1);
      else if (minor_version_ >= 4)
      {
        return std::unique_ptr<URParser<RTPacket>>(new URRTStateParser_V3_10__5_4);
      }
      else
      {
        LOG_FATAL("UR software version %u.%u not yet supported", major_version_, minor_version_);
        std::exit(EXIT_FAILURE);
      }
    }
    else
    {
      LOG_FATAL("UR software version %u.%u not yet supported", major_version_, minor_version_);
      std::exit(EXIT_FAILURE);
    }
  }
};
