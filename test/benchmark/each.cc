/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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
 *
*/

#include <benchmark/benchmark.h>

#include <memory>

#include "gz/sim/Entity.hh"
#include "gz/sim/EntityComponentManager.hh"

#include "flecs.h"
#include "flecs/addons/cpp/flecs.hpp"
#include "gz/sim/entt.hpp"

#include "gz/sim/components/AngularVelocity.hh"
#include "gz/sim/components/Inertial.hh"
#include "gz/sim/components/LinearAcceleration.hh"
#include "gz/sim/components/LinearVelocity.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/Pose.hh"
#include "gz/sim/components/World.hh"

#ifdef BENCHMARK_INTERNAL
using Benchmark = ::benchmark::internal::Benchmark;
#else
using Benchmark = ::benchmark::Benchmark;
#endif

using namespace gz;
using namespace sim;
using namespace components;

constexpr const int kEachIterations {100};

class EntityComponentManagerFixture: public benchmark::Fixture
{
  protected: void SetUp(const ::benchmark::State &_state) override
  {
    mgr = std::make_unique<EntityComponentManager>();
    auto matchingEntityCount = _state.range(0);
    auto nonmatchingEntityCount = _state.range(1);
    this->Populate(matchingEntityCount, nonmatchingEntityCount);
  }

  protected: void Populate(int _matchingEntityCount,
                           int _nonmatchingEntityCount)
  {
    for (int i = 0; i < _matchingEntityCount; ++i)
    {
      Entity worldEntity = mgr->CreateEntity();
      mgr->CreateComponent(worldEntity, World());
      mgr->CreateComponent(worldEntity, components::Name("world_name"));
    }

    for (int i = 0; i < _nonmatchingEntityCount; ++i)
    {
      Entity worldEntity = mgr->CreateEntity();
      mgr->CreateComponent(worldEntity, components::Name("world_name"));
    }
  }

  std::unique_ptr<EntityComponentManager> mgr;
};

BENCHMARK_DEFINE_F(EntityComponentManagerFixture, EachNoCache)
(benchmark::State &_st)
{
  for (auto _ : _st)
  {
    auto matchingEntityCount = _st.range(0);
    for (int eachIter = 0; eachIter < kEachIterations; ++eachIter)
    {
      int entitiesMatched = 0;

      mgr->EachNoCache<World, components::Name>(
          [&](const Entity &, const World *, const components::Name *)->bool
          {
            entitiesMatched++;
            return true;
          });

      if (entitiesMatched != matchingEntityCount)
      {
        _st.SkipWithError("Failed to match correct number of entities");
      }
    }
  }
}

BENCHMARK_DEFINE_F(EntityComponentManagerFixture, EachCache)
(benchmark::State &_st)
{
  for (auto _ : _st)
  {
    auto matchingEntityCount = _st.range(0);
    for (int eachIter = 0; eachIter < kEachIterations; eachIter++)
    {
      int entitiesMatched = 0;

      mgr->Each<World, components::Name>(
          [&](const Entity &, const World *, const components::Name *)->bool
          {
            entitiesMatched++;
            return true;
          });

      if (entitiesMatched != matchingEntityCount)
      {
        _st.SkipWithError("Failed to match correct number of entities");
      }
    }
  }
}

class ManyComponentFixture: public benchmark::Fixture
{
  protected: void SetUp(const ::benchmark::State &_state) override
  {
    mgr = std::make_unique<EntityComponentManager>();
    auto entityCount = _state.range(0);
    this->Populate(entityCount);
  }

  protected: void Populate(int _entityCount)
  {
    for (int i = 0; i < _entityCount; ++i)
    {
      Entity entity = mgr->CreateEntity();
      mgr->CreateComponent(entity, components::Name("world_name"));
      mgr->CreateComponent(entity, AngularVelocity());
      mgr->CreateComponent(entity, WorldAngularVelocity());
      mgr->CreateComponent(entity, Inertial());
      mgr->CreateComponent(entity, LinearAcceleration());
      mgr->CreateComponent(entity, WorldLinearAcceleration());
      mgr->CreateComponent(entity, LinearVelocity());
      mgr->CreateComponent(entity, WorldLinearVelocity());
      mgr->CreateComponent(entity, Pose());
      mgr->CreateComponent(entity, WorldPose());
    }
  }

  std::unique_ptr<EntityComponentManager> mgr;
};

class FlecsManyComponentFixture: public benchmark::Fixture
{
  protected: void SetUp(const ::benchmark::State &_state) override
  {
    ecs = std::make_unique<flecs::world>();
    /*
    ecs->component<components::Name>().add(flecs::Sparse);
    ecs->component<AngularVelocity>().add(flecs::Sparse);
    ecs->component<WorldAngularVelocity>().add(flecs::Sparse);
    ecs->component<Inertial>().add(flecs::Sparse);
    ecs->component<LinearAcceleration>().add(flecs::Sparse);
    ecs->component<WorldLinearAcceleration>().add(flecs::Sparse);
    ecs->component<LinearVelocity>().add(flecs::Sparse);
    ecs->component<WorldLinearVelocity>().add(flecs::Sparse);
    ecs->component<Pose>().add(flecs::Sparse);
    ecs->component<WorldPose>().add(flecs::Sparse);
    */
    auto entityCount = _state.range(0);
    this->Populate(entityCount);
    this->q_name = ecs->query_builder<components::Name&>().cached().build();
    this->q_all = ecs->query_builder<components::Name&, AngularVelocity&, Inertial&, LinearAcceleration&, LinearVelocity&>().cached().build();
    this->q_opt = ecs->query_builder<components::Name&, AngularVelocity*, Inertial*, LinearAcceleration*, LinearVelocity*>().cached().build();
  }

  protected: void Populate(int _entityCount)
  {
    for (int i = 0; i < _entityCount; ++i)
    {
      ecs->entity()
        .insert([](components::Name& n, AngularVelocity&, WorldAngularVelocity&, Inertial&, LinearAcceleration&, WorldLinearAcceleration&, LinearVelocity&, WorldLinearVelocity&, Pose&, WorldPose&) {
              n.Data() = "world_name";
            });
    }
  }

  std::unique_ptr<flecs::world> ecs;
  flecs::query<components::Name&> q_name;
  flecs::query<components::Name&, AngularVelocity&, Inertial&, LinearAcceleration&, LinearVelocity&> q_all;
  flecs::query<components::Name&, AngularVelocity*, Inertial*, LinearAcceleration*, LinearVelocity*> q_opt;
};

class EnttManyComponentFixture: public benchmark::Fixture
{
  protected: void SetUp(const ::benchmark::State &_state) override
  {
    ecs = std::make_unique<entt::registry>();
    auto entityCount = _state.range(0);
    this->Populate(entityCount);
  }

  protected: void Populate(int _entityCount)
  {
    for (int i = 0; i < _entityCount; ++i)
    {
      const auto e = ecs->create();
      ecs->emplace<components::Name>(e, "world_name");
      ecs->emplace<AngularVelocity>(e, AngularVelocity());
      ecs->emplace<WorldAngularVelocity>(e, WorldAngularVelocity());
      ecs->emplace<Inertial>(e, Inertial());
      ecs->emplace<LinearAcceleration>(e, LinearAcceleration());
      ecs->emplace<WorldLinearAcceleration>(e, WorldLinearAcceleration());
      ecs->emplace<LinearVelocity>(e, LinearVelocity());
      ecs->emplace<WorldLinearVelocity>(e, WorldLinearVelocity());
      ecs->emplace<Pose>(e, Pose());
      ecs->emplace<WorldPose>(e, WorldPose());
    }
  }

  std::unique_ptr<entt::registry> ecs;
};

BENCHMARK_DEFINE_F(ManyComponentFixture, Each1ComponentCache)
(benchmark::State &_st)
{
  for (auto _ : _st)
  {
    auto entityCount = _st.range(0);

    for (int eachIter = 0; eachIter < kEachIterations; eachIter++)
    {
      int entitiesMatched = 0;

      mgr->Each<components::Name>(
          [&](const Entity &, const components::Name *)->bool
          {
            entitiesMatched++;
            return true;
          });

      if (entitiesMatched != entityCount)
      {
        _st.SkipWithError("Failed to match correct number of entities");
      }
    }
  }
}

BENCHMARK_DEFINE_F(ManyComponentFixture, Each5ComponentCache)
(benchmark::State &_st)
{
  for (auto _ : _st)
  {
    auto entityCount = _st.range(0);

    for (int eachIter = 0; eachIter < kEachIterations; eachIter++)
    {
      int entitiesMatched = 0;

      mgr->Each<components::Name,
                AngularVelocity,
                Inertial,
                LinearAcceleration,
                LinearVelocity>(
          [&](const Entity &,
              const components::Name *,
              const AngularVelocity *,
              const Inertial *,
              const LinearAcceleration *,
              const LinearVelocity *)->bool
          {
            entitiesMatched++;
            return true;
          });

      if (entitiesMatched != entityCount)
      {
        _st.SkipWithError("Failed to match correct number of entities");
      }
    }
  }
}

BENCHMARK_DEFINE_F(FlecsManyComponentFixture, FlecsEach5ComponentNoCache)
(benchmark::State &_st)
{
  for (auto _ : _st)
  {
    auto entityCount = _st.range(0);

    for (int eachIter = 0; eachIter < kEachIterations; eachIter++)
    {
      int entitiesMatched = 0;

      ecs->each(
          [&](const components::Name&,
              const AngularVelocity&,
              const Inertial&,
              const LinearAcceleration&,
              const LinearVelocity&)->bool
          {
            entitiesMatched++;
            return true;
          });

      if (entitiesMatched != entityCount)
      {
        _st.SkipWithError("Failed to match correct number of entities");
      }
    }
  }
}

BENCHMARK_DEFINE_F(FlecsManyComponentFixture, FlecsEach5ComponentCache)
(benchmark::State &_st)
{
  for (auto _ : _st)
  {
    auto entityCount = _st.range(0);

    for (int eachIter = 0; eachIter < kEachIterations; eachIter++)
    {
      int entitiesMatched = 0;

      q_all.each(
          [&](const components::Name&,
              const AngularVelocity&,
              const Inertial&,
              const LinearAcceleration&,
              const LinearVelocity&)->bool
          {
            entitiesMatched++;
            return true;
          });

      if (entitiesMatched != entityCount)
      {
        _st.SkipWithError("Failed to match correct number of entities");
      }
    }
  }
}

BENCHMARK_DEFINE_F(EnttManyComponentFixture, EnttEach5Component)
(benchmark::State &_st)
{
  for (auto _ : _st)
  {
    auto entityCount = _st.range(0);

    for (int eachIter = 0; eachIter < kEachIterations; eachIter++)
    {
      int entitiesMatched = 0;

      auto view = ecs->view<const components::Name, AngularVelocity, Inertial, LinearAcceleration, LinearVelocity>();

      view.each(
          [&](const components::Name&,
              const AngularVelocity&,
              const Inertial&,
              const LinearAcceleration&,
              const LinearVelocity&)->bool
          {
            entitiesMatched++;

            return true;
          });

      if (entitiesMatched != entityCount)
      {
        _st.SkipWithError("Failed to match correct number of entities");
      }
    }
  }
}

BENCHMARK_DEFINE_F(ManyComponentFixture, Each1ComponentGet4More)
(benchmark::State &_st)
{
  for (auto _ : _st)
  {
    auto entityCount = _st.range(0);

    for (int eachIter = 0; eachIter < kEachIterations; eachIter++)
    {
      int entitiesMatched = 0;

      mgr->Each<components::Name>(
          [&](const Entity &e,
              const components::Name *)->bool
          {
            if (mgr->Component<AngularVelocity>(e) != nullptr &&
                mgr->Component<Inertial>(e) != nullptr &&
                mgr->Component<LinearAcceleration>(e) != nullptr &&
                mgr->Component<LinearVelocity>(e) != nullptr) {
              entitiesMatched++;
            }

            return true;
          });

      if (entitiesMatched != entityCount)
      {
        _st.SkipWithError("Failed to match correct number of entities");
      }
    }
  }
}

BENCHMARK_DEFINE_F(FlecsManyComponentFixture, FlecsEach1ComponentGet4More)
(benchmark::State &_st)
{
  for (auto _ : _st)
  {
    auto entityCount = _st.range(0);

    for (int eachIter = 0; eachIter < kEachIterations; eachIter++)
    {
      int entitiesMatched = 0;

      q_name.each(
          [&](flecs::entity e,
              const components::Name&)->bool
          {
            if (e.try_get<AngularVelocity>() != nullptr &&
                e.try_get<Inertial>() != nullptr &&
                e.try_get<LinearAcceleration>() != nullptr &&
                e.try_get<LinearVelocity>() != nullptr) {
              entitiesMatched++;
            }

            return true;
          });

      if (entitiesMatched != entityCount)
      {
        _st.SkipWithError("Failed to match correct number of entities");
      }
    }
  }
}

BENCHMARK_DEFINE_F(FlecsManyComponentFixture, FlecsEach1ComponentGet4MoreOptionalNoCache)
(benchmark::State &_st)
{
  for (auto _ : _st)
  {
    auto entityCount = _st.range(0);

    for (int eachIter = 0; eachIter < kEachIterations; eachIter++)
    {
      int entitiesMatched = 0;

      ecs->each(
          [&](const components::Name&,
              const AngularVelocity* av,
              const Inertial* i,
              const LinearAcceleration* la,
              const LinearVelocity* lv
              )->bool
          {
            if (av != nullptr && i != nullptr && la != nullptr && lv != nullptr) {
              entitiesMatched++;
            }

            return true;
          });

      if (entitiesMatched != entityCount)
      {
        _st.SkipWithError("Failed to match correct number of entities");
      }
    }
  }
}

BENCHMARK_DEFINE_F(FlecsManyComponentFixture, FlecsEach1ComponentGet4MoreOptionalCache)
(benchmark::State &_st)
{
  for (auto _ : _st)
  {
    auto entityCount = _st.range(0);

    for (int eachIter = 0; eachIter < kEachIterations; eachIter++)
    {
      int entitiesMatched = 0;

      q_opt.each(
          [&](const components::Name&,
              const AngularVelocity* av,
              const Inertial* i,
              const LinearAcceleration* la,
              const LinearVelocity* lv
              )->bool
          {
            if (av != nullptr && i != nullptr && la != nullptr && lv != nullptr) {
              entitiesMatched++;
            }

            return true;
          });

      if (entitiesMatched != entityCount)
      {
        _st.SkipWithError("Failed to match correct number of entities");
      }
    }
  }
}

BENCHMARK_DEFINE_F(EnttManyComponentFixture, EnttEach1ComponentGet4More)
(benchmark::State &_st)
{
  for (auto _ : _st)
  {
    auto entityCount = _st.range(0);

    for (int eachIter = 0; eachIter < kEachIterations; eachIter++)
    {
      int entitiesMatched = 0;

      auto view = ecs->view<const components::Name>();

      view.each(
          [&](const auto e,
              const components::Name&)->bool
          {
            if (ecs->try_get<AngularVelocity>(e) != nullptr &&
                ecs->try_get<Inertial>(e) != nullptr &&
                ecs->try_get<LinearAcceleration>(e) != nullptr &&
                ecs->try_get<LinearVelocity>(e) != nullptr) {
              entitiesMatched++;
            }

            return true;
          });

      if (entitiesMatched != entityCount)
      {
        _st.SkipWithError("Failed to match correct number of entities");
      }
    }
  }
}

BENCHMARK_DEFINE_F(ManyComponentFixture, Each10ComponentCache)
(benchmark::State &_st)
{
  for (auto _ : _st)
  {
    auto entityCount = _st.range(0);

    for (int eachIter = 0; eachIter < kEachIterations; eachIter++)
    {
      int entitiesMatched = 0;

      mgr->Each<components::Name,
                AngularVelocity,
                WorldAngularVelocity,
                Inertial,
                LinearAcceleration,
                WorldLinearAcceleration,
                LinearVelocity,
                WorldLinearVelocity,
                Pose,
                WorldPose>(
          [&](const Entity &,
              const components::Name *,
              const AngularVelocity *,
              const WorldAngularVelocity *,
              const Inertial *,
              const LinearAcceleration *,
              const WorldLinearAcceleration *,
              const LinearVelocity *,
              const WorldLinearVelocity *,
              const Pose *,
              const WorldPose *)->bool
          {
            entitiesMatched++;
            return true;
          });

      if (entitiesMatched != entityCount)
      {
        _st.SkipWithError("Failed to match correct number of entities");
      }
    }
  }
}

BENCHMARK_DEFINE_F(ManyComponentFixture, Each1ComponentNoCache)
(benchmark::State &_st)
{
  for (auto _ : _st)
  {
    auto entityCount = _st.range(0);

    for (int eachIter = 0; eachIter < kEachIterations; eachIter++)
    {
      int entitiesMatched = 0;

      mgr->EachNoCache<components::Name>(
          [&](const Entity &, const components::Name *)->bool
          {
            entitiesMatched++;
            return true;
          });

      if (entitiesMatched != entityCount)
      {
        _st.SkipWithError("Failed to match correct number of entities");
      }
    }
  }
}

BENCHMARK_DEFINE_F(ManyComponentFixture, Each5ComponentNoCache)
(benchmark::State &_st)
{
  for (auto _ : _st)
  {
    auto entityCount = _st.range(0);

    for (int eachIter = 0; eachIter < kEachIterations; eachIter++)
    {
      int entitiesMatched = 0;

      mgr->EachNoCache<components::Name,
                AngularVelocity,
                Inertial,
                LinearAcceleration,
                LinearVelocity>(
          [&](const Entity &,
              const components::Name *,
              const AngularVelocity *,
              const Inertial *,
              const LinearAcceleration *,
              const LinearVelocity *)->bool
          {
            entitiesMatched++;
            return true;
          });

      if (entitiesMatched != entityCount)
      {
        _st.SkipWithError("Failed to match correct number of entities");
      }
    }
  }
}

/// Method to generate test argument combinations.  google/benchmark does
/// powers of 2 by default, which looks kind of ugly.
static void EachTestArgs(Benchmark *_b)
{
  int maxEntityCount = 1000;
  int step = maxEntityCount/5;

  for (int matchingEntityCount = 0; matchingEntityCount <= maxEntityCount;
       matchingEntityCount += step)
  {
    for (int nonmatchingEntityCount = 0;
         nonmatchingEntityCount <= maxEntityCount;
         nonmatchingEntityCount += step)
    {
      _b->Args({matchingEntityCount, nonmatchingEntityCount});
    }
  }
}

BENCHMARK_REGISTER_F(ManyComponentFixture, Each5ComponentNoCache)
  ->Arg(10)
  ->Arg(100)
  ->Arg(1000)
  ->Arg(10000)
  ->Unit(benchmark::kMillisecond);

BENCHMARK_REGISTER_F(FlecsManyComponentFixture, FlecsEach5ComponentNoCache)
  ->Arg(10)
  ->Arg(100)
  ->Arg(1000)
  ->Arg(10000)
  ->Unit(benchmark::kMillisecond);

BENCHMARK_REGISTER_F(EnttManyComponentFixture, EnttEach5Component)
  ->Arg(10)
  ->Arg(100)
  ->Arg(1000)
  ->Arg(10000)
  ->Unit(benchmark::kMillisecond);

BENCHMARK_REGISTER_F(ManyComponentFixture, Each5ComponentCache)
  ->Arg(10)
  ->Arg(100)
  ->Arg(1000)
  ->Arg(10000)
  ->Unit(benchmark::kMillisecond);

BENCHMARK_REGISTER_F(FlecsManyComponentFixture, FlecsEach5ComponentCache)
  ->Arg(10)
  ->Arg(100)
  ->Arg(1000)
  ->Arg(10000)
  ->Unit(benchmark::kMillisecond);

BENCHMARK_REGISTER_F(ManyComponentFixture, Each1ComponentGet4More)
  ->Arg(10)
  ->Arg(100)
  ->Arg(1000)
  ->Arg(10000)
  ->Unit(benchmark::kMillisecond);

BENCHMARK_REGISTER_F(FlecsManyComponentFixture, FlecsEach1ComponentGet4More)
  ->Arg(10)
  ->Arg(100)
  ->Arg(1000)
  ->Arg(10000)
  ->Unit(benchmark::kMillisecond);

BENCHMARK_REGISTER_F(FlecsManyComponentFixture, FlecsEach1ComponentGet4MoreOptionalNoCache)
  ->Arg(10)
  ->Arg(100)
  ->Arg(1000)
  ->Arg(10000)
  ->Unit(benchmark::kMillisecond);

BENCHMARK_REGISTER_F(FlecsManyComponentFixture, FlecsEach1ComponentGet4MoreOptionalCache)
  ->Arg(10)
  ->Arg(100)
  ->Arg(1000)
  ->Arg(10000)
  ->Unit(benchmark::kMillisecond);

BENCHMARK_REGISTER_F(EnttManyComponentFixture, EnttEach1ComponentGet4More)
  ->Arg(10)
  ->Arg(100)
  ->Arg(1000)
  ->Arg(10000)
  ->Unit(benchmark::kMillisecond);

// OSX needs the semicolon, Ubuntu complains that there's an extra ';'
#if !defined(_MSC_VER)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#endif
BENCHMARK_MAIN();
#if !defined(_MSC_VER)
#pragma GCC diagnostic pop
#endif
