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
#ifndef GZ_SIM_DETAIL_ENTITYCOMPONENTMANAGER_HH_
#define GZ_SIM_DETAIL_ENTITYCOMPONENTMANAGER_HH_

#include <cstring>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>

// TODO(luca) move to a detail folder?
#ifdef emit
  // Conflict because qt also defines emit
  #pragma push_macro("emit")
  #undef emit
  #include <flecs.h>
  #pragma pop_macro("emit")
#else
  #include <flecs.h>
#endif
#include <gz/math/Helpers.hh>

#include "gz/sim/components/Factory.hh"
#include "gz/sim/EntityComponentManager.hh"

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE {
//////////////////////////////////////////////////
namespace traits
{
  /// \brief Helper struct to determine if an equality operator is present.
  struct TestEqualityOperator
  {
  };
  template<typename T>
  TestEqualityOperator operator == (const T&, const T&);

  /// \brief Type trait that determines if an operator== is defined for `T`.
  template<typename T>
  struct HasEqualityOperator
  {
#if !defined(_MSC_VER)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wnonnull"
#endif
    enum
    {
      // False positive codecheck "Using C-style cast"
      value = !std::is_same<decltype(*(T*)(0) == *(T*)(0)), TestEqualityOperator>::value // NOLINT
    };
#if !defined(_MSC_VER)
#pragma GCC diagnostic pop
#endif
  };
}

//////////////////////////////////////////////////
/// \brief Helper function to compare two objects of the same type using its
/// equality operator.
/// If `DataType` doesn't have an equality operator defined, it will return
/// false.
/// For doubles, `gz::math::equal` will be used.
template<typename DataType>
auto CompareData = [](const DataType &_a, const DataType &_b) -> bool
{
  // cppcheck-suppress syntaxError
  if constexpr (std::is_same<DataType, double>::value)
  {
    return math::equal(_a, _b);
  }
  else if constexpr (traits::HasEqualityOperator<DataType>::value)
  {
    return _a == _b;
  }

  return false;
};

//////////////////////////////////////////////////
template<typename ComponentTypeT>
ComponentTypeT *EntityComponentManager::CreateComponent(const Entity _entity,
            const ComponentTypeT &_data)
{
  if (!this->HasEntity(_entity))
    return nullptr;
  flecs::entity e = this->world.entity(_entity + this->EntityOffset());
  e.set<ComponentTypeT>(_data);
  this->SetChanged(_entity, ComponentTypeT::typeId, ComponentState::OneTimeChange);
  this->MarkComponentAsRemoved(_entity, ComponentTypeT::typeId, false);
  return e.try_get_mut<ComponentTypeT>();
}

//////////////////////////////////////////////////
template<typename ComponentTypeT>
const ComponentTypeT *EntityComponentManager::Component(
    const Entity _entity) const
{
  std::lock_guard<std::recursive_mutex> lock(this->flecsWorldMutex);
  if (!this->HasEntity(_entity))
    return nullptr;
  flecs::entity e = this->world.entity(_entity + this->EntityOffset());
  return e.try_get<ComponentTypeT>();
}

//////////////////////////////////////////////////
template<typename ComponentTypeT>
ComponentTypeT *EntityComponentManager::Component(const Entity _entity)
{
  std::lock_guard<std::recursive_mutex> lock(this->flecsWorldMutex);
  if (!this->HasEntity(_entity))
    return nullptr;
  flecs::entity e = this->world.entity(_entity + this->EntityOffset());
  return e.try_get_mut<ComponentTypeT>();
}

//////////////////////////////////////////////////
template<typename ComponentTypeT>
ComponentTypeT *EntityComponentManager::ComponentDefault(Entity _entity,
    const typename ComponentTypeT::Type &_default)
{
  auto comp = this->Component<ComponentTypeT>(_entity);
  if (!comp)
  {
    this->CreateComponent(_entity, ComponentTypeT(_default));
    comp = this->Component<ComponentTypeT>(_entity);
  }
  return comp;
}

//////////////////////////////////////////////////
template<typename ComponentTypeT>
std::optional<typename ComponentTypeT::Type>
    EntityComponentManager::ComponentData(const Entity _entity) const
{
  auto comp = this->Component<ComponentTypeT>(_entity);
  if (!comp)
    return std::nullopt;

  return std::make_optional(comp->Data());
}

//////////////////////////////////////////////////
template<typename ComponentTypeT>
bool EntityComponentManager::SetComponentData(const Entity _entity,
    const typename ComponentTypeT::Type &_data)
{
  auto comp = this->Component<ComponentTypeT>(_entity);

  if (nullptr == comp)
  {
    this->CreateComponent(_entity, ComponentTypeT(_data));
    return true;
  }

  return comp->SetData(_data, CompareData<typename ComponentTypeT::Type>);
}

//////////////////////////////////////////////////
template<typename ...ComponentTypeTs>
Entity EntityComponentManager::EntityByComponents(
    const ComponentTypeTs &..._desiredComponents) const
{
  std::lock_guard<std::recursive_mutex> lock(this->flecsWorldMutex);
  auto key = detail::ComponentTypeKey{ComponentTypeTs::typeId...};
  const flecs::query_t* q_ptr = this->QueryPtr(key);
  if (q_ptr == nullptr)
  {
    flecs::query<const ComponentTypeTs...> q = this->world.query<const ComponentTypeTs...>();
    this->SetQueryPtr(key, q);
    q_ptr = q.c_ptr();
  }

  flecs::query<const ComponentTypeTs...> q(const_cast<flecs::query_t*>(q_ptr));
  flecs::entity result = q.find([&](const ComponentTypeTs&... actualComponents) {
    return ((actualComponents == _desiredComponents) && ...);
  });
  if (result)
    return result.id() - this->EntityOffset();
  return kNullEntity;
}

//////////////////////////////////////////////////
template<typename ...ComponentTypeTs>
std::vector<Entity> EntityComponentManager::EntitiesByComponents(
    const ComponentTypeTs &..._desiredComponents) const
{
  std::lock_guard<std::recursive_mutex> lock(this->flecsWorldMutex);
  std::vector<Entity> result;
  auto key = detail::ComponentTypeKey{ComponentTypeTs::typeId...};
  const flecs::query_t* q_ptr = this->QueryPtr(key);
  if (q_ptr == nullptr)
  {
    flecs::query<const ComponentTypeTs...> q = this->world.query<const ComponentTypeTs...>();
    this->SetQueryPtr(key, q);
    q_ptr = q.c_ptr();
  }

  flecs::query<const ComponentTypeTs...> q(const_cast<flecs::query_t*>(q_ptr));
  const auto offset = this->EntityOffset();
  q.each([&](flecs::entity e, const ComponentTypeTs&... actualComponents) {
    if (((actualComponents == _desiredComponents) && ...))
    {
      result.push_back(e.id() - offset);
    }
  });
  // TODO(luca) we shouldn't need to do this if we can iterate over children of the parent
  // since we use the flecs::OrderedChildren trait which should iterate children in the order of adding
  // std::sort(result.begin(), result.end());
  return result;
}

//////////////////////////////////////////////////
template<typename ...ComponentTypeTs>
std::vector<Entity> EntityComponentManager::ChildrenByComponents(Entity _parent,
     const ComponentTypeTs &..._desiredComponents) const
{
  std::lock_guard<std::recursive_mutex> lock(this->flecsWorldMutex);
  std::vector<Entity> result;
  const auto offset = this->EntityOffset();
  flecs::entity p = this->world.entity(_parent + offset);

  p.children([&](flecs::entity child) {
    bool match = true;
    ([&]{
      if (!match) return;
      const ComponentTypeTs* comp = child.try_get<ComponentTypeTs>();
      if (!comp || !(*comp == _desiredComponents)) {
        match = false;
      }
    }(), ...);

    if (match) {
      result.push_back(child.id() - offset);
    }
  });

  // TODO(luca) we shouldn't need to do this if we can iterate over children of the parent
  // since we use the flecs::OrderedChildren trait which should iterate children in the order of adding
  // std::sort(result.begin(), result.end());
  return result;
}

//////////////////////////////////////////////////
template <typename T>
struct EntityComponentManager::identity  // NOLINT
{
  using type = T;
};

//////////////////////////////////////////////////
template<typename ...ComponentTypeTs, typename Func>
void EntityComponentManager::EachNoCache(Func &&_f) const
{
  // This is now functionally equivalent since cached queries are always automatically
  // updated and we don't need an explicit "rebuild cache" call
  this->Each<ComponentTypeTs...>(std::forward<Func>(_f));
}

//////////////////////////////////////////////////
template<typename ...ComponentTypeTs, typename Func>
void EntityComponentManager::EachNoCache(Func &&_f)
{
  // This is now functionally equivalent since cached queries are always automatically
  // updated and we don't need an explicit "rebuild cache" call
  this->Each<ComponentTypeTs...>(std::forward<Func>(_f));
}

//////////////////////////////////////////////////
template<typename ...ComponentTypeTs, typename Func>
void EntityComponentManager::Each(Func &&_f) const
{
  std::lock_guard<std::recursive_mutex> lock(this->flecsWorldMutex);
  auto key = detail::ComponentTypeKey{ComponentTypeTs::typeId...};
  const flecs::query_t* q_ptr = this->QueryPtr(key);
  if (q_ptr == nullptr)
  {
    flecs::query<const ComponentTypeTs...> q = this->world.query_builder<const ComponentTypeTs...>()
        .cached()
        .build();
    this->SetQueryPtr(key, q);
    q_ptr = q.c_ptr();
  }
  flecs::query<const ComponentTypeTs...> q(const_cast<flecs::query_t*>(q_ptr));
  const auto offset = this->EntityOffset();
  q.find([&](flecs::entity e, const ComponentTypeTs&... comps) {
    return !_f(e.id() - offset, &comps...);
  });
}

// This is run in the benchmark
//////////////////////////////////////////////////
template<typename ...ComponentTypeTs, typename Func>
void EntityComponentManager::Each(Func &&_f)
{
  std::lock_guard<std::recursive_mutex> lock(this->flecsWorldMutex);
  // If it is not deferred we need to apply it ourselves
  const bool applyDefer = !this->IsDeferred();
  auto key = detail::ComponentTypeKey{ComponentTypeTs::typeId...};
  const flecs::query_t* q_ptr = this->QueryPtr(key);
  if (q_ptr == nullptr)
  {
    flecs::query<ComponentTypeTs...> q = this->world.query_builder<ComponentTypeTs...>()
        .build();
    this->SetQueryPtr(key, q);
    q_ptr = q.c_ptr();
  }

  flecs::query<ComponentTypeTs...> q(const_cast<flecs::query_t*>(q_ptr));
  const auto offset = this->EntityOffset();
  if (applyDefer) this->DeferBegin();
  q.find([&](flecs::entity e, ComponentTypeTs&... comps) {
    return !_f(e.id() - offset, &comps...);
  });
  if (applyDefer) this->DeferEnd();
}

//////////////////////////////////////////////////
template <class Function, class... ComponentTypeTs>
void EntityComponentManager::ForEach(Function _f,
    const ComponentTypeTs &... _components)
{
  (void)_f;
  (_f(_components), ...);
}

//////////////////////////////////////////////////
template <typename... ComponentTypeTs, typename Func>
void EntityComponentManager::EachNew(Func &&_f)
{
  std::lock_guard<std::recursive_mutex> lock(this->flecsWorldMutex);
  const bool applyDefer = !this->IsDeferred();
  flecs::query<ComponentTypeTs...> q = this->world.query_builder<ComponentTypeTs...>().
    template with<NewEntity>().
    cached().
    build();
  const auto offset = this->EntityOffset();
  if (applyDefer) this->DeferBegin();
  q.find([&](flecs::entity e, ComponentTypeTs&... comps) {
    return !_f(e.id() - offset, &comps...);
  });
  if (applyDefer) this->DeferEnd();
}

//////////////////////////////////////////////////
template <typename... ComponentTypeTs, typename Func>
void EntityComponentManager::EachNew(Func &&_f) const
{
  std::lock_guard<std::recursive_mutex> lock(this->flecsWorldMutex);
  flecs::query<const ComponentTypeTs...> q = this->world.query_builder<const ComponentTypeTs...>().
    template with<NewEntity>()
    .build();
  const auto offset = this->EntityOffset();
  q.find([&](flecs::entity e, const ComponentTypeTs&... comps) {
    return !_f(e.id() - offset, &comps...);
  });
}

//////////////////////////////////////////////////
template<typename ...ComponentTypeTs, typename Func>
void EntityComponentManager::EachRemoved(Func &&_f) const
{
  std::lock_guard<std::recursive_mutex> lock(this->flecsWorldMutex);
  flecs::query<const ComponentTypeTs...> q = this->world.query_builder<const ComponentTypeTs...>().
    template with<RemoveEntity>()
    .build();
  const auto offset = this->EntityOffset();
  q.find([&](flecs::entity e, const ComponentTypeTs&... comps) {
    return !_f(e.id() - offset, &comps...);
  });
}

//////////////////////////////////////////////////
template<typename ComponentTypeT>
bool EntityComponentManager::RemoveComponent(Entity _entity)
{
  if (!this->HasEntity(_entity))
    return false;
  flecs::entity e = this->world.entity(_entity + this->EntityOffset());
  if (!e.has<ComponentTypeT>())
    return false;
  e.remove<ComponentTypeT>();
  this->PostRemoveComponent(_entity, ComponentTypeT::typeId);
  return true;
}
}
}
}

#endif

