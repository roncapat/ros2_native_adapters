#pragma once
#include <memory>
#define DEFINE_MSG_POINTERS(C) \
using RawPtr = C *; \
using ConstRawPtr = const C *; \
using SharedPtr = std::shared_ptr<C>; \
using ConstSharedPtr = std::shared_ptr<const C>; \
\
template<typename Deleter = std::default_delete<C>> \
using UniquePtrWithDeleter = std::unique_ptr<C, Deleter>; \
using UniquePtr = UniquePtrWithDeleter<>; \
\
template<typename Deleter = std::default_delete<C>> \
using ConstUniquePtrWithDeleter = std::unique_ptr<C const, Deleter>; \
using ConstUniquePtr = ConstUniquePtrWithDeleter<>; \
\
using WeakPtr = std::weak_ptr<C>; \
using ConstWeakPtr = std::weak_ptr<C const>;
