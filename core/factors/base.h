#pragma once

#include "common/hetero_collection.h"
#include "core/states/states.h"

namespace mc {

namespace factors {

class Factor
{
 public:
    virtual const std::vector<std::reference_wrapper<const states::State>> base_items() const = 0;
};

template <typename FactorType, typename... CollectionTypeList>
class FactorTemplate : public HeteroCollection<Factor, states::State, CollectionTypeList...>
{
 private:
    // This private constructor prevents copy/paste errors where you define class B : public
    // FactorTemplate<B... accidentally
    FactorTemplate(){};
    friend FactorType;

 protected:
    FactorTemplate(const CollectionTypeList&... _collection_template_iterate)
        : HeteroCollection<Factor, states::State, CollectionTypeList...>(
              _collection_template_iterate...)
    {
    }

    // Future CRTP stuff
};

}  // namespace factors

}  // namespace mc
