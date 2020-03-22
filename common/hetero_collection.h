#pragma once

#include <functional>
#include <tuple>
#include <vector>

namespace mc {

template <typename ParentType, typename CollectionBaseType, typename... CollectionTypeList>
class HeteroCollection : public ParentType
{
 private:
    // const pointer to a pointer, we want to prevent changing
    // the link between the member variables and this tuple
    // TODO rpottorff: I believe we could replace this tuple->array->vector
    // with a constructed vector directly
    std::tuple<const CollectionTypeList&...> __collection_tuple;

 protected:
    HeteroCollection(const CollectionTypeList&... _collection_template_iterate)
        : __collection_tuple(std::forward_as_tuple(_collection_template_iterate...))
    {
    }

    template <std::size_t... I>
    const std::array<std::reference_wrapper<const CollectionBaseType>, sizeof...(I)> __impl(
        std::index_sequence<I...>) const
    {
        return {static_cast<const CollectionBaseType&>(std::get<I>(__collection_tuple))...};
    }

 public:
    template <std::size_t i>
    typename std::tuple_element<i, decltype(__collection_tuple)>::type& get() const
    {
        return std::get<i>(__collection_tuple);
    }

    template <std::size_t i>
    const CollectionBaseType* get_base() const
    {
        return static_cast<const CollectionBaseType*>(&std::get<i>(__collection_tuple));
    }

    const std::vector<std::reference_wrapper<const CollectionBaseType>> base_items() const override
    {
        const auto array = __impl(std::make_index_sequence<sizeof...(CollectionTypeList)>{});
        return std::vector<std::reference_wrapper<const CollectionBaseType>>(array.begin(),
                                                                             array.end());
    }
};

}  // namespace mc
