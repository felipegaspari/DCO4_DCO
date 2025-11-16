#ifndef PARAM_ROUTER_H
#define PARAM_ROUTER_H

// Generic parameter routing helper.
//
// Each MCU defines a ParamDescriptorT<ValueT> table where:
//   - id    is a ParamId from params_def.h
//   - apply is a function that takes the decoded parameter value
//
// param_router_apply() then looks up the incoming ID in that table
// and calls the appropriate apply() function.

template<typename ValueT>
struct ParamDescriptorT {
  ParamId id;
  void (*apply)(ValueT value);
};

template<typename ValueT>
inline void param_router_apply(
    const ParamDescriptorT<ValueT>* table,
    size_t tableSize,
    uint16_t rawId,
    ValueT value)
{
  ParamId id = static_cast<ParamId>(rawId);
  for (size_t i = 0; i < tableSize; ++i) {
    if (table[i].id == id) {
      table[i].apply(value);
      return;
    }
  }
}

#endif  // PARAM_ROUTER_H


