#ifndef MEMORY_OPERATOR_H
#define MEMORY_OPERATOR_H

#include "stream_consts.h"
#include <cstddef>

namespace avos
{

class MemoryOperator
{
public:
    static inline char *Offset(const void *memory, stream::FileIndexType offset)
    {
        return MemoryOperator::Object<char *>(memory, offset);
    }

    template <typename T>
    static inline T Object(char *memory)
    {
        return reinterpret_cast<T>(memory);
    }

    template <typename T>
    static inline T Object(const void *memory, stream::FileIndexType offset)
    {
        return reinterpret_cast<T>(reinterpret_cast<size_t>(memory) + offset);
    }
};

} // namespace avos

#endif
