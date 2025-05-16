#ifndef STREAM_CONSTS_H
#define STREAM_CONSTS_H

namespace avos
{
namespace stream
{

typedef unsigned long FileIndexType;
typedef unsigned int DataIndex;
typedef int BlockIndex;

static const DataIndex NopDataIndex = 0;
static const BlockIndex NopBlockIndex = -1;

}
}

#endif
