#ifndef __ACU_SOCKET_PROTOCOL_HEADER_H__
#define __ACU_SOCKET_PROTOCOL_HEADER_H__

namespace acu {
namespace common {
typedef struct ProtocolHeader
{
	unsigned char magic_num_1;
	unsigned char magic_num_2;
	unsigned char magic_num_3;
	unsigned char magic_num_4;

	bool IsHeaderProtocol() {
		if (magic_num_1 != 0x11 || magic_num_2 != 0x22 || magic_num_3 != 0x33 || magic_num_4 != 0x44) {
			return false;
		}
		return true;
	}
	ProtocolHeader() {
		magic_num_1 = 0x11;
		magic_num_2 = 0x22;
		magic_num_3 = 0x33;
		magic_num_4 = 0x44;
	}

} ProtocolHeader;

typedef struct ProtocolTail
{
	unsigned char magic_num_1;
	unsigned char magic_num_2;
	unsigned char magic_num_3;
	unsigned char magic_num_4;

	bool IsTailProtocol() {
		if (magic_num_1 != 0xaa || magic_num_2 != 0xbb || magic_num_3 != 0xcc || magic_num_4 != 0xdd) {
			return false;
		}
		return true;
	}
	ProtocolTail() {
		magic_num_1 = 0xaa;
		magic_num_2 = 0xbb;
		magic_num_3 = 0xcc;
		magic_num_4 = 0xdd;
	}

} ProtocolTail;


typedef char* pchar;
}
}

#endif