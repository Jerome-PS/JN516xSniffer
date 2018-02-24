#ifndef __utils_h__
#define __utils_h__

// IEEE 802.15.4 is litte endian, so work in little endian
#if BYTE_ORDER == BIG_ENDIAN
#define cpu_to_le32(x)	__builtin_bswap32((x))
#define cpu_to_le16(x)	((((x) & 0xFF) << 8) | (((x) >> 8) & 0xFF))
#else
#define cpu_to_le32(x)	(x)
#define cpu_to_le16(x)	(x)
#endif

#define min(a,b)	({__typeof__(a) _a=(a); __typeof__(b) _b=(b); _a<_b?_a:_b;})
#define isdigit(x)	({__typeof__(x) _x=(x);(_x>='0') && (_x<='9');})

#define READ_REG32(A)     *(volatile uint32 *)(A)

#endif	//ndef __utils_h__

