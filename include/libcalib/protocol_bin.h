#pragma once

#include <stddef.h>
#include <stdint.h>

#include "libcalib/common.h"

namespace libcalib
{
namespace Protocol
{

struct IWriter;

namespace Binary
{

// --- packet structure constants ---

static const int s_cBHeader = 2;								// 2-byte signature
static const int s_cBCrc = 2;									// CRC-16 little-endian

extern uint8_t g_aBHeader[];									// { 0x75, 0x54 }

// --- float index enum for calibration packet body ---

enum IG		// index into packet float array
{
	IG_AccelX,	IG_AccelY,	IG_AccelZ,
	IG_GyroX,	IG_GyroY,	IG_GyroZ,
	IG_MagX,	IG_MagY,	IG_MagZ,
	IG_FieldB,
	IG_SoftXX,	IG_SoftYY,	IG_SoftZZ,
	IG_SoftXY,	IG_SoftXZ,	IG_SoftYZ,

	IG_Max
};

static const int s_cBBody = IG_Max * sizeof(float);				// float payload
static const int s_cBPacket = s_cBHeader + s_cBBody + s_cBCrc;	// total packet size
static_assert(s_cBPacket == 68, "packet size must match MotionCal 68-byte format");

// --- CRC-16/IBM (ARC): poly 0xA001, init 0xFFFF, reflected ---

union UCrc
{
	uint16_t	m_w;
	uint8_t		m_aB[s_cBCrc];	// [0] = low, [1] = high

				UCrc();								// init to 0xFFFF
				UCrc(size_t cB, const uint8_t * pB);	// compute from buffer
	void		Update(uint8_t b);					// feed one byte
};

// --- CParser — binary calibration packet parser (68-byte MotionCal packets) ---

class CParser	// tag = binp
{
public:
			CParser();
	void	Reset();

	// Feed bytes. Returns true when a complete valid packet is parsed.
	bool	FFeedBytes(size_t cB, const uint8_t * pB);

	// Accessor (valid after FFeedBytes returns true)
	const Mag::SCal &	Cal() const		{ return m_cal; }

private:
	void	FeedByte(uint8_t b);
	bool	FTryParseCalibration();

	enum PKSTATE
	{
		PKSTATE_Header,
		PKSTATE_Body,

		PKSTATE_Max,
		PKSTATE_Nil = -1
	};

	PKSTATE		m_pkstate;
	int			m_iB;
	uint8_t		m_aB[s_cBPacket];
	Mag::SCal	m_cal;
	bool		m_fReady;
};

// --- emit function (inverse of parsing) ---

// Emit 68-byte binary calibration packet (VER_MotionCal wire format)

void EmitCalibration(IWriter * pWriter, const Mag::SCal & cal);

} // namespace Binary
} // namespace Protocol
} // namespace libcalib
