#include "libcalib/protocol_bin.h"
#include "libcalib/protocol.h"

#include <string.h>

namespace libcalib
{
namespace Protocol
{
namespace Binary
{

// --- UCrc ---

UCrc::UCrc()
{
	m_w = 0xFFFF;
}

UCrc::UCrc(size_t cB, const uint8_t * pB)
{
	m_w = 0xFFFF;
	for (size_t i = 0; i < cB; ++i)
		Update(pB[i]);
}

void UCrc::Update(uint8_t b)
{
	m_w ^= b;
	for (int iBit = 0; iBit < 8; ++iBit)
	{
		if (m_w & 1)
			m_w = (m_w >> 1) ^ 0xA001;
		else
			m_w = m_w >> 1;
	}
}

// packet header signature
uint8_t g_aBHeader[] = { 0x75, 0x54 };
static_assert(sizeof(g_aBHeader) == s_cBHeader, "header array size must match s_cBHeader");

// --- CParser ---

CParser::CParser()
{
	Reset();
}

void CParser::Reset()
{
	m_pkstate = PKSTATE_Header;
	m_iB = 0;
	memset(m_aB, 0, sizeof(m_aB));
	m_cal = Mag::SCal();
	m_fReady = false;
}

bool CParser::FFeedBytes(size_t cB, const uint8_t * pB)
{
	m_fReady = false;
	for (size_t iB = 0; iB < cB; ++iB)
	{
		FeedByte(pB[iB]);
		if (m_fReady)
			return true;
	}
	return false;
}

void CParser::FeedByte(uint8_t b)
{
	switch (m_pkstate)
	{
	case PKSTATE_Header:
		if (b == g_aBHeader[m_iB])
		{
			m_aB[m_iB] = b;
			++m_iB;
			if (m_iB == 2)
				m_pkstate = PKSTATE_Body;
		}
		else
		{
			// reset: if b matches the first header byte, keep it
			if (b == g_aBHeader[0])
			{
				m_aB[0] = b;
				m_iB = 1;
			}
			else
			{
				m_iB = 0;
			}
		}
		break;

	case PKSTATE_Body:
		m_aB[m_iB] = b;
		++m_iB;
		if (m_iB == s_cBPacket)
		{
			m_fReady = FTryParseCalibration();
			m_pkstate = PKSTATE_Header;
			m_iB = 0;
		}
		break;

	default:
		m_pkstate = PKSTATE_Header;
		m_iB = 0;
		break;
	}
}

bool CParser::FTryParseCalibration()
{
	// validate CRC over all 68 bytes — correct residual is 0x0000
	UCrc crc(s_cBPacket, m_aB);
	if (crc.m_w != 0x0000)
		return false;

	// extract floats from packet body
	float aG[IG_Max];
	memcpy(aG, &m_aB[s_cBHeader], sizeof(aG));

	m_cal.m_vecV.x = aG[IG_MagX];
	m_cal.m_vecV.y = aG[IG_MagY];
	m_cal.m_vecV.z = aG[IG_MagZ];
	m_cal.m_sB     = aG[IG_FieldB];

	m_cal.m_matWInv.vecX.x = aG[IG_SoftXX];
	m_cal.m_matWInv.vecX.y = aG[IG_SoftXY];
	m_cal.m_matWInv.vecX.z = aG[IG_SoftXZ];
	m_cal.m_matWInv.vecY.x = aG[IG_SoftXY];	// symmetric
	m_cal.m_matWInv.vecY.y = aG[IG_SoftYY];
	m_cal.m_matWInv.vecY.z = aG[IG_SoftYZ];
	m_cal.m_matWInv.vecZ.x = aG[IG_SoftXZ];	// symmetric
	m_cal.m_matWInv.vecZ.y = aG[IG_SoftYZ];	// symmetric
	m_cal.m_matWInv.vecZ.z = aG[IG_SoftZZ];

	return true;
}

// --- emit function ---

void EmitCalibration(IWriter * pWriter, const Mag::SCal & cal)
{
	if (pWriter == nullptr)
		return;

	uint8_t aB[s_cBPacket];

	// header
	aB[0] = g_aBHeader[0];
	aB[1] = g_aBHeader[1];

	// assemble floats at correct offsets
	float aG[IG_Max];

	aG[IG_AccelX] = 0.0f;				// not yet implemented
	aG[IG_AccelY] = 0.0f;
	aG[IG_AccelZ] = 0.0f;
	aG[IG_GyroX]  = 0.0f;				// not yet implemented
	aG[IG_GyroY]  = 0.0f;
	aG[IG_GyroZ]  = 0.0f;
	aG[IG_MagX]   = cal.m_vecV.x;
	aG[IG_MagY]   = cal.m_vecV.y;
	aG[IG_MagZ]   = cal.m_vecV.z;
	aG[IG_FieldB] = cal.m_sB;
	aG[IG_SoftXX] = cal.m_matWInv.vecX.x;
	aG[IG_SoftYY] = cal.m_matWInv.vecY.y;
	aG[IG_SoftZZ] = cal.m_matWInv.vecZ.z;
	aG[IG_SoftXY] = cal.m_matWInv.vecX.y;
	aG[IG_SoftXZ] = cal.m_matWInv.vecX.z;
	aG[IG_SoftYZ] = cal.m_matWInv.vecY.z;

	memcpy(&aB[s_cBHeader], aG, sizeof(aG));

	// CRC-16 over header + body, append as little-endian
	UCrc crc(s_cBHeader + s_cBBody, aB);
	aB[s_cBHeader + s_cBBody]     = crc.m_aB[0];
	aB[s_cBHeader + s_cBBody + 1] = crc.m_aB[1];

	pWriter->Write(s_cBPacket, aB);
}

} // namespace Binary
} // namespace Protocol
} // namespace libcalib
