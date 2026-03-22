#include "libcalib/protocol.h"

#include <string.h>
#include <stdio.h>

namespace libcalib
{
namespace Protocol
{

// --- CRC-16/IBM (ARC): poly 0xA001, init 0xFFFF, reflected ---

uint16_t Crc16Update(uint16_t crc, uint8_t b)
{
	crc ^= b;
	for (int iBit = 0; iBit < 8; ++iBit)
	{
		if (crc & 1)
			crc = (crc >> 1) ^ 0xA001;
		else
			crc = crc >> 1;
	}
	return crc;
}

UCrc CrcFromBuffer(const uint8_t * pB, int cB)
{
	UCrc crc;
	crc.m_w = 0xFFFF;
	for (int i = 0; i < cB; ++i)
		crc.m_w = Crc16Update(crc.m_w, pB[i]);
	return crc;
}

// packet header signature
uint8_t g_aBHeader[2] = { 0x75, 0x54 };

// --- CPacketParser ---

CPacketParser::CPacketParser()
{
	Reset();
}

void CPacketParser::Reset()
{
	m_pkstate = PKSTATE_Header;
	m_iB = 0;
	memset(m_aB, 0, sizeof(m_aB));
	m_cal = Mag::SCal();
	m_fReady = false;
}

bool CPacketParser::FFeedBytes(const uint8_t * pB, int cB)
{
	m_fReady = false;
	for (int iB = 0; iB < cB; ++iB)
	{
		FeedByte(pB[iB]);
		if (m_fReady)
			return true;
	}
	return false;
}

void CPacketParser::FeedByte(uint8_t b)
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
			m_fReady = FParsePacket();
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

bool CPacketParser::FParsePacket()
{
	// validate CRC over all 68 bytes — correct residual is 0x0000
	UCrc crc = CrcFromBuffer(m_aB, s_cBPacket);
	if (crc.m_w != 0x0000)
		return false;

	// extract 16 floats from packet body (bytes 2..65)
	static const int s_cG = 16;
	float aG[s_cG];
	memcpy(aG, &m_aB[2], s_cG * sizeof(float));

	// aG[0..2]  = accel zero-g offsets (unused for now)
	// aG[3..5]  = gyro zero-rate offsets (unused for now)
	// aG[6..8]  = mag hard iron xyz
	// aG[9]     = mag field strength
	// aG[10]    = soft iron XX
	// aG[11]    = soft iron YY
	// aG[12]    = soft iron ZZ
	// aG[13]    = soft iron XY (= YX)
	// aG[14]    = soft iron XZ (= ZX)
	// aG[15]    = soft iron YZ (= ZY)

	m_cal.m_vecV.x = aG[6];
	m_cal.m_vecV.y = aG[7];
	m_cal.m_vecV.z = aG[8];
	m_cal.m_sB     = aG[9];

	m_cal.m_matWInv.vecX.x = aG[10];	// XX
	m_cal.m_matWInv.vecX.y = aG[13];	// XY
	m_cal.m_matWInv.vecX.z = aG[14];	// XZ
	m_cal.m_matWInv.vecY.x = aG[13];	// YX = XY (symmetric)
	m_cal.m_matWInv.vecY.y = aG[11];	// YY
	m_cal.m_matWInv.vecY.z = aG[15];	// YZ
	m_cal.m_matWInv.vecZ.x = aG[14];	// ZX = XZ (symmetric)
	m_cal.m_matWInv.vecZ.y = aG[15];	// ZY = YZ (symmetric)
	m_cal.m_matWInv.vecZ.z = aG[12];	// ZZ

	return true;
}

// --- CManager ---

CManager::CManager(VER ver)
: m_ver(ver),
  m_pWriter(nullptr),
  m_pReader(nullptr),
  m_pReceiver(nullptr),
  m_linep(),
  m_calPending(),
  m_fHasCal1(false),
  m_packetp()
{
}

void CManager::Init(IWriter * pWriter, IReader * pReader, IReceiver * pReceiver)
{
	m_pWriter = pWriter;
	m_pReader = pReader;
	m_pReceiver = pReceiver;
}

// --- Update: drain reader, feed ver-appropriate parser ---

void CManager::Update()
{
	if (m_pReader == nullptr)
		return;

	uint8_t aBuf[256];
	for (;;)
	{
		int cB = m_pReader->CbRead(aBuf, static_cast<int>(sizeof(aBuf)));
		if (cB <= 0)
			break;

		if (m_ver == VER_MotionCal)
		{
			// host receives text lines from device
			CLineParser::LINETYPE lt = m_linep.LinetypeFeedBytes(aBuf, cB);
			if (lt != CLineParser::LINETYPE_None)
				DispatchLine(lt);
		}
		else if (m_ver == VER_Imucal)
		{
			// device receives binary calibration packets from host
			if (m_packetp.FFeedBytes(aBuf, cB))
			{
				if (m_pReceiver != nullptr)
					m_pReceiver->OnMagCal(m_packetp.Cal());
			}
		}
	}
}

// --- line parse dispatch ---

void CManager::DispatchLine(CLineParser::LINETYPE lt)
{
	if (m_pReceiver == nullptr)
		return;

	switch (lt)
	{
	case CLineParser::LINETYPE_Raw:
	case CLineParser::LINETYPE_Uni:
		m_pReceiver->OnSample(m_linep.Samp());
		break;

	case CLineParser::LINETYPE_Cal1:
		{
			const float * pG = m_linep.PGCal();
			// pG[0..2] = accel offsets (unused for now)
			// pG[3..5] = gyro offsets (unused for now)
			// pG[6..8] = mag hard iron xyz
			// pG[9]    = field strength
			m_calPending = Mag::SCal();
			m_calPending.m_vecV.x = pG[6];
			m_calPending.m_vecV.y = pG[7];
			m_calPending.m_vecV.z = pG[8];
			m_calPending.m_sB     = pG[9];
			m_fHasCal1 = true;
		}
		break;

	case CLineParser::LINETYPE_Cal2:
		{
			if (!m_fHasCal1)
				break;

			const float * pG = m_linep.PGCal();
			// Cal2: row-major flat: XX, XY, XZ, YX, YY, YZ, ZX, ZY, ZZ
			m_calPending.m_matWInv.vecX.x = pG[0];
			m_calPending.m_matWInv.vecX.y = pG[1];
			m_calPending.m_matWInv.vecX.z = pG[2];
			m_calPending.m_matWInv.vecY.x = pG[3];
			m_calPending.m_matWInv.vecY.y = pG[4];
			m_calPending.m_matWInv.vecY.z = pG[5];
			m_calPending.m_matWInv.vecZ.x = pG[6];
			m_calPending.m_matWInv.vecZ.y = pG[7];
			m_calPending.m_matWInv.vecZ.z = pG[8];

			m_fHasCal1 = false;
			m_pReceiver->OnMagCal(m_calPending);
		}
		break;

	default:
		break;
	}
}

// --- SendSample (VER_Imucal only) ---

void CManager::SendSample(const SSample & samp)
{
	if (m_ver != VER_Imucal || m_pWriter == nullptr)
		return;

	char aBuf[256];

	// Raw: line — int16 encoded values
	int16_t ax = static_cast<int16_t>(samp.m_pntAccel.x * 8192.0f);
	int16_t ay = static_cast<int16_t>(samp.m_pntAccel.y * 8192.0f);
	int16_t az = static_cast<int16_t>(samp.m_pntAccel.z * 8192.0f);
	int16_t gx = static_cast<int16_t>(samp.m_pntGyro.x * 16.0f);
	int16_t gy = static_cast<int16_t>(samp.m_pntGyro.y * 16.0f);
	int16_t gz = static_cast<int16_t>(samp.m_pntGyro.z * 16.0f);
	int16_t mx = static_cast<int16_t>(samp.m_pntMag.x * 10.0f);
	int16_t my = static_cast<int16_t>(samp.m_pntMag.y * 10.0f);
	int16_t mz = static_cast<int16_t>(samp.m_pntMag.z * 10.0f);

	int cCh = snprintf(aBuf, sizeof(aBuf),
		"Raw:%d,%d,%d,%d,%d,%d,%d,%d,%d\r\n",
		ax, ay, az, gx, gy, gz, mx, my, mz);
	m_pWriter->Write(reinterpret_cast<const uint8_t *>(aBuf), cCh);

	// Uni: line — SI units
	cCh = snprintf(aBuf, sizeof(aBuf),
		"Uni:%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\r\n",
		MPerSecSqFromG(samp.m_pntAccel.x),
		MPerSecSqFromG(samp.m_pntAccel.y),
		MPerSecSqFromG(samp.m_pntAccel.z),
		RadFromDeg(samp.m_pntGyro.x),
		RadFromDeg(samp.m_pntGyro.y),
		RadFromDeg(samp.m_pntGyro.z),
		samp.m_pntMag.x,
		samp.m_pntMag.y,
		samp.m_pntMag.z);
	m_pWriter->Write(reinterpret_cast<const uint8_t *>(aBuf), cCh);
}

// --- SendMagCal ---

void CManager::SendMagCal(const Mag::SCal & cal)
{
	if (m_pWriter == nullptr)
		return;

	if (m_ver == VER_MotionCal)
	{
		SendBinaryPacket(cal);
		return;
	}

	if (m_ver != VER_Imucal)
		return;

	char aBuf[256];

	// Cal1: accel(0,0,0), gyro(0,0,0), hard iron xyz, field strength
	int cCh = snprintf(aBuf, sizeof(aBuf),
		"Cal1:%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\r\n",
		0.0f, 0.0f, 0.0f,	// accel offsets (not yet implemented)
		0.0f, 0.0f, 0.0f,	// gyro offsets (not yet implemented)
		cal.m_vecV.x, cal.m_vecV.y, cal.m_vecV.z,
		cal.m_sB);
	m_pWriter->Write(reinterpret_cast<const uint8_t *>(aBuf), cCh);

	// Cal2: flat row-major soft iron 3x3 (XX, XY, XZ, YX, YY, YZ, ZX, ZY, ZZ)
	const SMatrix3 & w = cal.m_matWInv;
	cCh = snprintf(aBuf, sizeof(aBuf),
		"Cal2:%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\r\n",
		w.vecX.x, w.vecX.y, w.vecX.z,
		w.vecY.x, w.vecY.y, w.vecY.z,
		w.vecZ.x, w.vecZ.y, w.vecZ.z);
	m_pWriter->Write(reinterpret_cast<const uint8_t *>(aBuf), cCh);
}

// --- binary packet assembly (VER_MotionCal) ---

void CManager::SendBinaryPacket(const Mag::SCal & cal)
{
	uint8_t aB[s_cBPacket];

	// header
	aB[0] = g_aBHeader[0];
	aB[1] = g_aBHeader[1];

	// assemble 16 floats at correct offsets
	static const int s_cG = 16;
	float aG[s_cG];
	aG[0]  = 0.0f;				// accel zero-g X (not yet implemented)
	aG[1]  = 0.0f;				// accel zero-g Y
	aG[2]  = 0.0f;				// accel zero-g Z
	aG[3]  = 0.0f;				// gyro zero-rate X (not yet implemented)
	aG[4]  = 0.0f;				// gyro zero-rate Y
	aG[5]  = 0.0f;				// gyro zero-rate Z
	aG[6]  = cal.m_vecV.x;		// mag hard iron X
	aG[7]  = cal.m_vecV.y;		// mag hard iron Y
	aG[8]  = cal.m_vecV.z;		// mag hard iron Z
	aG[9]  = cal.m_sB;			// mag field strength
	aG[10] = cal.m_matWInv.vecX.x;	// soft iron XX
	aG[11] = cal.m_matWInv.vecY.y;	// soft iron YY
	aG[12] = cal.m_matWInv.vecZ.z;	// soft iron ZZ
	aG[13] = cal.m_matWInv.vecX.y;	// soft iron XY
	aG[14] = cal.m_matWInv.vecX.z;	// soft iron XZ
	aG[15] = cal.m_matWInv.vecY.z;	// soft iron YZ

	memcpy(&aB[2], aG, s_cG * sizeof(float));

	// CRC-16 over bytes 0..65, append as little-endian
	UCrc crc = CrcFromBuffer(aB, s_cBPacket - 2);
	aB[66] = crc.m_aB[0];
	aB[67] = crc.m_aB[1];

	m_pWriter->Write(aB, s_cBPacket);
}

} // namespace Protocol
} // namespace libcalib
