#include "libcalib/protocol_text.h"
#include "libcalib/protocol.h"

#include <string.h>
#include <stdlib.h>
#include <stdio.h>

namespace libcalib
{
namespace Protocol
{
namespace Text
{

// Raw: line int16 -> float conversion.
// These must match the encoding in imucal.ino (Adafruit imucal firmware).
// The imucal encoding includes unit conversions as a side effect:
//   accel: m/s² × (8192/9.8) → int16, so decoding gives g (not m/s²)
//   gyro: deg/s × 16 → int16, so decoding gives deg/s
//   mag: µT × 10 → int16, so decoding gives µT

static constexpr float SAccelFromS16(int16_t s)
{
	return float(s) * (1.0f / 8192.0f);		// returns g
}

static constexpr float SGyroFromS16(int16_t s)
{
	return float(s) * (1.0f / 16.0f);		// returns deg/s
}

static constexpr float SMagFromS16(int16_t s)
{
	return float(s) * (1.0f / 10.0f);		// returns uT
}

// --- CParser ---

CParser::CParser()
{
	Reset();
}

void CParser::Reset()
{
	m_cChLine = 0;
	m_linek = LINEK_None;
	memset(&m_samp, 0, sizeof(m_samp));
	memset(m_aGCal, 0, sizeof(m_aGCal));
	m_cGCal = 0;
}

CParser::LINEK CParser::LinekFeedBytes(size_t cB, const uint8_t * pB)
{
	LINEK linekResult = LINEK_None;

	for (size_t iB = 0; iB < cB; ++iB)
	{
		char ch = static_cast<char>(pB[iB]);

		// CR marks end of line — parse and reset

		if (ch == '\r')
		{
			m_aChLine[m_cChLine] = '\0';
			linekResult = LinekParseLine();
			m_cChLine = 0;
		}
		else if (ch == '\n')
		{
			// ignore LF
		}
		else if (m_cChLine < s_cChLineMax - 1)
		{
			m_aChLine[m_cChLine++] = ch;
		}
		else
		{
			// line too long — discard and reset

			m_cChLine = 0;
		}
	}

	return linekResult;
}

CParser::LINEK CParser::LinekParseLine()
{
	m_linek = LINEK_None;

	// Match line prefix

	if (strncmp(m_aChLine, "Uni:", 4) == 0)
	{
		if (FParseUni(m_aChLine + 4))
			m_linek = LINEK_Uni;
	}
	else if (strncmp(m_aChLine, "Raw:", 4) == 0)
	{
		if (FParseRaw(m_aChLine + 4))
			m_linek = LINEK_Raw;
	}
	else if (strncmp(m_aChLine, "Cal1:", 5) == 0)
	{
		if (FParseCal(10, m_aChLine + 5))
		{
			m_linek = LINEK_Cal1;
			m_cGCal = 10;
		}
	}
	else if (strncmp(m_aChLine, "Cal2:", 5) == 0)
	{
		if (FParseCal(9, m_aChLine + 5))
		{
			m_linek = LINEK_Cal2;
			m_cGCal = 9;
		}
	}

	return m_linek;
}

bool CParser::FParseUni(const char * pCh)
{
	// Parse 9 comma-separated floats: ax,ay,az,gx,gy,gz,mx,my,mz

	float aG[9];
	char * pChEnd = nullptr;

	for (int iG = 0; iG < 9; ++iG)
	{
		aG[iG] = strtof(pCh, &pChEnd);

		if (pChEnd == pCh)
			return false;	// no digits parsed

		pCh = pChEnd;

		if (iG < 8)
		{
			if (*pCh != ',')
				return false;
			++pCh;
		}
	}

	// Convert from Uni: units (m/s^2, rad/s, uT) to SSample units (g, deg/s, uT)

	m_samp.m_pntAccel = SPoint(
		GFromMPerSecSq(aG[0]),
		GFromMPerSecSq(aG[1]),
		GFromMPerSecSq(aG[2]));
	m_samp.m_pntGyro = SPoint(
		DegFromRad(aG[3]),
		DegFromRad(aG[4]),
		DegFromRad(aG[5]));
	m_samp.m_pntMag = SPoint(aG[6], aG[7], aG[8]);

	return true;
}

bool CParser::FParseRaw(const char * pCh)
{
	// Parse 9 comma-separated int16s, convert to floats

	int16_t aN[9];
	char * pChEnd = nullptr;

	for (int iN = 0; iN < 9; ++iN)
	{
		long n = strtol(pCh, &pChEnd, 10);

		if (pChEnd == pCh)
			return false;	// no digits parsed

		// Range check for int16

		if (n < -32768 || n > 32767)
			return false;

		aN[iN] = static_cast<int16_t>(n);
		pCh = pChEnd;

		if (iN < 8)
		{
			if (*pCh != ',')
				return false;
			++pCh;
		}
	}

	// Convert to physical units

	m_samp.m_pntAccel = SPoint(
		SAccelFromS16(aN[0]),
		SAccelFromS16(aN[1]),
		SAccelFromS16(aN[2]));
	m_samp.m_pntGyro = SPoint(
		SGyroFromS16(aN[3]),
		SGyroFromS16(aN[4]),
		SGyroFromS16(aN[5]));
	m_samp.m_pntMag = SPoint(
		SMagFromS16(aN[6]),
		SMagFromS16(aN[7]),
		SMagFromS16(aN[8]));

	return true;
}

bool CParser::FParseCal(size_t cGExpected, const char * pCh)
{
	// Parse cGExpected comma-separated floats

	char * pChEnd = nullptr;

	for (size_t iG = 0; iG < cGExpected; ++iG)
	{
		m_aGCal[iG] = strtof(pCh, &pChEnd);

		if (pChEnd == pCh)
			return false;	// no digits parsed

		pCh = pChEnd;

		if (iG < cGExpected - 1)
		{
			if (*pCh != ',')
				return false;
			++pCh;
		}
	}

	return true;
}

// --- emit functions ---

void EmitSensorData(
	IWriter * pWriter,
	float xAccel, float yAccel, float zAccel,
	float xGyro,  float yGyro,  float zGyro,
	float xMag,   float yMag,   float zMag)
{
	if (pWriter == nullptr)
		return;

	char aBuf[256];

	// Raw: line — int16 encoded values
	// scale factors match MotionCal's *_PER_COUNT inverses
	int16_t nxA = static_cast<int16_t>(GFromMPerSecSq(xAccel) * 8192.0f);
	int16_t nyA = static_cast<int16_t>(GFromMPerSecSq(yAccel) * 8192.0f);
	int16_t nzA = static_cast<int16_t>(GFromMPerSecSq(zAccel) * 8192.0f);
	int16_t nxG = static_cast<int16_t>(DegFromRad(xGyro) * 16.0f);
	int16_t nyG = static_cast<int16_t>(DegFromRad(yGyro) * 16.0f);
	int16_t nzG = static_cast<int16_t>(DegFromRad(zGyro) * 16.0f);
	int16_t nxM = static_cast<int16_t>(xMag * 10.0f);
	int16_t nyM = static_cast<int16_t>(yMag * 10.0f);
	int16_t nzM = static_cast<int16_t>(zMag * 10.0f);

	int cCh = snprintf(aBuf, sizeof(aBuf),
		"Raw:%d,%d,%d,%d,%d,%d,%d,%d,%d\r\n",
		nxA, nyA, nzA, nxG, nyG, nzG, nxM, nyM, nzM);
	pWriter->Write(cCh, reinterpret_cast<const uint8_t *>(aBuf));

	// Uni: line — already SI units, print directly
	cCh = snprintf(aBuf, sizeof(aBuf),
		"Uni:%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f\r\n",
		xAccel, yAccel, zAccel,
		xGyro, yGyro, zGyro,
		xMag, yMag, zMag);
	pWriter->Write(cCh, reinterpret_cast<const uint8_t *>(aBuf));
}

void EmitMagCal(IWriter * pWriter, const Mag::SCal & cal)
{
	if (pWriter == nullptr)
		return;

	char aBuf[256];

	// Cal1: accel(0,0,0), gyro(0,0,0), hard iron xyz, field strength
	int cCh = snprintf(aBuf, sizeof(aBuf),
		"Cal1:%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\r\n",
		0.0f, 0.0f, 0.0f,	// accel offsets (not yet implemented)
		0.0f, 0.0f, 0.0f,	// gyro offsets (not yet implemented)
		cal.m_vecV.x, cal.m_vecV.y, cal.m_vecV.z,
		cal.m_sB);
	pWriter->Write(cCh, reinterpret_cast<const uint8_t *>(aBuf));

	// Cal2: flat row-major soft iron 3x3 (XX, XY, XZ, YX, YY, YZ, ZX, ZY, ZZ)
	const SMatrix3 & w = cal.m_matWInv;
	cCh = snprintf(aBuf, sizeof(aBuf),
		"Cal2:%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\r\n",
		w.vecX.x, w.vecX.y, w.vecX.z,
		w.vecY.x, w.vecY.y, w.vecY.z,
		w.vecZ.x, w.vecZ.y, w.vecZ.z);
	pWriter->Write(cCh, reinterpret_cast<const uint8_t *>(aBuf));
}

} // namespace Text
} // namespace Protocol
} // namespace libcalib
