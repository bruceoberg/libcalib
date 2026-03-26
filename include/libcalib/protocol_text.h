#pragma once

#include <stddef.h>
#include <stdint.h>

#include "libcalib/common.h"

namespace libcalib
{
namespace Protocol
{

struct IWriter;

namespace Text
{

// CParser — line-buffered parser for IMU serial data
//
// Accumulates bytes until CR, then parses complete lines.
// Handles Uni:, Raw:, Cal1:, and Cal2: line formats.
// Per-instance state (no statics) — fully reentrant.

class CParser	// tag = textp
{
public:
	enum LINEK
	{
		LINEK_None,		// no complete line yet
		LINEK_Uni,		// Uni: 9 floats (full precision)
		LINEK_Raw,		// Raw: 9 int16s (converted to floats)
		LINEK_Cal1,		// Cal1: 10 floats (calibration echo)
		LINEK_Cal2,		// Cal2: 9 floats (calibration echo)

		LINEK_Max,
		LINEK_Nil = -1
	};

	CParser();
	void Reset();

	// Feed bytes. Returns LINEK_None while buffering,
	// or a valid LINEK when a complete line is parsed.

	LINEK LinekFeedBytes(size_t cB, const uint8_t * pB);

	// Accessors (valid after corresponding line kind)

	const SSample & Samp() const		{ return m_samp; }
	const float * PGCal() const			{ return m_aGCal; }
	int CGCal() const					{ return m_cGCal; }

private:
	static const int s_cChLineMax = 256;

	char m_aChLine[s_cChLineMax];	// line buffer
	int m_cChLine;					// current line length

	LINEK m_linek;					// kind of last parsed line
	SSample m_samp;					// parsed sample data
	float m_aGCal[10];				// parsed calibration data
	int m_cGCal;					// count of calibration floats

	LINEK LinekParseLine();
	bool FParseUni(const char * pCh);
	bool FParseRaw(const char * pCh);
	bool FParseCal(size_t cGExpected, const char * pCh);
};

// --- emit functions (inverse of parsing) ---

// Emit Raw: + Uni: lines for sensor data (VER_Imucal wire format)
// Input units: m/s², rad/s, µT (SI)

void EmitSensorData(
		IWriter * pWriter,
		float xAccel, float yAccel, float zAccel,	// m/s²
		float xGyro,  float yGyro,  float zGyro,	// rad/s
		float xMag,   float yMag,   float zMag);	// µT

// Emit Cal1: + Cal2: lines for magnetometer calibration (VER_Imucal wire format)

void EmitMagCal(IWriter * pWriter, const Mag::SCal & cal);

} // namespace Text
} // namespace Protocol
} // namespace libcalib
