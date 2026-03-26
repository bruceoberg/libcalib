#pragma once

#include <stddef.h>
#include <stdint.h>

#include "libcalib/common.h"
#include "libcalib/protocol_text.h"
#include "libcalib/protocol_bin.h"

namespace libcalib
{
namespace Protocol
{

enum VER
{
	VER_Imucal,		// embedded side: sends samples + cal echoes, receives binary cal packet
					// see https://github.com/adafruit/Adafruit_SensorLab/blob/master/examples/calibration/imucal/imucal.ino
	VER_MotionCal,	// desktop side: receives samples, sends binary cal packet
					// https://github.com/PaulStoffregen/MotionCal/blob/master/serialdata.c

	VER_Max,
	VER_Nil = -1
};

struct IWriter	// tag = wtr
{
	virtual void	Write(size_t cB, const uint8_t * pB) = 0;
};

struct IReader	// tag = rdr
{
	virtual size_t	CbRead(size_t cBMax, uint8_t * pB) = 0;	// returns bytes read, 0 if none
};

struct IReceiver	// tag = rcvr
{
	virtual void	OnSample(const SSample &)		{ }
	virtual void	OnMagCal(const Mag::SCal &)		{ }
	// NOTE(claude) OnAccelCal and OnGyroCal deferred until those calibration types are defined
};

class CManager	// tag = mgr
{
public:
			CManager(VER ver);

	void	Init(IWriter * pWriter, IReader * pReader, IReceiver * pReceiver);
	void	Update();						// drain IReader, parse, fire IReceiver callbacks

	// detected remote protocol version (VER_Nil until first valid message is parsed)
	VER		VerRemote() const				{ return m_verRemote; }
	bool	FHasRemote() const				{ return m_verRemote != VER_Nil; }

	// semantic send API — clients call these directly, CManager handles wire format
	void	SendSensorData(							// VER_Imucal: emits Raw: + Uni: lines
				float xAccel, float yAccel, float zAccel,	// m/s²
				float xGyro,  float yGyro,  float zGyro,	// rad/s
				float xMag,   float yMag,   float zMag);	// µT
	void	SendMagCal(const Mag::SCal & cal);		// VER_Imucal: emits Cal1:/Cal2: lines
												// VER_MotionCal: emits 68-byte binary packet

private:
	// line parse dispatch — called from Update() when Text::CParser completes a line
	void	DispatchLine(Text::CParser::LINEK linek);

	VER					m_ver;
	VER					m_verRemote;	// detected from incoming data; VER_Nil until identified
	IWriter *			m_pWriter;
	IReader *			m_pReader;
	IReceiver *			m_pReceiver;

	// text line parser (handles Raw:, Uni:, Cal1:, Cal2:)
	Text::CParser		m_textp;

	// Cal1:/Cal2: accumulation
	Mag::SCal			m_calPending;
	bool				m_fHasCal1;

	// binary packet parser
	Binary::CParser		m_binp;
};

} // namespace Protocol
} // namespace libcalib
