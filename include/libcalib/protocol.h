#pragma once

#include <stdint.h>

#include "libcalib/common.h"
#include "libcalib/lineparser.h"

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
	virtual void	Write(const uint8_t * pB, int cB) = 0;
};

struct IReader	// tag = rdr
{
	virtual int		CbRead(uint8_t * pB, int cBMax) = 0;	// returns bytes read, 0 if none
};

struct IReceiver	// tag = rcvr
{
	virtual void	OnSample(const SSample &)		{ }
	virtual void	OnMagCal(const Mag::SCal &)		{ }
	// NOTE(claude) OnAccelCal and OnGyroCal deferred until those calibration types are defined
};

// packet constants
static const int s_cBPacket = 68;
extern uint8_t g_aBHeader[2];		// { 0x75, 0x54 }

// CRC
union UCrc
{
	uint16_t	m_w;
	uint8_t		m_aB[2];	// [0] = low, [1] = high
};

uint16_t	Crc16Update(uint16_t crc, uint8_t b);
UCrc		CrcFromBuffer(const uint8_t * pB, int cB);

// CPacketParser — binary calibration packet parser (68-byte MotionCal packets)

class CPacketParser	// tag = packetp
{
public:
				CPacketParser();
	void		Reset();

	// Feed bytes one at a time. Returns true when a complete valid packet is parsed.
	bool		FFeedBytes(const uint8_t * pB, int cB);

	// Accessor (valid after FFeedBytes returns true)
	const Mag::SCal &	Cal() const		{ return m_cal; }

private:
	void		FeedByte(uint8_t b);
	bool		FParsePacket();

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

class CManager	// tag = mgr
{
public:
				CManager(VER ver);

	void		Init(IWriter * pWriter, IReader * pReader, IReceiver * pReceiver);
	void		Update();						// drain IReader, parse, fire IReceiver callbacks

	// semantic send API — clients call these directly, CManager handles wire format
	void		SendSample(const SSample & samp);		// VER_Imucal: emits Raw: + Uni: lines
	void		SendMagCal(const Mag::SCal & cal);		// VER_Imucal: emits Cal1:/Cal2: lines
													// VER_MotionCal: emits 68-byte binary packet

private:
	// line parse dispatch — called from Update() when CLineParser completes a line
	void		DispatchLine(CLineParser::LINETYPE lt);

	// binary packet sending
	void		SendBinaryPacket(const Mag::SCal & cal);

	VER				m_ver;
	IWriter *		m_pWriter;
	IReader *		m_pReader;
	IReceiver *		m_pReceiver;

	// line parser (handles Raw:, Uni:, Cal1:, Cal2:)
	CLineParser		m_linep;

	// Cal1:/Cal2: accumulation
	Mag::SCal		m_calPending;
	bool			m_fHasCal1;

	// binary packet parser
	CPacketParser	m_packetp;
};

} // namespace Protocol
} // namespace libcalib
