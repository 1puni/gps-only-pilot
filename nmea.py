"""NMEA-0183 parsing for the T-Deck's GPS feed.

The T-Deck broadcasts its u-blox MIA-M10Q's own sentences over UDP rather than
re-encoding them, so what arrives here is whatever the receiver emits: `$GNRMC`
and `$GNGGA` at the configured nav rate.

RMC is the sentence that matters -- it is the only one carrying course and speed
over ground, which is what the autopilot steers on. GGA is parsed only for fix
quality and satellite count, which are useful for diagnosing a bad feed but are
never steered on.

Pure parsing, no sockets: `test_nmea.py` covers it without hardware.
"""

from dataclasses import dataclass


# A sentence is at most 82 chars including the leading delimiter and CRLF.
# Anything longer is malformed, or two sentences that got glued together.
MAX_SENTENCE_LEN = 82


@dataclass
class NmeaFix:
    """A usable position from an RMC sentence. Angles in degrees, speed in knots."""

    lat: float
    lon: float
    cog_deg: float | None  # None when stationary: COG is meaningless without way on
    sog_kn: float


def checksum_ok(sentence):
    """True if the trailing *hh checksum matches the XOR of the body.

    UDP is unordered and unreliable, and a truncated datagram that still parses
    is the dangerous case -- a half-received latitude would otherwise steer us.
    A sentence with no checksum is rejected: the receiver always sends one, so
    its absence means damage.
    """
    if not sentence or sentence[0] not in "$!":
        return False
    star = sentence.rfind("*")
    if star == -1 or star + 3 > len(sentence):
        return False

    body = sentence[1:star]
    try:
        expected = int(sentence[star + 1 : star + 3], 16)
    except ValueError:
        return False

    actual = 0
    for char in body:
        actual ^= ord(char)
    return actual == expected


def parse_coordinate(value, hemisphere):
    """Converts NMEA ddmm.mmmm / dddmm.mmmm plus hemisphere into signed degrees.

    Returns None if either field is empty or malformed, which is what the
    receiver sends before it has a fix.
    """
    if not value or not hemisphere:
        return None
    if hemisphere not in ("N", "S", "E", "W"):
        return None

    # Degrees are the leading 2 digits for latitude, 3 for longitude; the rest is
    # decimal minutes. Split on the dot rather than assuming a fixed width, since
    # the number of decimal places varies between receivers.
    dot = value.find(".")
    if dot == -1:
        dot = len(value)
    if dot < 3:
        return None

    try:
        degrees = float(value[: dot - 2])
        minutes = float(value[dot - 2 :])
    except ValueError:
        return None
    if minutes >= 60:
        return None

    result = degrees + minutes / 60.0
    if hemisphere in ("S", "W"):
        result = -result
    if hemisphere in ("N", "S") and not -90 <= result <= 90:
        return None
    if hemisphere in ("E", "W") and not -180 <= result <= 180:
        return None
    return result


def _float_or_none(value):
    if not value:
        return None
    try:
        result = float(value)
    except ValueError:
        return None
    return result


def parse_rmc(sentence):
    """Parses an RMC sentence into an NmeaFix, or returns None if unusable.

    Accepts any talker ID (GN/GP/GL/GA): the M10 sends GN when it has a
    multi-constellation solution, but falls back to GP on GPS alone.
    """
    sentence = sentence.strip()
    if len(sentence) > MAX_SENTENCE_LEN or not checksum_ok(sentence):
        return None

    star = sentence.rfind("*")
    fields = sentence[1:star].split(",")
    # 0=id 1=time 2=status 3=lat 4=N/S 5=lon 6=E/W 7=SOG 8=COG
    if len(fields) < 9:
        return None
    if not fields[0].endswith("RMC"):
        return None

    # Status: A = active, V = void. A void fix still carries stale-looking
    # coordinates, so this check has to come before parsing them.
    if fields[2] != "A":
        return None

    lat = parse_coordinate(fields[3], fields[4])
    lon = parse_coordinate(fields[5], fields[6])
    if lat is None or lon is None:
        return None

    sog = _float_or_none(fields[7])
    if sog is None or sog < 0:
        return None

    # COG is empty when the receiver is stationary. That is not an error, but it
    # is also not a heading -- the caller must not steer on a fabricated 0.
    cog = _float_or_none(fields[8])
    if cog is not None and not 0 <= cog <= 360:
        cog = None

    return NmeaFix(lat=lat, lon=lon, cog_deg=cog, sog_kn=sog)


def parse_gga_quality(sentence):
    """Returns (fix_quality, satellites) from a GGA sentence, or (None, None).

    Quality 0 means no fix. Only used for diagnostics -- steering decisions come
    from RMC, which has its own validity flag.
    """
    sentence = sentence.strip()
    if len(sentence) > MAX_SENTENCE_LEN or not checksum_ok(sentence):
        return None, None

    star = sentence.rfind("*")
    fields = sentence[1:star].split(",")
    if len(fields) < 8 or not fields[0].endswith("GGA"):
        return None, None

    try:
        quality = int(fields[6]) if fields[6] else None
        sats = int(fields[7]) if fields[7] else None
    except ValueError:
        return None, None
    return quality, sats


def split_datagram(payload):
    """Splits a received datagram into candidate sentences.

    The T-Deck sends one sentence per datagram, but nothing guarantees that --
    other NMEA sources batch them, and we should not care which we are talking to.
    """
    if isinstance(payload, bytes):
        # Latin-1 never raises on arbitrary bytes; a corrupt sentence will be
        # caught by the checksum rather than by a decode error.
        payload = payload.decode("latin-1", "replace")
    return [line.strip() for line in payload.replace("\r", "\n").split("\n") if line.strip()]
