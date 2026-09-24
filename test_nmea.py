import nmea

# The two classic published NMEA examples, used as independent anchors for the
# checksum routine: their *6A and *47 are documented, not computed by us.
CLASSIC_RMC = "$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A"
CLASSIC_GGA = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47"

# Sentences shaped like what the T-Deck's MIA-M10Q actually sends, at the
# position it was sitting at during bring-up.
LIVE_RMC = "$GNRMC,151321.00,A,5829.32904,N,02151.85104,E,0.045,185.38,050826,,,A*7F"
LIVE_RMC_VOID = "$GNRMC,151322.00,V,,,,,,,050826,,,N*6C"
LIVE_RMC_NO_COG = "$GNRMC,151323.00,A,5829.32904,N,02151.85104,E,5.400,,050826,,,A*64"
LIVE_GGA = "$GNGGA,151321.00,5829.32904,N,02151.85104,E,1,12,1.02,-4.0,M,21.0,M,,*68"


def test_checksum_accepts_published_examples():
    assert nmea.checksum_ok(CLASSIC_RMC)
    assert nmea.checksum_ok(CLASSIC_GGA)


def test_checksum_rejects_corrupted_body():
    # Flip a digit in the latitude but keep the original checksum.
    corrupted = CLASSIC_RMC.replace("4807.038", "4807.039")
    assert not nmea.checksum_ok(corrupted)


def test_checksum_rejects_missing_or_malformed():
    assert not nmea.checksum_ok("")
    assert not nmea.checksum_ok("$GPRMC,123519,A,4807.038,N")  # no checksum at all
    assert not nmea.checksum_ok("$GPRMC,123519,A*ZZ")  # non-hex checksum
    assert not nmea.checksum_ok("GPRMC,123519,A*6A")  # no leading delimiter


def test_parse_rmc_extracts_position_course_and_speed():
    fix = nmea.parse_rmc(LIVE_RMC)
    assert fix is not None
    assert abs(fix.lat - 58.4888173) < 1e-6
    assert abs(fix.lon - 21.8641840) < 1e-6
    assert abs(fix.cog_deg - 185.38) < 1e-9
    assert abs(fix.sog_kn - 0.045) < 1e-9


def test_parse_rmc_handles_southern_and_western_hemispheres():
    body = "GPRMC,123519,A,4807.038,S,01131.000,W,022.4,084.4,230394,,,A"
    sentence = f"${body}*{_checksum(body)}"
    fix = nmea.parse_rmc(sentence)
    assert fix is not None
    assert abs(fix.lat + 48.1173) < 1e-4
    assert abs(fix.lon + 11.5166667) < 1e-6


def test_parse_rmc_rejects_void_status():
    # A void fix still carries plausible-looking fields; steering on it would be
    # steering on a receiver that has told us it has no solution.
    assert nmea.parse_rmc(LIVE_RMC_VOID) is None


def test_parse_rmc_returns_none_cog_when_stationary():
    # The receiver leaves COG empty with no way on. That must not become 0.0,
    # which would read as "heading due north".
    fix = nmea.parse_rmc(LIVE_RMC_NO_COG)
    assert fix is not None
    assert fix.cog_deg is None
    assert abs(fix.sog_kn - 5.4) < 1e-9


def test_parse_rmc_rejects_truncated_datagram():
    # A datagram cut mid-sentence is the dangerous case: it can still look like
    # a parseable sentence. The checksum is what catches it.
    assert nmea.parse_rmc(LIVE_RMC[:40]) is None


def test_parse_rmc_ignores_other_sentence_types():
    assert nmea.parse_rmc(LIVE_GGA) is None


def test_parse_coordinate_rejects_impossible_minutes():
    assert nmea.parse_coordinate("4867.000", "N") is None  # 67 minutes
    assert nmea.parse_coordinate("", "N") is None
    assert nmea.parse_coordinate("4807.038", "") is None
    assert nmea.parse_coordinate("4807.038", "X") is None


def test_parse_gga_quality():
    quality, sats = nmea.parse_gga_quality(LIVE_GGA)
    assert quality == 1
    assert sats == 12


def test_parse_gga_quality_ignores_rmc():
    assert nmea.parse_gga_quality(LIVE_RMC) == (None, None)


def test_split_datagram_handles_bytes_and_multiple_sentences():
    payload = (LIVE_RMC + "\r\n" + LIVE_GGA + "\r\n").encode("ascii")
    lines = nmea.split_datagram(payload)
    assert lines == [LIVE_RMC, LIVE_GGA]


def test_split_datagram_survives_non_ascii_noise():
    # Arbitrary bytes must not raise -- the checksum rejects them downstream.
    lines = nmea.split_datagram(b"\xff\xfe garbage\r\n" + LIVE_RMC.encode("ascii"))
    assert LIVE_RMC in lines
    assert nmea.parse_rmc(lines[0]) is None


def _checksum(body):
    value = 0
    for char in body:
        value ^= ord(char)
    return f"{value:02X}"
