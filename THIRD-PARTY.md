# Dependencies and notices

Python dependencies are installed separately; versions, distribution URLs and
SHA256 hashes are retained in `uv.lock`. No virtualenv, interpreter or dependency
binary is part of this source export. Copied license texts are under
`third-party-licenses/`; third-party copyright and terms remain their own.

Runtime: paho-mqtt 2.1.0 (EPL-2.0 / EDL-1.0 choices in its full LICENSE.txt),
pigpio 1.78 (Unlicense). The vendored driver is GPL-3.0-or-later, separately
identified in PROVENANCE.md. pigpio daemon/hardware installation is separate.

Test dependencies: pytest 9.1.1, iniconfig 2.3.0 and pluggy 1.6.0 (MIT),
packaging 26.2 (Apache-2.0 or BSD-2-Clause), Pygments 2.20.0 (BSD-2-Clause),
and Windows-only colorama 0.4.6 (BSD-3-Clause). All full license texts are included.

License evidence: installed exact-version distribution LICENSE files for all
except pigpio, whose PyPI sdist omits a license file. Its full text was retrieved
from https://raw.githubusercontent.com/joan2937/pigpio/v78/UNLICENCE.
The colorama notice came from the wheel pinned by `uv.lock`, verified against
its SHA256. The Paho wheel omits the two texts referenced by LICENSE.txt;
`epl-v20` and `edl-v10` were retrieved from the official repository at tag
https://github.com/eclipse-paho/paho.mqtt.python/tree/v2.1.0 . This documentation does not relicense third-party components.
