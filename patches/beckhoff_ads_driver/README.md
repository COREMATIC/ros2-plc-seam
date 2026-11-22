# Fixes for beckhoff_ads_driver on ROS 2 Jazzy

These patches apply to [b-robotized/beckhoff_ads_driver](https://github.com/b-robotized/beckhoff_ads_driver)
at commit `5aa1f59` (the head of `rolling` on 22 November 2026). `tools/fetch_ads_driver.sh`
applies them. They are being proposed upstream; remove them once upstream has merged them.

| Patch | What it fixes |
|---|---|
| `0001` | The first failed ADS read or write crashed the controller manager (a null log clock). |
| `0002` | The driver released its symbol handles as soon as it took them, so every read failed. After an ADS error, the component went to FINALIZED instead of back to UNCONFIGURED, so it could not reconnect without a restart. |
| `0003` | Adds the optional `ads_timeout_ms` hardware parameter. The default ADS timeout is 5000 ms per call. |
| `0004` | A PLC symbol the PLC does not know now fails `on_configure`, by name. |
| `0005` | `write()` sends only the symbols that received a command this cycle, so it no longer writes back a value the PLC changed in the meantime. |

Each patch builds on Jazzy, one after the other. Together, they run on a bin-picking cell in
commissioning since October 2026. They also passed fault tests against an ADS test server: link
cut, PLC not answering, PLC restart. The driver is licensed Apache-2.0, like these patches.
