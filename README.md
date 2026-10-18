# Beebo: MeshCore Utilities for Heltec V4.3 Nodes <!-- omit in toc -->

Beebo is a fun project I started after experimenting with [MeshCore](https://meshcore.io/) using the [Heltec ESP32-S3R2 V4.3.1](https://heltec.org/project/wifi-lora-32-v4) LoRa development board, specifically in its headless configuration.

It is a mix of MeshCore firmware (**FW**) additions on top of the already excellent MeshCore code base, a brand new command line interface (**CLI**), and a collection of analysis and experiments done while using the Heltec V4.3.1 board in both companion and repeater roles.

In particular Beebo enables multi-transport **USB**, **BLE**, **TCP** (WiFi) connectivity where each mode can be turned on and off at runtime.
Coupled with a new fast client based Over-The-Air (OTA) FW update capability, this enables rapid in-the-field experimentation and maintenance.
For instance, in the case of a repeater, new FW can be uploaded to a repeater in WiFi range in around 30 seconds, with the node remotely toggled between a low power state with TCP/BLE radios off and a higher power mode with TCP radio on for servicing.

Statistical analysis of RX performance with respect to the use of the front end module (FEM) low noise amplifier (LNA) and other experiments were also a lot of fun, enabled by data captured by a custom live monitor ring that persists in free PSRAM memory.

The project is organized as a fork of [MeshCore](https://github.com/meshcore-dev/MeshCore). For the upstream project — what MeshCore is, supported hardware, mobile apps, and how to get started — see the [upstream README](https://github.com/meshcore-dev/MeshCore#readme).

Releases are made in lockstep with the upstream project and may be eventually be contributed back to the main MeshCore repository through PR.

Although FW releases are backward compatible with standard MeshCore applications, the Beebo CLI must be used to unlock some of the new features.
OTA upload, WiFi provisioning, transport selection, RF measurements, etc., are all handled by the `beebo` CLI, a Typer + Rich tool built on top of the upstream [`meshcore`](https://pypi.org/project/meshcore/) Python library and included in this repo under the `beebo/` directory.
General-purpose mesh commands (messaging, contacts, channels) are not included in beebo as these are already available from standard phone and desktop application as well as the use standard [meshcore-cli](https://github.com/meshcore-dev/meshcore-cli) unmodified command line interface application.

Beebo FW and CLI versions are both tagged as `beebo-v<meshcore_ver>.<beebo_ver>` (e.g. `beebo-v1.16.0.1`), so that it is easy to determine what beebo version is derived from what upstream MeshCore version, and what beebo CLI version should be used with what beebo FW.

By the way, Beebo 🐈 is the name of one of my cats 😄!

## Documentation

- [Getting Started](beebo/docs/getting-started.md) — flashing firmware, installing the CLI, connecting over USB/BLE/WiFi/apps
- [The Beebo CLI](beebo/docs/beebo-cli.md) — CLI reference and settings
- [Firmware Changes](beebo/docs/firmware.md) — OTA updates and the beebo-specific firmware additions on top of MeshCore
- [Board Analysis](beebo/docs/hardware.md) — SX1262/KCT8103L component analysis and TX/RX signal chain modeling
- [RF Experiments](beebo/docs/rf-experiments.md) — bench measurements, TX power sweeps, noise floor characterization, battery management

## References

<a id="bib-1"></a>**\[1\]** [SX1261/2 Data Sheet Rev 1.2](https://cdn.sparkfun.com/assets/6/b/5/1/4/SX1262_datasheet.pdf), Semtech, June 2019.

<a id="bib-2"></a>**\[2\]** [KCT Product Portfolio Q1 2025](https://www.ftelectronic.com/Public/Upload/news/20250430/68121966da4e7.pdf), Kangxi Communication Technologies.

<a id="bib-3"></a>**\[3\]** [KCT8101L Product Datasheet Rev C](http://datasheet5.oss-cn-shanghai.aliyuncs.com/datasheets/kxcomtech/KCT8101L.pdf), Kangxi Communication Technologies, October 2019.

<a id="bib-4"></a>**\[4\]** [47 CFR § 15.247](https://www.law.cornell.edu/cfr/text/47/15.247) — Operation within the bands 902–928 MHz, 2400–2483.5 MHz, and 5725–5850 MHz. See also [Understanding FCC Power Limits for Meshtastic Devices](https://meshcola.com/2024/11/13/understanding-fcc-power-limits-for-meshtastic-devices-a-deep-dive-into-section-15-247b3/).

<a id="bib-5"></a>**\[5\]** [Heltec V4: Measured TX Power Output Table](https://github.com/meshcore-dev/MeshCore/issues/1708), MeshCore Issue #1708 — conducted power measurements on V4.2 (GC1109) hardware across TX settings 1–24.

<a id="bib-6"></a>**\[6\]** [MeshCore FAQ — Radio Presets](https://github.com/meshcore-dev/MeshCore/blob/main/docs/faq.md), MeshCore upstream documentation — USA/Canada (Recommended) preset: 910.525 MHz, SF7, BW 62.5 kHz, CR 4/5.

<a id="bib-7"></a>**\[7\]** [Friis formulas for noise](https://en.wikipedia.org/wiki/Friis_formulas_for_noise), Wikipedia — cascade noise figure formula showing why the first amplifier stage dominates system noise performance.

<a id="bib-8"></a>**\[8\]** [Johnson-Nyquist noise](https://en.wikipedia.org/wiki/Johnson%E2%80%93Nyquist_noise), Wikipedia — thermal noise generated by conductors at non-zero temperature; derives the $kT$ noise power spectral density.

<a id="bib-9"></a>**\[9\]** [Receiver sensitivity](https://en.wikipedia.org/wiki/Receiver_sensitivity), Wikipedia — derivation of the standard sensitivity formula from thermal noise floor, noise figure, bandwidth, and required SNR.

<a id="bib-10"></a>**\[10\]** [Heltec LoRa 32 (V4.3) Datasheet](https://resource.heltec.cn/download/WiFi_LoRa_32_V4.3/HTIT-WB32LAv4.3_Reference_Design_V1.0.pdf), Heltec Automation — Table 3.5.2 lists RX sensitivity at SF7/125 kHz as $-122$ dBm.

## License

MeshCore is open-source software released under the MIT License.
## 🛠 Hardware Compatibility

MeshCore is designed for devices listed in the [MeshCore Flasher](https://meshcore.io/flasher)

## Contributing

Please submit PR's using 'dev' as the base branch!
For minor changes just submit your PR and we'll try to review it, but for anything more 'impactful' please open an Issue first and start a discussion. It is better to sound out what it is you want to achieve first, and try to come to a consensus on what the best approach is, especially when it impacts the structure or architecture of this codebase.

Here are some general principles you should try to adhere to:
* Keep it simple. Please, don't think like a high-level lang programmer. Think embedded, and keep code concise, without any unnecessary layers.
* No dynamic memory allocation, except during setup/begin functions.
* Use the same brace and indenting style that's in the core source modules. (A .clang-format is probably going to be added soon, but please do NOT retroactively re-format existing code. This just creates unnecessary diffs that make finding problems harder)

Help us prioritize! Please react with thumbs-up to issues/PRs you care about most. We look at reaction counts when planning work.

### Running unit tests

To run unit tests, run the following command:

```bash
pio test --environment native --verbose
```

## Road-Map / To-Do

There are a number of fairly major features in the pipeline, with no particular time-frames attached yet. In very rough chronological order:
- [X] Companion radio: UI redesign
- [X] Repeater + Room Server: add ACL's (like Sensor Node has)
- [X] Standardise Bridge mode for repeaters
- [ ] Repeater/Bridge: Standardise the Transport Codes for zoning/filtering
- [X] Core + Repeater: enhanced zero-hop neighbour discovery
- [ ] Core: round-trip manual path support
- [ ] Companion + Apps: support for multiple sub-meshes (and 'off-grid' client repeat mode)
- [ ] Core + Apps: support for LZW message compression
- [ ] Core: dynamic CR (Coding Rate) for weak vs strong hops
- [ ] Core: new framework for hosting multiple virtual nodes on one physical device
- [ ] V2 protocol spec: discussion and consensus around V2 packet protocol, including path hashes, new encryption specs, etc

## 📞 Get Support

- Report bugs and request features on the [GitHub Issues](https://github.com/meshcore-dev/meshcore/issues) page.
- Find additional guides and components on [my site](https://buymeacoffee.com/ripplebiz).
- Join [MeshCore Discord](https://meshcore.gg) to chat with the developers and get help from the community.
