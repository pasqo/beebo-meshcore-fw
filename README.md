# Beebo: MeshCore for Heltec V4.3 Nodes <!-- omit in toc -->

Beebo is a fun project I started after experimenting with [MeshCore](https://meshcore.io/) on the Heltec WiFi LoRa 32 V4 (the V4.3 is the reference board), specifically in its headless configuration. It is a fork of MeshCore: firmware additions on top of the already excellent code base, a command line tool, and a collection of analysis and experiments.

Beebo adds four things to stock MeshCore: a repeater that tests its own settings against real traffic and keeps what works; one firmware image that is a companion or a repeater at your command; a feed in your app where the node reports its health and progress; and a monitor ring that records the packets it handles for analysis. Release `1.18.0.1.0` is a drop-in replacement for stock MeshCore v1.18.0 and keeps working with the apps you already have.

USB, BLE and TCP (WiFi) can each be turned on and off at runtime, and a fast over-the-air update makes field experiments quick: a repeater in WiFi range takes new firmware in about 30 seconds.

## Get started

- **[Documentation](https://pasqo.github.io/beebo-meshcore-fw/)**: the [Quick Start](https://pasqo.github.io/beebo-meshcore-fw/quick-start/) flashes a board and runs a first tuning trial from your phone, with no computer tools. The [feature list](https://pasqo.github.io/beebo-meshcore-fw/features/) shows everything beebo adds.
- **Firmware**: `heltec-v4-1.18.0.beebo.1.0.bin` on the [releases](https://github.com/pasqo/beebo-meshcore-fw/releases) page, flashed with the MeshCore [Web Flasher](https://meshcore.io/flasher) (Custom Firmware).
- **Command line** (optional): `pip install beebo-cli`, for over-the-air updates, WiFi setup, monitoring and analysis. See [Install and Connect](https://pasqo.github.io/beebo-meshcore-fw/getting-started/). General mesh commands (messages, contacts, channels) stay in the apps and the stock [meshcore-cli](https://github.com/meshcore-dev/meshcore-cli).

Versions are tagged `beebo-v<meshcore version>.<beebo version>` (`beebo-v1.18.0.1.0`), so it is clear which MeshCore release a beebo release is built on.

## Support and feature requests

Beebo is a one-person project supported in spare time. Open an [issue](https://github.com/pasqo/beebo-meshcore-fw/issues) for a bug or an idea, or chat on the [MeshCore Discord](https://meshcore.gg). Details are on the [Support page](https://pasqo.github.io/beebo-meshcore-fw/support/).

## Upstream and contributing

For what MeshCore is, the supported hardware and the apps, see the [upstream README](https://github.com/meshcore-dev/MeshCore#readme). Releases follow upstream and may be contributed back by pull request. Changes that belong to MeshCore itself go to the upstream `dev` branch. Run the unit tests with `pio test --environment native --verbose`.

## License

MIT, like MeshCore.

By the way, Beebo 🐈 is the name of one of my cats 😄!
