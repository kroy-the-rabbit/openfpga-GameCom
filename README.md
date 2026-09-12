# Game.com tech demo for Analogue Pocket

Hold **RT** to show the stylus. Use **RT + D-pad** to move it and **RT + A**
to touch the screen. **LT** is the Game.com **Sound** button.
D-pad moves; A/B/X/Y map to A/B/C/D. Start: Pause. Select: Menu.
Reset and Power are in the core menu.

**Highly alpha.** SD-ROM loading based on the MiSTer Game.com core.

## Tested

| ROM | Result |
|---|---|
| Duke Nukem 3D | Played on Pocket; video, controls and sound worked. Chunky; unimpressed |
| Lights Out | Played on Pocket; had fun |
| Castlevania: Symphony of the Night (prototype) | Tried on Pocket; fun until it came time to play |
| Centipede | Booted: publisher splash |
| Wheel of Fortune 2 | Booted: title and board |
| Indy 500 | Booted: logo |
| Sonic Jam | Booted: copyright/intro |
| Resident Evil 2 | Booted: title |

Storage readback tests passed for 23 unique ROMs across six sizes.
[ROM names and hashes](tools/fixtures/roms.json).

## Use

Merge the release ZIP's `Assets`, `Cores` and `Platforms` into the SD root.
Put individual `.tgc` ROMs in `Assets/gamecom/common/`.

BIOS tested: **Game.com External BIOS (1997), model 71-516**, 262,144 bytes.
Install as `Assets/gamecom/common/gamecom_bios.bin`.

```text
CRC32:  e235a589
SHA1:   97f782e72d738f4d7b861363266bf46b438d9b50
SHA256: 4cade226b44f09ebf5897b137d5564e5579b4d11b23dcabacd308fb0d18e7077
```

The internal boot replacement is included in the upstream RTL.
ROM/BIOS collections and external toolchain libraries are not shipped.
Supply the external BIOS, ROMs and Quartus libraries separately.

No physical cartridges, persistent saves, save states or sleep.

## Rebuild

Build from a Git checkout on Linux with Quartus Prime Lite **25.1std.0 Build 1129**,
Cyclone V device/IP libraries, Python 3 with venv support, Git, Make, rsync and zip.
Intel's toolchain, device libraries and simulation models must be installed separately.

```sh
QUARTUS_BIN=/opt/intelFPGA_lite/25.1std/quartus/bin \
  FITTER_EFFORT='STANDARD FIT' NPROC=16 make gamecom
```

Adjust `QUARTUS_BIN` for your installation. The container path accepts an `IMAGE`
containing Quartus. Outputs and timing/provenance reports: `build/gamecom/`.
Build-ID timestamps and random data can change the rebuilt bitstream hash.

[Timing reports](timing/README.md)

Optional tests use Icarus 13.0 and Verilator:

```sh
make sim-image
make test
```

## Provenance

- Machine: [GameCom_MiSTer](https://github.com/MiSTer-devel/GameCom_MiSTer),
  Jamie Blanks / kitrinx and contributors, `96ed90ec865302d5eb5a0aa8336844dbfb4a5342`.
- Pocket framework/build: `pocket-gba`, `f5de823aa9d2c4adf8470a6cbf3ce1fc98650762`,
  based on [mincer-ray/openfpga-GBA](https://github.com/mincer-ray/openfpga-GBA).
  APF: Analogue. SDRAM reference: Sorgelig. Generated IP: Intel/Altera.
- Pocket integration: kroy. Source hashes and copyright/license notices retained.
  Machine license: `src/fpga/gamecom/LICENSE`.

Releases use `0.99999.<commit>`. The Pocket ZIP contains installable files.
Source and rebuild tooling are available separately through GitHub.

## Related Analogue Pocket projects

- [Pocket tools](https://github.com/kroy-the-rabbit/pocket-tools): desktop UI.
- [GB/GBC cheats](https://github.com/kroy-the-rabbit/openfpga-GBC-cheats).
- [GBA cheats](https://github.com/kroy-the-rabbit/openfpga-GBA-cheats).
- [PC Engine cheats](https://github.com/kroy-the-rabbit/openfpga-pcengine-cheats).
- [Cartridge tools](https://github.com/kroy-the-rabbit/openfpga-carttools).
