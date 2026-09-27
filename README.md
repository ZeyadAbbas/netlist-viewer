# Netlist Viewer

Netlist Viewer turns SPICE and PSpice netlists into cleaner, easier-to-read
schematic diagrams. It can be used interactively as a desktop viewer or from
the command line to render a netlist directly to a PNG image.

> [!IMPORTANT]
> This repository is a fork of
> [f18m/netlist-viewer](https://github.com/f18m/netlist-viewer). The fork is
> focused on producing prettier schematic output through improved automatic
> placement, conventional wiring, clearer labels, and headless image export.
> It is not an official upstream release.

## Improvements in this fork

- Topology-aware automatic placement that favors a conventional left-to-right
  signal flow and keeps components from overlapping.
- Orthogonal, obstacle-aware wire routing with shared trunks, junction dots,
  ground symbols, and global power-rail markers.
- Clearer component annotations with horizontal reference names, formatted
  values and units, and model names where appropriate.
- Improved transistor, JFET, independent-source, and waveform rendering.
- Support for both standalone `.SUBCKT` definitions and normal top-level
  SPICE/PSpice decks.
- Command-line PNG export for use in scripts, reports, and other applications.

## What is a netlist?

A [netlist](https://en.wikipedia.org/wiki/Netlist) describes the components in
an electrical circuit and the nodes that connect them. It is a compact text
representation of a [circuit diagram](https://en.wikipedia.org/wiki/Circuit_diagram).
For example:

```spice
.SUBCKT test_misc1 IN OUT

V1 0 IN DC=4V
R1 IN 2 1K
Q1 3 2 0 NPNstd
M1 OUT 3 0 NMOSstd
D1 OUT 0 DIODEstd

.ENDS
```

## Usage

### Interactive viewer

Launch Netlist Viewer without arguments to open the normal desktop interface:

```text
netlist_viewer.exe
```

Open a supported SPICE netlist from the application and use the canvas to
inspect, move, rotate, and zoom the automatically generated schematic.

### Headless PNG export

Render a netlist directly to a PNG without opening the interactive window:

```text
netlist_viewer.exe --input circuit.cir --output circuit.png
```

Short flags are also available:

```text
netlist_viewer.exe -i circuit.cir -o circuit.png
```

Run `netlist_viewer.exe --help` for the complete command-line usage. Headless
rendering uses the same parser, placement, routing, and drawing code as the GUI
and automatically crops unused whitespace from the exported image.

## Input support and limitations

- Common passive components, independent and controlled sources, diodes,
  BJTs, MOSFETs, and JFETs are supported.
- Files may contain a standalone `.SUBCKT` or a top-level circuit ending in
  `.END`.
- Embedded `.SUBCKT` model definitions are ignored when a drawable top-level
  circuit is present.
- Simulation directives such as `.DC`, `.AC`, `.TRAN`, `.PRINT`, and `.MODEL`
  are ignored because this project visualizes circuits; it does not simulate
  them.
- Standalone files containing multiple `.SUBCKT` blocks are not yet supported
  by the viewer interface.
- Subcircuit-instance elements and other unsupported SPICE device types may
  cause parsing to fail.
- Automatic layout is heuristic. Dense or unusual circuits may still benefit
  from manual adjustment in the interactive viewer.

## Building from source

Netlist Viewer depends primarily on
[wxWidgets](https://www.wxwidgets.org/) and [Boost](https://www.boost.org/).
Platform-specific instructions are available for:

- [Windows](NetlistViewer/build/win/README.md)
- [Linux](NetlistViewer/build/linux/README.md)
- [macOS](NetlistViewer/build/macos/README.md)

Development of this fork has primarily used the Visual Studio 2022 Windows
project. If a Windows build reports a missing `VCRUNTIME*.dll`, install the
current [Microsoft Visual C++ Redistributable](https://learn.microsoft.com/cpp/windows/latest-supported-vc-redist).

This fork currently expects users to build from source. Binaries published by
the upstream project predate and do not include the rendering changes described
above.

## Upstream project and history

The original Netlist Viewer was created by Francesco Montorsi in 2010. Versions
0.1 and 0.2 were published on
[SourceForge](https://sourceforge.net/projects/netlistviewer/), and later work
moved to the
[upstream GitHub repository](https://github.com/f18m/netlist-viewer).

This fork retains the upstream history and builds on that work with a stronger
focus on schematic presentation and automated image generation. Changes that
are generally useful to the original project may also be suitable for upstream
contribution.

## Contributing

Bug reports and pull requests are welcome. Helpful contributions include
parser compatibility, layout and routing improvements, reproducible netlist
fixtures, cross-platform build fixes, and automated visual regression tests.

When reporting a rendering problem, include the smallest netlist that
reproduces it and, when possible, the generated PNG.

## License

Netlist Viewer is available under the [MIT License](LICENSE). The original
copyright and license notice are retained from the upstream project.
