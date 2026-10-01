# 108 | Gdansk Electric

A playable Godot 4 prototype of ZTM Gdansk bus route 108, between Plac
Solidarnosci and Chelm Wieckowskiego. The bundled data retains Polish stop
names and supports both directions, including their different street paths.
The target period is present day. The bundled service date is 2026-09-15;
terrain survey age is recorded independently from the transport timetable.

## Run

Open `project.godot` in Godot 4.7.2 and press **F6** on `scenes/main.tscn`,
or **F5** to run the project. No plugins, API keys or network connection are
needed to play. The initial city generation can take several seconds.
Compatibility rendering is selected for broad desktop GPU support.

| Control | Action |
| --- | --- |
| W / Up | Accelerate |
| S / Down | Brake |
| A / D or Left / Right | Steer |
| Space | Hold parking brake |
| E | Open/close all doors while stopped |
| Q | Select forward/reverse while stopped |
| C | Switch chase/cab camera |
| O | Toggle orthophoto terrain imagery |
| R | Recover to the next stop (practice aid) |
| Tab | Restart in the opposite direction while stopped |
| Escape | Pause/resume |

The bus starts at the first passenger stop. Open the doors for five seconds
to board, close them, then follow the mint road arrows to the next stop.
Stop near its platform and repeat. Open doors inhibit acceleration.
All stops, including request stops, are mandatory in this prototype.
Passengers are a simulated count, not animated characters. The service ends
after the final stop; Tab starts the opposite direction.

## What Is Real

- ZTM GTFS route shape coordinates, ordered stop names, stop coordinates,
  headsigns and reference trip times. One Godot unit represents one metre.
- 13 passenger stops towards Chelm and 12 towards Plac Solidarnosci, checked
  against the public ZTM line page on 2026-09-14.
- OpenStreetMap road centre lines and building footprints in the corridor.
  Building height tags, levels or defaults remain the fallback outside the
  LiDAR-supported buildings. Buildings have collision geometry.
- GUGiK/Geoportal NMT bare-earth elevations from the supplied 2018-03-18
  survey: 1 m source grids, cropped and bilinearly resampled to 5 m for play.
  Route elevations range from approximately 2 to 67 m in the source vertical
  datum. The runtime crop is approximately 3 by 3.6 km, with a 250 m margin.
- Geoportal RGB orthophotos acquired on 2021-07-13 and 2021-09-08, at
  25 cm source resolution. Available coverage is applied as a terrain texture,
  resampled to 1 m per pixel; it does not replace generated road geometry.
- Classified Geoportal LiDAR from 2018 supplies measured roof elevations for
  767 buildings, including 193 fitted simple gables, and 2,949 tree canopy
  proxies within 180 m of the route shapes. This is derived scenery, not a
  raw point cloud or a complete photorealistic reconstruction.

## Prototype Limits

This is a driving/gameplay foundation, not a finished city reconstruction.
Road widths, sidewalks, shelters and stop platforms are generated approximations.
Stops are projected from their real coordinates onto the route and placed on
the right-hand side. Terrain follows the imported NMT, and roads are draped
over it. This is not surveyed road grading: kerbs, cuttings and retaining walls
can still cause rough road profiles. Bridges and grade-separated roads are not
reconstructed; bare-earth data cannot supply bridge decks. The 2018 survey
does not reflect every present-day roadwork. Intersections are overlapping road ribbons,
not lane-aware junctions. OSM relation-based multipolygons and water surfaces
are not rendered. LiDAR-supported facades have procedural windows and sills;
their layouts and colours are illustrative, not surveyed architecture. Roofs
use measured elevations and simple fitted shapes, with the 2021 orthophoto
projected onto them. Complex roofs, spires and dormers are not reconstructed.
Orthophotos contain photographed roofs, trees, vehicles and baked-in shadows;
these are flat image detail, not new 3D objects. Imagery and current generated
roads may disagree due to capture dates and approximate road widths. Use O to
compare textured and plain terrain. Uncovered areas retain plain ground colour.

The procedural bus follows Solaris Urbino 12 electric proportions: 12 m body,
2.55 m width, 5.9 m wheelbase and three right-side doors. It is not a licensed
Solaris model, and there is no affiliation with ZTM, GAiT or Solaris. The cab is
simplified, and the mirrors are not live cameras.

### Vehicle Repaints

If `Vehicles/Urbino IV/*.cti` exists, the bus loads that OMSI 2 "Urbino IV"
repaint instead of the plain colours. `scripts/bus_livery.gd` reads the `.cti`
`[item]` texture list and `[setvar]` options. It maps the body atlas onto a
UV-mapped shell using regions measured on the Urbino IV template: the side
bands, front mask, rear panel and cap, roof and wheel covers. The side bands
are 343 px/m, and their wheel-arch spacing matches the 5.9 m wheelbase. Door
openings come from the gaps in the right-side panel. Pure-black template
cut-outs on the lower panels are discarded, so the wheels show through the
arches.

These options are used:

- `EV_kaganiec` sets the speed governor (70/60/65/80 km/h).
- `EV_zasieg` sets the nominal range. Battery capacity is that range times an
  illustrative 1.2 kWh/km.
- `wyswietlacz_tyl`, `wyswietlacz_bok_drzwi` and `wyswietlacz_bok_kier`
  configure the rear and side destination displays.

The registration plate texture is placed front and rear. Interior, glass-sticker,
seat and rail textures are listed but not loaded, because their UVs belong to
the OMSI model, which is not included. Without a repaint, the plain procedural
body is used. Repaints are third-party artwork: check the author's licence
before redistributing them in a build. Enable mipmaps in the `.import` settings
for large atlases.

Driving uses Godot CharacterBody3D collisions with a force-based longitudinal
model. It includes approximately 20 kN/160 kW traction, rolling and aerodynamic
drag, grade, 6.5 m/s² brakes, coast and regenerative braking, plus automatic
hold and hill-hold. Steering uses a rear-axle bicycle model with a grip limit.
The body tilts to four ground probes and has damped visual squat, dive, roll
and door-side kneeling. There is no tyre or suspension force simulation.
Drivetrain figures are approximations, not manufacturer data. There is no
traffic, pedestrians, signals, enforced speed limits, ticketing, audio,
timetable scoring or save system yet. The HUD's 50 km/h sign is a fixed
gameplay reference, not imported speed-limit data. Desktop keyboard only.

`tests/bus_dynamics.gd` checks acceleration, top speed, braking distance,
turning radius, rear-axle slip, grip limit, hold and hill starts. It also
checks repaint parsing and fallback. Run it with
`--headless --fixed-fps 60 --script tests/bus_dynamics.gd`.

## Data And Attribution

- **ZTM Gdansk / Otwarty Gdansk**, catalog marked CC BY:
  https://ckan.multimediagdansk.pl/dataset/tristar
- Public route reference: https://ztm.gda.pl/rozklady/linia-108.html
- **(c) OpenStreetMap contributors**, ODbL 1.0:
  https://www.openstreetmap.org/copyright
- **GUGiK / Geoportal**, NMT, orthophotos and LiDAR supplied by the user:
  https://www.geoportal.gov.pl/
  Download provenance is retained in `data/pobieracz_nmt_20260915143625.txt`.
  Orthophoto provenance is in `data/pobieracz_ortofoto_20260915222837.txt`.
  LiDAR provenance is in `data/pobieracz_las_20260915232138.txt` (2018).
  Retain source attribution and check the dataset's reuse terms for distribution.

`data/route_108.json` contains the transformed GTFS snapshot and source trip
IDs. The importer selects a representative longest active stop sequence in each
direction, removes Jana z Kolna terminal entries outside the published passenger
route, and retains the full source shape. It applies `calendar.txt` (if present)
and `calendar_dates.txt` for `service_date`, defaulting to the import day's date.
It is an offline snapshot, not a guarantee of live operation. Original arrival times
are reference metadata only, not the simulation clock.

`data/map_108.json` is a transformed OSM extract with source way IDs and tags.
It remains ODbL data, separate from game code. Keep attribution and applicable
database share-alike obligations when redistributing it. Consult ZTM's linked
data-use terms before publishing a release. Source timestamps are in each file.

## Refresh Data

Run from PowerShell in the project directory:

```powershell
./tools/Import-Route.ps1
./tools/Import-Map.ps1
```

The first command downloads the official GTFS archive, uses PowerShell CSV
parsing, and stores only route 108. To reuse a downloaded archive:

```powershell
./tools/Import-Route.ps1 -GtfsDirectory .cache/gtfs
```

Use `-ServiceDate '2026-09-15'` to reproduce a particular day's selection if
that date is included in the feed. Unsupported dates fail without replacing
the bundled route. Refresh the terrain crop after changing route geometry.

The map importer makes nine sequential bounded requests to the official OSM
API, then deduplicates by source ID. Run only when refreshing data, not each
time the game launches. Network errors stop imports before replacing the
existing snapshot. Raw GTFS downloads and test captures live in ignored `.cache`.

## Terrain Import

The seven original `.asc` files (about 180 MiB) stay untouched in `data/` and
are ignored by Git. Only six intersect the crop. The game reads the generated
`data/terrain_108.json` metadata and `data/terrain_108.bin` float32 heightfield
(590 x 733 samples, about 1.65 MiB), not the large source grids. Keep both
generated files together; include JSON and BIN data when configuring exports,
and exclude raw ASC tiles from a distributed build.

Python is needed only to regenerate terrain, not to play:

```powershell
python -m pip install -r tools/requirements-terrain.txt
python tools/import_terrain.py --source-crs EPSG:2180
```

Use the Python interpreter where these dependencies are installed. The importer
uses Rasterio/GDAL for ASCII raster parsing, mosaicking and resampling, and
PROJ for coordinate transforms. The ASC headers contain no CRS declaration;
EPSG:2180 (Poland CS92) is supplied explicitly and spatially matches the route.
Confirm the CRS when substituting other downloads. No vertical datum conversion
is applied. The manifest records source survey dates, not the gameplay period.

The importer checks all 708 route shape samples, rejects missing data, checks
the runtime projection against the game's metre-based coordinates and reports
coverage in `.cache/terrain-report.json`. It adds a source border for reprojection
before exporting the final crop. All seven tiles can remain in the input folder;
non-intersecting tiles are skipped automatically. `--spacing` and `--margin`
adjust the output resolution and crop margin in metres.

Terrain renders in chunks with matching triangle collision surfaces. Both route
directions share the heightfield's geographic origin. A low fallback plane is
only a safety floor outside the crop, not a substitute for missing route terrain.

## Orthophoto Import

The five source TIFFs (about 215 MiB) stay untouched in `data/` and are ignored
by Git. They contain RGB imagery at 25 cm resolution, in PL-1992/CS92, from
July and September 2021. These dates are separate from the present-day service
date and the 2018 NMT survey. Newer roadworks may not appear in the images.

Use the same Python environment and dependencies as the terrain importer:

```powershell
python tools/import_orthophotos.py --inspect-only
python tools/import_orthophotos.py
```

The importer checks each TIFF's embedded CRS, RGB band interpretation, data
type and download metadata. The supplied TIFFs have differing datum/axis
definitions; their CS92 projection parameters are checked, but each original
CRS is retained for reprojection rather than silently relabelled EPSG:2180.
Rasterio/GDAL warps one source at a time onto the existing terrain extent.
The original 25 cm files remain available for detailed road tracing in GIS.

Outputs:

- `data/orthophoto_108.png`: 2945 x 3660 RGB terrain texture, approximately
  17.02 MiB on disk at 1 m per pixel. GPU memory is larger: about 55 MiB for
  RGBA pixels with mipmaps, depending on the rendering backend.
- `data/orthophoto_108.json`: image extents, source CRS definitions, survey
  dates, valid-pixel coverage, uncovered stops and suggested missing sheets.
- `.cache/orthophoto_coverage.png`: north-up reference preview with both route
  shapes in red. Plain green areas have no supplied imagery.

The five tiles cover 87.9% of the terrain crop, all 708 route shape points,
and all 25 stop entries across both directions. Point counts are not a
percentage of route length. Sheet **N-34-50-C-c-4-4**, acquired on 2021-09-08,
fills the former gap at Sikorskiego, Kopeckiego and Chelm Wieckowskiego. Its
download metadata is retained in `data/pobieracz_ortofoto_20260915225632.txt`.
A northwest area outside the route remains uncovered; no further sheet is
currently flagged for the route's shape points or stops.

To extend the surroundings, add TIFFs and their metadata to `data/`, then rerun the importer. Missing
imagery is deliberately filled with the existing ground colour; it never
changes elevations or collisions. `--metres-per-pixel` controls runtime texture
resolution, with a 4096-pixel maximum dimension. The game uses mipmaps and
anisotropic filtering and maps image edges to the terrain's outer vertices.
An absent or misaligned imagery crop falls back to plain terrain with a warning
for misalignment. Reimport imagery whenever the terrain extent/origin changes.

For distribution, include the generated PNG and JSON (the PNG is loaded as an
Image from its original path), and exclude the source TIFFs. Godot currently
warns that this raw Image loading path is not export-safe; resource loading
must be adapted and an exported build tested before distribution. Retain source
attribution and verify imagery reuse terms before publishing a build. This
layer is a visual reference, not automatic extraction of lanes or buildings.

## LiDAR Scenery

The supplied survey has 27 unique LAZ tiles, 171,112,894 points and about
916 MiB of compressed data. The extra COPC copy of tile
`70930_849046_N-34-50-C-c-4-4-4-1` is skipped when its ordinary LAZ exists.
Original LAZ/LAS files are untouched and ignored by Git. The game loads only
`data/lidar_108.json`, approximately 1.02 MiB, with source records, measurements
and generated geometry. Python is needed only for regeneration:

```powershell
python -m pip install -r tools/requirements-lidar.txt
python tools/import_lidar.py --inspect-only
python tools/import_lidar.py --source-crs EPSG:2180
```

The importer uses laspy/lazrs to decode 500,000-point chunks, PROJ for coordinate
transforms, SciPy for spatial queries and robust fitting, and Shapely for roof
clipping. Use the project's `.venv` interpreter with those packages installed.
`--corridor` changes the processing radius in metres (default 180). A full
import can take several minutes; raw points never become runtime scene nodes.

These files contain a quoted-empty WKT placeholder, so a source CRS must be
supplied explicitly. EPSG:2180 is spatially consistent with the existing NMT.
Classified ground is checked against the terrain before publishing output:
median absolute difference is 0.066 m over 208,459 sampled ground returns in
this import. This is an alignment check, not a guarantee of absolute survey
accuracy. No vertical datum conversion is performed.

Processing and rendering:

- Ground class 2 is used for alignment checks, building class 6 for roofs,
  and vegetation classes 4/5 for canopies. Noise, overlap class 12 and withheld
  points are excluded from scenery. RGB colours in the cloud are not used.
- Roof points are aggregated in 2 m cells and matched to OSM footprints.
  Buildings need at least 12 supported cells and 55% footprint-cell coverage.
  Robust gable fits are accepted only when sufficiently better than a flat
  roof; otherwise a measured median flat roof is used. Missing or rejected
  buildings retain their previous OSM-based appearance.
- Tree positions come from supported canopy-height peaks, with spacing and
  exclusion checks around mapped roads and buildings. Crown radii, trunk
  dimensions and species are procedural approximations. One canopy proxy is
  not necessarily one real tree, especially in continuous woodland.
- Repeated crown and trunk meshes are batched in 160 m areas using Godot
  MultiMesh, with a 900 m vegetation visibility distance. This adds volume
  and shadows without thousands of independently processed tree nodes.
- Driving ground and original building collision geometry are preserved.
  Trees are visual-only. Roof and facade visuals may therefore differ from
  the simplified collision envelope; this is not a rooftop exploration game.

Reimport LiDAR after regenerating terrain or changing OSM footprints/route
geometry. The runtime rejects a changed terrain origin or terrain reference
and falls back to the old buildings without LiDAR vegetation. The 2018 point
cloud is not present-day evidence of every tree or building. Keep GUGiK source
attribution and OSM/ODbL attribution and obligations for derived footprints
when distributing the generated data; exclude the large survey originals.

## Detailed Landmarks

Individual buildings can override the lightweight city model without rebuilding
the full corridor. Selections live in `data/landmark_selections.json`, separate
from generated mesh data. Each entry needs a unique lowercase `key`, a name,
and a coordinate inside the desired building. The shared CRS is EPSG:2180;
coordinates are explicitly **northing, easting**, matching the supplied values.
Use the same Python environment/dependencies as the LiDAR importer:

```powershell
python tools/import_landmark.py --list
python tools/import_landmark.py --select forum
python tools/import_landmark.py
```

`--list` resolves the coordinates without decoding LAZ. `--select` builds one
named landmark (repeat the option for several). With no selection, all configured
entries are processed. `--inspect-only` checks native point density and coverage
without publishing models. `--force` rebuilds even if unchanged. Coordinate-based
commands remain supported, for example:

```powershell
python tools/import_landmark.py --northing 720639.46 --easting 476850.34 --source-crs EPSG:2180
```

To add another building, append an entry like this to the configuration's
`landmarks` array, using its actual coordinate and a new key:

```json
{
  "key": "forum",
  "name": "Forum Gdansk",
  "northing": 720639.46,
  "easting": 476850.34,
  "expected_osm_id": "569238861"
}
```

`expected_osm_id` is optional but recommended after inspecting `--list`; it
prevents a changed map from silently targeting the wrong building. A per-entry
`source_crs` can override the shared setting. Unknown keys, duplicate keys and
nonfinite coordinates are rejected. No renderer or test edits are needed when
adding selections. The runtime accepts existing mapped buildings even when
they were absent from the coarse LiDAR set, with the same terrain alignment
checks and fallback to the coarse appearance for stale overrides.

The importer skips unchanged outputs using a fingerprint of the selection,
footprint, importer code, terrain data and source file names/sizes/modification
times, plus a checksum of the generated texture. Source timestamps are used
instead of hashing the entire raw LAZ collection; use `--force` after replacing
files while preserving their size and timestamps. Legacy entries without a
fingerprint rebuild once. Only intersecting LAZ tiles are decoded.

Entries are published by OSM ID. Reimporting one retains other landmark records;
failed builds keep the last published entry, and texture publication is rolled
back if the registry update fails. A batch continues independent selections
and returns failure if any fail. Removing an entry from the selection list does
not silently delete an already generated override. Reimport after changing
terrain or building footprints. Unselected city models and collisions remain
unchanged.

Current selections:

| Key | Building | Northing | Easting | OSM ID |
| --- | --- | --- | --- | --- |
| station | Gdansk Glowny station | 721345.37 | 476906.75 | 60147322 |
| forum | Forum Gdansk | 720639.46 | 476850.34 | 569238861 |

Forum's matched footprint is approximately 8,507 m2. Its model uses 118,071
building returns from two 2018 survey tiles, with 0.148 m median XY spacing.
It has 119,687 vertices and 237,065 triangles, retaining over 99.99% of the
footprint area, and a 332 x 755 native 25 cm texture from 2021-09-08 imagery.
Forum is mapped as several separate buildings: this selection upgrades the
named mall building containing the coordinate. Adjacent parking and retail
volumes keep their existing models until explicitly selected. Neither the
survey nor this model guarantees present-day details; raw scan spikes and
procedural facades remain visible.

For the station, the extraction reads only the intersecting LAZ tile and retains
56,658 non-withheld class-6 returns inside the footprint, above the local ground.
The source is the 2018 survey, with about 16.9 returns per square metre and a
median nearest-neighbour XY spacing of 0.136 m. Point spacing is not survey
accuracy, and density varies over the roof. There is no 2 m aggregation, mesh
decimation, height smoothing or flat/gable fitting in this override.

The native roof mesh contains 58,238 vertices and 114,848 triangles, compared
with the former flat roof's 843 aggregated cells and 143 triangles. Duplicate
XY locations (37 returns) retain the highest elevation; 1,617 boundary vertices
use nearby return heights to close the footprint edge. Delaunay triangles are
clipped to the footprint and rejected if an XY edge exceeds 2 m, preventing
large unsupported gaps from being filled. More than 99.99% of the footprint
is covered. Exported coordinates retain millimetre precision, not millimetre
measurement accuracy.

Outputs are `data/landmarks_108.json` (about 10.36 MiB for both current landmarks)
and `data/landmark_<osm_id>.png` per building. The station texture is a native
25 cm orthophoto crop (207 x 423 pixels, 2021-07-13 imagery); Forum's is
`data/landmark_569238861.png`. The coarse `data/lidar_108.json` and source LAZ/TIFF files
are not modified. The original selected points are cached for inspection in
`.cache/landmark_60147322_source.npz`. Include the landmark JSON and raw PNG
bytes when configuring a distributed build; exported builds are not yet tested.

This is a **2.5D roof envelope**, not a fully scanned 3D architectural model.
It preserves the station's complex roofline and tower height, but raw returns
also produce jagged edges and spikes. Vertical surfaces, overhangs, clocks,
occluded areas and accurate facade ornamentation cannot be recovered by this
method. Facades still use generated windows/materials and walls extruded from
the footprint. The scan and imagery predate the present-day service. Existing
collision geometry is unchanged. This high-detail option is intended for a
few landmarks, not every building along the route.

## Checks

Substitute your Godot executable path for `godot`:

```powershell
godot --headless --path . --editor --import --quit
godot --headless --path . --script tests/smoke.gd
godot --path . --script tests/visual.gd
python -m unittest discover -s tests -p test_lidar.py -v
```

Smoke tests cover both directions, monotonic stop progress, boarding,
door interlocks, acceleration, recovery and service completion, ground support
at all 25 stops, terrain/collision agreement at 1,637 lane samples, and short
hill drives. `-- --ground-only` or `-- --terrain-only` runs a focused subset.
The full suite also checks LiDAR counts/alignment in both directions and that
the scenery layer introduces no new collision objects.
Visual tests capture chase, cab and smaller-window views in `.cache`, check
imagery UV alignment, reject shifted metadata, compare textured/plain pixels,
verify that the O toggle does not change terrain heights, and capture the
newly covered Chelm terminus in `chelm.png`. The bundled coverage report must
include all route shape points and stops. They require a
display and graphics driver.
LiDAR visual checks verify imported building/tree instance counts and compare
the hill view with vegetation hidden (`.cache/lidar_without_trees.png`). Python
tests cover flat/gabled roof fitting, footprint containment, terrain sampling
and duplicate COPC handling, plus native landmark source-height preservation
and rejection of unsupported gaps, plus named selection, identity guards,
incremental skipping and publication rollback. Visual tests iterate the generated
registry, check every landmark's mesh counts and texture, and verify that all
unselected building records are unchanged. They frame each model automatically
and capture `.cache/<key>_detail.png` and `.cache/<key>_street.png` without the
HUD, including `forum_detail.png` and `forum_street.png`. Smoke tests check that
all generated landmarks load exactly once in both route directions.

Suggested next milestones: road grading and bridge decks; a detailed bus
asset and vehicle physics; traffic and junction signals; timetable/passenger
simulation; then city landmarks and route-specific scenery.