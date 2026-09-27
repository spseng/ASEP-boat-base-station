# Screenshots

Captured headless at 1280x800 against the simulator. Regenerate from the
repository root after building (`cmake --build build`); on a desktop you can
drop the `xvfb-run ...` prefix.

| File | Caption |
|---|---|
| `overview.png` | Default layout, connected to three wandering boats, boat 2 selected, session recording on. |
| `commands_teleop.png` | Boat 2 after a Disable: gate TRIPPED in the map, table and Commands panel; teleop on and targeting it, with the "will ignore commands" warning. |
| `plots.png` | Plots tab after 30 s: scalar, RSSI in both directions (dots: boat 1 hearing the base while teleop sends to it), SNR. |

The `--fresh`, `--select`, `--test-disable`, `--test-teleop` and `--focus`
options exist for screenshots and testing (see `asep_base_station --help`).
The simulator's `--time-scale 3` makes the boats move three times faster so the
trails are long enough to see.

## overview.png

```sh
./build/src/sim/asep_fake_base --seed 7 --start-mode auto --time-scale 3 --quiet --link /tmp/asep-sim > /dev/null &
xvfb-run -a -s "-screen 0 1600x1000x24" ./build/src/app/asep_base_station --fresh --port /tmp/asep-sim \
  --log-dir /tmp/asep-shots --select 2 --size 1280x800 \
  --screenshot docs/screenshots/overview.png --screenshot-after 18
kill %1
```

## commands_teleop.png

```sh
./build/src/sim/asep_fake_base --seed 7 --start-mode auto --time-scale 3 --quiet --link /tmp/asep-sim > /dev/null &
xvfb-run -a -s "-screen 0 1600x1000x24" ./build/src/app/asep_base_station --fresh --port /tmp/asep-sim \
  --no-record --select 2 --test-disable 2 --test-teleop 2 --size 1280x800 \
  --screenshot docs/screenshots/commands_teleop.png --screenshot-after 10
kill %1
```

## plots.png

```sh
./build/src/sim/asep_fake_base --seed 7 --start-mode auto --time-scale 3 --quiet --link /tmp/asep-sim > /dev/null &
xvfb-run -a -s "-screen 0 1600x1000x24" ./build/src/app/asep_base_station --fresh --port /tmp/asep-sim \
  --no-record --select 1 --test-teleop 1 --focus Plots --size 1280x800 \
  --screenshot docs/screenshots/plots.png --screenshot-after 30
kill %1
```
