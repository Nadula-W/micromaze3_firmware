# v41 - one terminal-tunable Kp / Ki / Kd drive PID

This build is based on v40 Home 10x10 + ToF zero-reading fix.

The old terminal meaning `pid <straight> <wall> <gyroTurn>` is removed.
Now the command is literally:

```text
pid <Kp> <Ki> <Kd>
```

Example using the older values that worked well on this robot:

```text
pid 1.8 0 0.03
```

The same PID controls steering during `cell`, `distance`, DFS/explore and fast path movement. It combines encoder left/right progress mismatch with the calibrated side-wall centering error. There is no separate tunable wall-P gain fighting it.

Competition 90-degree turns remain the calibrated 378-tick encoder pivots; the PID command does not change that.

Important after flashing: because the NVS binary layout is intentionally preserved, the three OLD stored runtime gain numbers may load into the new Kp/Ki/Kd slots. Therefore run:

```text
status
pid 1.8 0 0.03
cell
```

If `cell` is smooth, then:

```text
savecal
```

Then test `distance 768 90`, followed by DFS/explore.
