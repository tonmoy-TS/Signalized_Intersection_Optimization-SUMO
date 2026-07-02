# TraCI4Matlab: `getNextSwitch()` / `getPhaseDuration()` type-mismatch fix

This harness relies on `traci.trafficlights.getNextSwitch()` and
`traci.trafficlights.getPhaseDuration()` (see `tl_utils/globalTLsInformation.m`)
to compute traffic-light timing. Both are affected by a long-standing
TraCI4Matlab bug: they return corrupted values because of an `int32` vs.
`double` type mismatch — documented and discussed in
[traci4matlab#8](https://github.com/pipeacosta/traci4matlab/issues/8).

## The bug

Both functions resolve through `traci.trafficlights.getUniversal()`, which
looks up a type-reader function for each SUMO variable ID in
`+traci/RETURN_VALUE_FUNC.m`. `TL_NEXT_SWITCH` and `TL_PHASE_DURATION` are
`double`-typed per the SUMO TraCI wire protocol, but are mapped to
`'readInt'`, silently corrupting the returned values.

## The fix

Change those two map entries from `'readInt'` to `'readDouble'` in
`+traci/RETURN_VALUE_FUNC.m`. `TL_CURRENT_PHASE` (a genuine integer) and
every other `trafficlights` getter are unaffected and untouched.

This is the root-cause fix — a 2-word change — rather than the wrapper-function
workaround (`getUniversalNextLightFix`) originally worked out and documented
in this project's report (Appendix B). Same underlying bug, smaller patch.

## Status

Submitted upstream as [traci4matlab#29](https://github.com/pipeacosta/traci4matlab/pull/29),
referencing issue #8, credited to the original diagnosis by
[@bjyurkovich](https://github.com/bjyurkovich) and confirmed/extended by
this project's author. **Not merged as of this writing** — apply the patch
below locally until it is.

## Applying it

From your local TraCI4Matlab checkout:

```bash
git apply /path/to/traci4matlab-trafficlights-fix.patch
```

Or manually: in `+traci/RETURN_VALUE_FUNC.m`, find the `trafficlights` map
and change its last two entries from `'readInt','readInt'` to
`'readDouble','readDouble'`.
