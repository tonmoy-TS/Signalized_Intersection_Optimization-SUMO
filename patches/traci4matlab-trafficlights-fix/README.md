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

## Fix 1: patch the type map (recommended)

Change those two map entries from `'readInt'` to `'readDouble'` in
`+traci/RETURN_VALUE_FUNC.m`. `TL_CURRENT_PHASE` (a genuine integer) and
every other `trafficlights` getter are unaffected and untouched. This is
the root-cause fix — a 2-word change. See [Applying it](#applying-it) below
for the ready-to-use patch file.

## Fix 2: alternative wrapper function

If you'd rather not touch the shared type-map, add a dedicated function
instead and call it from just the two affected getters.

Add `+traci/+trafficlights/getUniversalNextLightFix.m`:

```matlab
function returnedValue = getUniversalNextLightFix(varID, tlsID)
import traci.constants
global tlsSubscriptionResults

if isempty(tlsSubscriptionResults)
    returnValueFunc = traci.RETURN_VALUE_FUNC.trafficlights;
else
    returnValueFunc = tlsSubscriptionResults.valueFunc;
end

% Prepare the outgoing message and read the response. The result variable
% is a traci.Storage object
result = traci.sendReadOneStringCmd(constants.CMD_GET_TL_VARIABLE,varID,tlsID);

% Use the proper method to read the variable of interest from the result
returnedValue = result.readDouble(); % THIS IS THE FIXED LINE
```

Then update `+traci/+trafficlights/getNextSwitch.m`:

```matlab
function nextSwitch = getNextSwitch(tlsID)
import traci.constants
nextSwitch = traci.trafficlights.getUniversalNextLightFix(constants.TL_NEXT_SWITCH, tlsID);
```

And `+traci/+trafficlights/getPhaseDuration.m`:

```matlab
function phaseDuration = getPhaseDuration(tlsID)
import traci.constants
phaseDuration = traci.trafficlights.getUniversalNextLightFix(constants.TL_PHASE_DURATION, tlsID);
```

## Status

Fix 1 is submitted upstream as
[traci4matlab#29](https://github.com/pipeacosta/traci4matlab/pull/29),
referencing issue #8, credited to the original diagnosis by
[@bjyurkovich](https://github.com/bjyurkovich) and confirmed/extended by
this project's author. **Not merged as of this writing** — apply one of
the fixes above locally until it is.

## Applying it

From your local TraCI4Matlab checkout:

```bash
git apply /path/to/traci4matlab-trafficlights-fix.patch
```

Or manually: in `+traci/RETURN_VALUE_FUNC.m`, find the `trafficlights` map
and change its last two entries from `'readInt','readInt'` to
`'readDouble','readDouble'`.
