%% checkingTlinfo — Debug/validation script for tlsInformation
%
%   NOT USED in the main ECO-AND OCP pipeline.
%   NOT A FUNCTION — runs as a script in the caller's workspace.
%
%   Purpose: exhaustively calls the per-step TL query function tlsInformation
%   for every (phase, link-index) combination and stores [tg, tr, ty] into
%   checkingFn{ph, idn}. Used to cross-check results against the pre-computed
%   cache in globalTLsInformation / globalTLsInformation_Y.
%
%   Requires in workspace (set up by main_SUMO.m / globalTLsInformation):
%     tlsID              - TL ID string (e.g. 'jnCent')
%     tlAllRYgGStates    - {phase, tlsIndex} state chars (un-normalised)
%
%   Loop bounds:
%     idn = 1:16  → 16 controlled link indices (4 arms × 3 links + 4 pedestrian crossings)
%     ph  = 1:4   → 4 SUMO signal phases
%   Both are converted to 0-based for the TraCI call (ph-1, idn-1) because
%   tlsInformation follows SUMO's 0-based phase and link indexing internally.
%
%   NOTE: this call previously did not match tlsInformation.m's signature
%   (function [tg, tr, ty] = tlsInformation(tlsID, tlsPhase, tlsIndex, tlsState, tlsTimeToEnd)) —
%   it passed tlsRYGDefinition/tlscontrolledLinks positionally, which
%   tlsInformation does not accept as arguments (it queries them itself
%   internally). Fixed below. tlsTimeToEnd is passed as 0 since
%   tlsInformation currently hardcodes tlElapsedTime=0 internally anyway
%   (see the NOTE in tlsInformation.m), so this placeholder has no effect
%   on tg/tr/ty for this validation script's purposes.

for idn = 1:16
    for ph = 1:4
        [tg, tr, ty] = tlsInformation(tlsID, ph-1, idn-1, tlAllRYgGStates{ph, idn}, 0)
        checkingFn{ph, idn} = [tg, tr, ty];
    end
end