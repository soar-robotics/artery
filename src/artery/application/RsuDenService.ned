//
// Artery V2X Simulation Framework
// Licensed under GPLv2, see COPYING file for detailed license and warranty terms.
//

package artery.application;

simple RsuDenService like ItsG5Service
{
    parameters:

        @class(RsuDenService);
        @signal[DenmSent](type=artery::DenmObject);
        @signal[DenmReceived](type=artery::DenmObject);

        // CAM generation interval
        double generationInterval @unit(s) = default(1.0s);

        @statistic[reception](source=DenmReceived; record=count,vector(denmActionId)?,vector(denmCauseCode)?);
        @statistic[transmission](source=DenmSent; record=count,vector(denmActionId)?,vector(denmCauseCode)?);

        //xml useCases;
}
