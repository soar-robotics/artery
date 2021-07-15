package artery.application.den;

simple RoadWorkWarning like SuspendableUseCase
{
    @class(den::RoadWorkWarning);
    double sampleDuration @unit(s) = default(500ms);
    double sampleInterval @unit(s) = default(100ms);
    double speedThreshold @unit(mps) = default(60kmph);
    double decelerationThreshold = default(1.0);
    double detectionBlockingInterval @unit(s) = default(100ms);
}
