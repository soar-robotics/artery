#ifndef ARTERY_LANCHANGEMODE_H_
#define ARTERY_LANCHANGEMODE_H_

#include "artery/storyboard/Effect.h"

namespace artery
{

/**
 * SpeedEffect
 * Changes the cars speed according to m_speed
 * Resets the speed to the previous with removing the Effect
 */
class LaneChangeMode : public Effect
{
public:
    /**
     * Creates a new Speed Effect
     * \param car Affected vehicle
     * \param Speed in m/s
     * \param Story from which the Effect is created
     */
    LaneChangeMode(Vehicle& car, int mode, Story& story) :
        Effect(story, car), mMode(mode)
    {
    }

    /**
     * Saves the current speed in m_current and changes the speed of m_car according to m_speed
     */
    void applyEffect();

    /**
     * Changes back m_cars speed to m_current
     */
    void removeEffect();

    /*
     * Calls apply Effect
     */
    void reapplyEffect();

private:
    int mMode;
};

} // namespace artery

#endif /* ARTERY_LANCHANGEMODE_H_ */
