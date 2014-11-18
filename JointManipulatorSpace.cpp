#include "JointManipulatorSpace.h"
#include <algorithm>
#include <limits>
#include <cmath>
#include "ompl/tools/config/MagicConstants.h"
#include <boost/math/constants/constants.hpp>

// Define for boost version < 1.47
#ifndef BOOST_ASSERT_MSG
#define BOOST_ASSERT_MSG(expr, msg) assert(expr)
#endif


void ompl::base::JointManipulatorSampler::sampleUniform(State *state)
{
    state->as<JointManipulatorSpace::StateType>()->phis = malloc(sizeof(double) * space_->as<JointManipulatorSpace>()->numberOfJoints);
    state->as<JointManipulatorSpace::StateType>()->velocities = malloc(sizeof(double) * space_->as<JointManipulatorSpace>()->numberOfJoints);
        // rng_.uniformReal(-boost::math::constants::pi<double>(), boost::math::constants::pi<double>());
}

void ompl::base::JointManipulatorSampler::sampleUniformNear(State *state, const State *near, const double distance)
{
    state->as<JointManipulatorSpace::StateType>()->phis = malloc(sizeof(double) * space_->as<JointManipulatorSpace>()->numberOfJoints);
    state->as<JointManipulatorSpace::StateType>()->velocities = malloc(sizeof(double) * space_->as<JointManipulatorSpace>()->numberOfJoints);
    // rng_.uniformReal(near->as<JointManipulatorSpace::StateType>()->value - distance,
    //                                                                    near->as<JointManipulatorSpace::StateType>()->value + distance);
    space_->enforceBounds(state);
}

void ompl::base::JointManipulatorSampler::sampleGaussian(State *state, const State *mean, const double stdDev)
{
    state->as<JointManipulatorSpace::StateType>()->phis = malloc(sizeof(double) * space_->as<JointManipulatorSpace>()->numberOfJoints);
    state->as<JointManipulatorSpace::StateType>()->velocities = malloc(sizeof(double) * space_->as<JointManipulatorSpace>()->numberOfJoints);
    // rng_.gaussian(mean->as<JointManipulatorSpace::StateType>()->value, stdDev);
    space_->enforceBounds(state);
}

unsigned int ompl::base::JointManipulatorSpace::getDimension() const
{
    return numberOfJoints;
}

double ompl::base::JointManipulatorSpace::getMaximumExtent() const
{
    return boost::math::constants::pi<double>();
}

double ompl::base::JointManipulatorSpace::getMeasure() const
{
    return 2.0 * boost::math::constants::pi<double>();
}

void ompl::base::JointManipulatorSpace::enforceBounds(State *state) const
{
    // double v = fmod(state->as<StateType>()->value, 2.0 * boost::math::constants::pi<double>());
    // if (v <= -boost::math::constants::pi<double>())
    //     v += 2.0 * boost::math::constants::pi<double>();
    // else
    //     if (v > boost::math::constants::pi<double>())
    //         v -= 2.0 * boost::math::constants::pi<double>();
    // state->as<StateType>()->value = v;
}

bool ompl::base::JointManipulatorSpace::satisfiesBounds(const State *state) const
{
    // return (state->as<StateType>()->value <= boost::math::constants::pi<double>()) &&
    //        (state->as<StateType>()->value > -boost::math::constants::pi<double>());
    return true;
}



unsigned int ompl::base::JointManipulatorSpace::getSerializationLength() const
{
    return sizeof(double);
}

void ompl::base::JointManipulatorSpace::serialize(void *serialization, const State *state) const
{
    memcpy(serialization, &state->as<StateType>()->value, sizeof(double));
}

void ompl::base::JointManipulatorSpace::deserialize(State *state, const void *serialization) const
{
    memcpy(&state->as<StateType>()->value, serialization, sizeof(double));
}

double ompl::base::JointManipulatorSpace::distance(const State *state1, const State *state2) const
{
    // assuming the states 1 & 2 are within bounds
    double d = fabs(state1->as<StateType>()->value - state2->as<StateType>()->value);
    BOOST_ASSERT_MSG(satisfiesBounds(state1) && satisfiesBounds(state2),
        "The states passed to SO2StateSpace::distance are not within bounds. Call "
        "SO2StateSpace::enforceBounds() in, e.g., ompl::control::ODESolver::PostPropagationEvent, "
        "ompl::control::StatePropagator, or ompl::base::StateValidityChecker");
    return (d > boost::math::constants::pi<double>()) ? 2.0 * boost::math::constants::pi<double>() - d : d;
}

bool ompl::base::JointManipulatorSpace::equalStates(const State *state1, const State *state2) const
{
    return fabs(state1->as<StateType>()->value - state2->as<StateType>()->value) < std::numeric_limits<double>::epsilon() * 2.0;
}

void ompl::base::JointManipulatorSpace::interpolate(const State *from, const State *to, const double t, State *state) const
{
    double diff = to->as<StateType>()->value - from->as<StateType>()->value;
    if (fabs(diff) <= boost::math::constants::pi<double>())
        state->as<StateType>()->value = from->as<StateType>()->value + diff * t;
    else
    {
        double &v = state->as<StateType>()->value;
        if (diff > 0.0)
            diff = 2.0 * boost::math::constants::pi<double>() - diff;
        else
            diff = -2.0 * boost::math::constants::pi<double>() - diff;
        v = from->as<StateType>()->value - diff * t;
        // input states are within bounds, so the following check is sufficient
        if (v > boost::math::constants::pi<double>())
            v -= 2.0 * boost::math::constants::pi<double>();
        else
            if (v < -boost::math::constants::pi<double>())
                v += 2.0 * boost::math::constants::pi<double>();
    }
}

ompl::base::StateSamplerPtr ompl::base::JointManipulatorSpace::allocDefaultStateSampler() const
{
    return StateSamplerPtr(new JointManipulatorSampler(this));
}

ompl::base::State* ompl::base::JointManipulatorSpace::allocState() const
{
    return new StateType();
}

void ompl::base::JointManipulatorSpace::freeState(State *state) const
{
    delete static_cast<StateType*>(state);
}

void ompl::base::JointManipulatorSpace::registerProjections()
{
    class JointManipulatorDefaultProjection : public ProjectionEvaluator
    {
    public:

        JointManipulatorDefaultProjection(const StateSpace *space) : ProjectionEvaluator(space)
        {
        }

        virtual unsigned int getDimension() const
        {
            return 1;
        }

        virtual void defaultCellSizes()
        {
            cellSizes_.resize(1);
            cellSizes_[0] = boost::math::constants::pi<double>() / magic::PROJECTION_DIMENSION_SPLITS;
            bounds_.resize(1);
            bounds_.low[0] = -boost::math::constants::pi<double>();
            bounds_.high[0] = boost::math::constants::pi<double>();
        }

        virtual void project(const State *state, EuclideanProjection &projection) const
        {
            projection(0) = state->as<JointManipulatorSpace::StateType>()->value;
        }
    };

    registerDefaultProjection(ProjectionEvaluatorPtr(dynamic_cast<ProjectionEvaluator*>(new JointManipulatorDefaultProjection(this))));
}




double* ompl::base::JointManipulatorSpace::getValueAddressAtIndex(State *state, const unsigned int index) const
{
    return index == 0 ? &(state->as<StateType>()->value) : NULL;
}




void ompl::base::JointManipulatorSpace::printState(const State *state, std::ostream &out) const
{
    out << "SO2State [";
    if (state)
        out << state->as<StateType>()->value;
    else
        out << "NULL";
    out << ']' << std::endl;
}



void ompl::base::JointManipulatorSpace::printSettings(std::ostream &out) const
{
    out << "Joint Manipulator state space '" << getName() << "'" << std::endl;
}


void ompl::base::JointManipulatorSpace::copyState(State *destination, const State *source) const
{
    destination->as<StateType>()->phis = source->as<StateType>()->phis;
    destination->as<StateType>()->velocities = source->as<StateType>()->velocities;
}
