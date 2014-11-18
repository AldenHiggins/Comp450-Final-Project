
#ifndef JOINT_MANIPULATOR_SPACE_H_
#define JOINT_MANIPULATOR_SPACE_H_

#include "ompl/base/StateSpace.h"

namespace ompl
{
    namespace base
    {

        /** \brief State space sampler for SO(2) */
        class JointManipulatorSampler : public StateSampler
        {
        public:

            /** \brief Constructor */
            JointManipulatorSampler(const StateSpace *space) : StateSampler(space)
            {
            }

            virtual void sampleUniform(State *state);
            virtual void sampleUniformNear(State *state, const State *near, const double distance);
            virtual void sampleGaussian(State *state, const State *mean, const double stdDev);
        };

        /** \brief A state space representing SO(2). The distance
            function and interpolation take into account angle
            wrapping. */
        class JointManipulatorSpace : public StateSpace
        {
        public:

            /** \brief The definition of a state in SO(2) */
            // TODO: add destructor
            class StateType : public State
            {
            public:

                /** \brief Set the state to identity -- no rotation (value = 0.0) */
                void setIdentity()
                {
                    value = 0.0;
                }

                /** \brief The value of the angle in the interval (-Pi, Pi] */
                double phis[];
                // Angular velocity
                double velocities[];
            };

            JointManipulatorSpace(int numberOfJointsInput) : StateSpace()
            {
                setName("JOINT" + getName());
                // TEMP : not sure if this is already being used, check later
                type_ = 42;
                numberOfJoints = numberOfJointsInput;
            }

            virtual ~JointManipulatorSpace()
            {
            }

            virtual unsigned int getDimension() const;

            virtual double getMaximumExtent() const;

            virtual double getMeasure() const;

            /** \brief Normalize the value of the state to the interval (-Pi, Pi] */
            virtual void enforceBounds(State *state) const;

            /** \brief Check if the value of the state is in the interval (-Pi, Pi] */
            virtual bool satisfiesBounds(const State *state) const;

            virtual void copyState(State *destination, const State *source) const;

            virtual unsigned int getSerializationLength() const;

            virtual void serialize(void *serialization, const State *state) const;

            virtual void deserialize(State *state, const void *serialization) const;

            virtual double distance(const State *state1, const State *state2) const;

            virtual bool equalStates(const State *state1, const State *state2) const;

            virtual void interpolate(const State *from, const State *to, const double t, State *state) const;

            virtual StateSamplerPtr allocDefaultStateSampler() const;

            virtual State* allocState() const;

            virtual void freeState(State *state) const;

            virtual double* getValueAddressAtIndex(State *state, const unsigned int index) const;

            virtual void printState(const State *state, std::ostream &out) const;

            virtual void printSettings(std::ostream &out) const;

            virtual void registerProjections();
            
            int numberOfJoints;
        private:
        };
    }
}

#endif 
//JOINT_MANIPULATOR_SPACE_H_
